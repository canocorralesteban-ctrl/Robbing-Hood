#include "funciones.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/rmt_rx.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static int s_min[16];
static int s_max[16];
static int umbral[16];
bool calibrado = false;

bool linea_perdida = false;
uint32_t t_go = 0;
static uint32_t t_linea = 0;
static int pos_filtrada = SETPOINT;
static float last_prop = 0;
static float derivada_filtrada = 0;
static int vel_base = VEL_CURVA;

bool turbina_encendida = false;
static uint32_t t_turbina = 0;
static int turbina_us = TURBINA_APAGADA_US;

static adc_oneshot_unit_handle_t adc;

static rmt_channel_handle_t canal_ir;
static QueueHandle_t cola_ir;
static rmt_symbol_word_t simbolos[64];
static uint32_t codigo_ir;
static uint32_t ultimo_ir = 0;

uint32_t millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

int limitar(int valor, int minimo, int maximo)
{
    if (valor < minimo)
        return minimo;
    if (valor > maximo)
        return maximo;
    return valor;
}

float limitar_f(float valor, float minimo, float maximo)
{
    if (valor < minimo)
        return minimo;
    if (valor > maximo)
        return maximo;
    return valor;
}

static int escalar(float x, float de_min, float de_max, int a_min, int a_max)
{
    return a_min + (x - de_min) * (a_max - a_min) / (de_max - de_min);
}

static void salida(int pin)
{
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
}

static void canal_pwm(ledc_channel_t canal, ledc_timer_t timer, int pin, uint32_t duty)
{
    ledc_channel_config_t config = {
        .gpio_num = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = canal,
        .timer_sel = timer,
        .duty = duty,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&config));
}

void motores_init(void)
{
    salida(PIN_DIR_IZQ);
    salida(PIN_DIR_DER);
    gpio_set_level(PIN_DIR_IZQ, 0);
    gpio_set_level(PIN_DIR_DER, 0);

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .clk_cfg = LEDC_USE_APB_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    canal_pwm(LEDC_CHANNEL_0, LEDC_TIMER_0, PIN_PWM_IZQ, 0);
    canal_pwm(LEDC_CHANNEL_1, LEDC_TIMER_0, PIN_PWM_DER, 0);
}

static void poner_pwm(ledc_channel_t canal, int vel)
{
    uint32_t duty = limitar(vel * ESCALA_BATERIA, 0, 255);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, canal, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, canal);
}

static void hardware_motores(int izq, int der)
{
    gpio_set_level(PIN_DIR_IZQ, izq >= 0);
    poner_pwm(LEDC_CHANNEL_0, abs(izq));

    gpio_set_level(PIN_DIR_DER, der >= 0);
    poner_pwm(LEDC_CHANNEL_1, abs(der));
}

void moverRobot(int vel_izq, int vel_der)
{
    hardware_motores(vel_izq, -vel_der);
}

void detenerMotores(void)
{
    moverRobot(0, 0);
    vel_base = VEL_CURVA;
    last_prop = 0;
    derivada_filtrada = 0;
    t_go = millis();
    t_linea = millis();
}

void turbina_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 50,
        .clk_cfg = LEDC_USE_APB_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    canal_pwm(LEDC_CHANNEL_2, LEDC_TIMER_1, PIN_ESC, TURBINA_APAGADA_US * 16384 / 20000);
}

void turbina(uint16_t us)
{
    us = limitar(us, TURBINA_APAGADA_US, TURBINA_US_MAX);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, us * 16384 / 20000);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

void turbina_encender(bool encender)
{
    turbina_encendida = encender;
    turbina_us = TURBINA_US_ARRANQUE;
    t_turbina = millis();
}

void actualizar_turbina(void)
{
    if (!turbina_encendida) {
        turbina(TURBINA_APAGADA_US);
        return;
    }
    turbina(turbina_us);
    if (turbina_us >= TURBINA_US_POTENCIA) return;
    if (millis() - t_turbina < T_TURBINA_PASO_MS) return;
    t_turbina = millis();
    turbina_us = limitar(turbina_us + TURBINA_PASO_US, 0, TURBINA_US_POTENCIA);
}

void sensores_init(void)
{
    salida(PIN_S0);
    salida(PIN_S1);
    salida(PIN_S2);
    salida(PIN_S3);

    adc_oneshot_unit_init_cfg_t config_adc = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&config_adc, &adc));

    adc_oneshot_chan_cfg_t config_canal = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, ADC_CHANNEL_1, &config_canal));
}

static int leer_canal(int canal)
{
    gpio_set_level(PIN_S0, (canal >> 0) & 1);
    gpio_set_level(PIN_S1, (canal >> 1) & 1);
    gpio_set_level(PIN_S2, (canal >> 2) & 1);
    gpio_set_level(PIN_S3, (canal >> 3) & 1);

    esp_rom_delay_us(5);

    int lectura = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc, ADC_CHANNEL_1, &lectura));
    return lectura;
}

static void reiniciar_calibracion(void)
{
    for (int i = 0; i < 16; i++) {
        s_min[i] = 4095;
        s_max[i] = 0;
    }
}

static void muestrear(uint32_t ms)
{
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        for (int i = 0; i < 16; i++) {
            int lectura = leer_canal(i);
            if (lectura < s_min[i]) s_min[i] = lectura;
            if (lectura > s_max[i]) s_max[i] = lectura;
        }
        vTaskDelay(1);
    }
}

void calibrar_auto(void)
{
    reiniciar_calibracion();
    printf("CALIBRANDO AUTO...\n");
    moverRobot(-VEL_CALIBRAR, VEL_CALIBRAR);
    muestrear(T_CALIBRAR_GIRO_MS);
    moverRobot(VEL_CALIBRAR, -VEL_CALIBRAR);
    muestrear(2 * T_CALIBRAR_GIRO_MS);
    moverRobot(-VEL_CALIBRAR, VEL_CALIBRAR);
    muestrear(T_CALIBRAR_GIRO_MS + T_CALIBRAR_EXTRA_MS);
    moverRobot(0, 0);
    for (int i = 0; i < 16; i++) {
        umbral[i] = (s_min[i] + s_max[i]) / 2;
        printf("%d: min %d max %d umbral %d\n", i, s_min[i], s_max[i], umbral[i]);
    }
    calibrado = true;
    printf("LISTO\n");
}

void lecturaLinea(void)
{
    int sumap = 0;
    int suma = 0;

    for (int i = 0; i < 16; i++) {
        int activo = leer_canal(i) > umbral[i] && i != SENSOR_OMITIDO;
        sumap += activo * (15 - i) * 100;
        suma += activo;
    }

    if (suma == 0) {
        linea_perdida = millis() - t_linea >= T_PERDIDA_MS;
        return;
    }

    t_linea = millis();
    linea_perdida = false;
    if (suma > MAX_ACTIVOS) return;

    pos_filtrada = FILTRO_POS * (sumap / suma) + (1 - FILTRO_POS) * pos_filtrada;
}

void recuperarLinea(void)
{
    if (pos_filtrada > SETPOINT) {
        moverRobot(-VEL_RECUPERACION, VEL_RECUPERACION);
        return;
    }
    moverRobot(VEL_RECUPERACION, -VEL_RECUPERACION);
}

static int calcularBaseDinamica(float error_abs)
{
    error_abs = limitar_f(error_abs, 0, ERROR_CURVA);
    int base = escalar(error_abs, 0, ERROR_CURVA, VEL_RECTA, VEL_CURVA);

    float deriv_abs = fabsf(derivada_filtrada);
    if (deriv_abs > 8) {
        float factor_error = limitar_f(error_abs / 300, 0, 1);
        int freno_extra = escalar(deriv_abs, 8, 70, 0, 70) * factor_error;
        if (deriv_abs > 15) freno_extra += escalar(deriv_abs, 15, 100, 0, 40);
        base -= limitar(freno_extra, 0, 90);
    }

    return limitar(base, VEL_CURVA, VEL_RECTA);
}

static void actualizarRampa(float error_abs)
{
    int rampa_bajada = RAMPA_BAJADA;
    if (error_abs > 100) rampa_bajada = RAMPA_BAJADA_MEDIA;
    if (error_abs > 200) rampa_bajada = RAMPA_BAJADA_FUERTE;

    int objetivo = calcularBaseDinamica(error_abs);
    vel_base += limitar(objetivo - vel_base, -rampa_bajada, RAMPA_SUBIDA);
}

void controlPID(void)
{
    float proporcional = pos_filtrada - SETPOINT;
    float error_abs = fabsf(proporcional);

    if (error_abs < ZONA_MUERTA) {
        actualizarRampa(error_abs);
        moverRobot(vel_base, vel_base);
        last_prop = proporcional;
        return;
    }

    derivada_filtrada += FILTRO_D * ((proporcional - last_prop) - derivada_filtrada);
    float derivativo = limitar_f(derivada_filtrada, -DERIVADA_MAX, DERIVADA_MAX);
    int diferencial = limitar(KP * proporcional + KD * derivativo, -SALIDA_MAX, SALIDA_MAX);
    last_prop = proporcional;

    actualizarRampa(error_abs);

    int vel_izq = vel_base - diferencial;
    int vel_der = vel_base + diferencial;

    int mayor = vel_izq;
    if (vel_der > mayor) mayor = vel_der;
    if (mayor > VEL_PWM_MAX) {
        vel_izq -= mayor - VEL_PWM_MAX;
        vel_der -= mayor - VEL_PWM_MAX;
    }

    if (millis() - t_go < T_BLOQUEO_REVERSA_MS) {
        if (vel_izq < 0) vel_izq = 0;
        if (vel_der < 0) vel_der = 0;
    }

    moverRobot(vel_izq, vel_der);
}

void probar_sensores(void)
{
    while (1) {
        int raw[16];
        for (int i = 0; i < 16; i++)
            raw[i] = leer_canal(i);
        for (int i = 0; i < 16; i++)
            printf("%5d", raw[i]);
        printf("  |  ");
        for (int i = 0; i < 16; i++)
            printf("%d", raw[i] > umbral[i]);
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static bool al_recibir_ir(rmt_channel_handle_t canal, const rmt_rx_done_event_data_t *datos, void *contexto)
{
    BaseType_t despertar_tarea = pdFALSE;
    xQueueSendFromISR(cola_ir, &datos->num_symbols, &despertar_tarea);
    return despertar_tarea == pdTRUE;
}

static rmt_receive_config_t config_recepcion = {
    .signal_range_min_ns = 1250,
    .signal_range_max_ns = 12000000,
};

static void escuchar_ir(void)
{
    ESP_ERROR_CHECK(rmt_receive(canal_ir, simbolos, sizeof(simbolos), &config_recepcion));
}

void ir_init(void)
{
    rmt_rx_channel_config_t config_canal = {
        .gpio_num = PIN_IR,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .mem_block_symbols = 48,
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&config_canal, &canal_ir));

    cola_ir = xQueueCreate(1, sizeof(size_t));

    rmt_rx_event_callbacks_t callbacks = {
        .on_recv_done = al_recibir_ir,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(canal_ir, &callbacks, NULL));
    ESP_ERROR_CHECK(rmt_enable(canal_ir));
    escuchar_ir();
}

static bool cerca(int medido, int esperado)
{
    return (medido > esperado - 200) && (medido < esperado + 200);
}

static bool decodificar_nec(size_t cantidad)
{
    if (cantidad != 34)
        return false;
    if (!cerca(simbolos[0].duration0, 9000) || !cerca(simbolos[0].duration1, 4500))
        return false;

    uint32_t trama = 0;
    for (int bit = 0; bit < 32; bit++) {
        int alto_us = simbolos[bit + 1].duration1;
        if (cerca(alto_us, 1690)) {
            trama |= (1UL << bit);
            continue;
        }
        if (!cerca(alto_us, 560))
            return false;
    }
    codigo_ir = trama;
    return true;
}

int ir_boton(void)
{
    size_t cantidad = 0;
    if (xQueueReceive(cola_ir, &cantidad, 0) != pdTRUE)
        return -1;

    bool trama_valida = decodificar_nec(cantidad);
    escuchar_ir();
    if (!trama_valida)
        return -1;

    uint8_t direccion = codigo_ir & 0xFF;
    uint8_t comando = (codigo_ir >> 16) & 0xFF;
    printf("IR: direccion=0x%02X comando=0x%02X\n", direccion, comando);

    if (millis() - ultimo_ir < T_ANTIRREBOTE_MS)
        return -1;
    ultimo_ir = millis();
    return comando;
}
