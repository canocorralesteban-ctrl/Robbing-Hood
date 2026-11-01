#include "funciones.h"

#include <stdio.h>

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
static int s_norm[16];

float posicion_actual = 0;
static float ultima_posicion = 0;
uint32_t tiempo_sin_linea = 0;

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
    vel = limitar(vel, 0, VEL_TOPE);
    uint32_t duty = (uint32_t)(vel * ESCALA_BATERIA);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, canal, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, canal);
}

static void hardware_motores(int izq, int der)
{
    if (izq >= 0)
        gpio_set_level(PIN_DIR_IZQ, 1);
    else {
        gpio_set_level(PIN_DIR_IZQ, 0);
        izq = -izq;
    }
    poner_pwm(LEDC_CHANNEL_0, izq);

    if (der >= 0)
        gpio_set_level(PIN_DIR_DER, 1);
    else {
        gpio_set_level(PIN_DIR_DER, 0);
        der = -der;
    }
    poner_pwm(LEDC_CHANNEL_1, der);
}

void moverRobot(int vel_izq, int vel_der)
{
    hardware_motores(-vel_izq, vel_der);
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

void calibrar(void)
{
    for (int i = 0; i < 16; i++) {
        s_min[i] = 4095;
        s_max[i] = 0;
    }

    printf("CALIBRANDO...\n");
    uint32_t t0 = millis();
    while (millis() - t0 < T_CALIBRAR_MS) {
        for (int i = 0; i < 16; i++) {
            int lectura = leer_canal(i);
            if (lectura < s_min[i]) s_min[i] = lectura;
            if (lectura > s_max[i]) s_max[i] = lectura;
        }
        vTaskDelay(1);
    }
    printf("LISTO\n");
}

int leerSensores(void)
{
    int suma_p = 0;
    int suma = 0;
    int detectados = 0;
    int ultimo_activo = -1;
    int gap_max = 0;

    for (int i = 0; i < 16; i++) {
        int raw = leer_canal(i);

        int val = 0;
        if (s_max[i] > s_min[i])
            val = (raw - s_min[i]) * 1000 / (s_max[i] - s_min[i]);
        val = limitar(val, 0, 1000);
        s_norm[i] = val;

        if (val > UMBRAL_LINEA) {
            if (ultimo_activo >= 0) {
                int gap = i - ultimo_activo;
                if (gap > gap_max) gap_max = gap;
            }
            ultimo_activo = i;

            int peso = (int)((i - 7.5f) * 1000);
            suma_p += peso * val;
            suma += val;
            detectados++;
        }
    }

    if (detectados >= BIF_DETECTADOS && gap_max >= BIF_HUECO) {
        suma_p = 0;
        suma = 0;
        for (int i = 0; i < 16; i++) {
            if (s_norm[i] <= UMBRAL_LINEA) continue;
            if (ultima_posicion <= 0 && i >= 8) continue;
            if (ultima_posicion > 0 && i < 8) continue;

            int peso = (int)((i - 7.5f) * 1000);
            suma_p += peso * s_norm[i];
            suma += s_norm[i];
        }
        if (suma == 0) {
            posicion_actual = ultima_posicion;
            return 1;
        }
        posicion_actual = suma_p / suma;
        ultima_posicion = posicion_actual;
        tiempo_sin_linea = millis();
        return 1;
    }

    if (detectados > 0) {
        posicion_actual = suma_p / suma;
        ultima_posicion = posicion_actual;
        tiempo_sin_linea = millis();
        return 1;
    }

    if (millis() - tiempo_sin_linea < T_PERDIDA_MS) {
        posicion_actual = ultima_posicion;
        return 1;
    }
    if (ultima_posicion > 0)
        moverRobot(VEL_BUSQUEDA, -VEL_BUSQUEDA);
    else
        moverRobot(-VEL_BUSQUEDA, VEL_BUSQUEDA);
    return 0;
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
        if (cerca(alto_us, 1690))
            trama |= (1UL << bit);
        else if (!cerca(alto_us, 560))
            return false;
    }
    codigo_ir = trama;
    return true;
}

bool ir_start(void)
{
    size_t cantidad = 0;
    if (xQueueReceive(cola_ir, &cantidad, 0) != pdTRUE)
        return false;

    bool trama_valida = decodificar_nec(cantidad);
    escuchar_ir();
    if (!trama_valida)
        return false;

    uint8_t direccion = codigo_ir & 0xFF;
    uint8_t comando = (codigo_ir >> 16) & 0xFF;
    printf("IR: direccion=0x%02X comando=0x%02X\n", direccion, comando);

    if (comando != IR_COMANDO)
        return false;
    if (millis() - ultimo_ir < T_ANTIRREBOTE_MS)
        return false;
    ultimo_ir = millis();
    return true;
}
