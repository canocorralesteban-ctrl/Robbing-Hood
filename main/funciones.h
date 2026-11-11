#ifndef FUNCIONES_H
#define FUNCIONES_H

#include <stdbool.h>
#include <stdint.h>

#define PIN_COM         1
#define PIN_S0          0
#define PIN_S1          2
#define PIN_S2          3
#define PIN_S3          4
#define PIN_DIR_IZQ     20
#define PIN_PWM_IZQ     21
#define PIN_DIR_DER     6
#define PIN_PWM_DER     5
#define PIN_ESC         8
#define PIN_IR          10

#define ESCALA_BATERIA  0.667f
#define VEL_PWM_MAX     (int)(255 / ESCALA_BATERIA)

#define VEL_RECTA         220
#define VEL_CURVA         190
#define VEL_RECUPERACION  220
#define KP                0.12f
#define KD                4.0f
#define FILTRO_D          0.25f
#define DERIVADA_MAX      180
#define SALIDA_MAX        120
#define ZONA_MUERTA       25
#define ERROR_CURVA       800
#define RAMPA_SUBIDA      3
#define RAMPA_BAJADA      10
#define RAMPA_BAJADA_MEDIA  17
#define RAMPA_BAJADA_FUERTE 22

#define SETPOINT          750
#define FILTRO_POS        0.88f
#define MAX_ACTIVOS       12
#define SENSOR_OMITIDO    10
#define T_PERDIDA_MS      20
#define T_BLOQUEO_RECUPERACION_MS 180
#define T_BLOQUEO_REVERSA_MS      220

#define VEL_CALIBRAR    60
#define T_CALIBRAR_GIRO_MS 300
#define T_CALIBRAR_EXTRA_MS 40
#define TS_MS           1

#define TURBINA_APAGADA_US  1000
#define TURBINA_US_MAX      2000
#define TURBINA_US_ARRANQUE 1200
#define TURBINA_US_POTENCIA 1700
#define TURBINA_PASO_US     10
#define T_TURBINA_PASO_MS   100

#define IR_BOTON_MOTORES 0x45
#define IR_BOTON_TURBINA 0x46
#define IR_BOTON_CALIBRAR 0x47
#define T_ANTIRREBOTE_MS 500

extern bool calibrado;
extern bool linea_perdida;
extern uint32_t t_go;
extern bool turbina_encendida;

uint32_t millis(void);
int limitar(int valor, int minimo, int maximo);
float limitar_f(float valor, float minimo, float maximo);

void motores_init(void);
void moverRobot(int vel_izq, int vel_der);
void detenerMotores(void);

void turbina_init(void);
void turbina(uint16_t us);
void turbina_encender(bool encender);
void actualizar_turbina(void);

void sensores_init(void);
void calibrar_auto(void);
void lecturaLinea(void);
void recuperarLinea(void);
void controlPID(void);
void probar_sensores(void);

void ir_init(void);
int ir_boton(void);

#endif
