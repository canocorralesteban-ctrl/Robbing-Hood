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

#define VEL_MAXIMA      220
#define VEL_MINIMA      180
#define VEL_BUSQUEDA    90
#define KP              0.07f
#define KD              3.0f
#define ALFA_D          0.7f
#define K_FRENO         0.25f
#define ACELERACION     0.5f
#define FRENADO         2.0f
#define LIMITE_BASE     45
#define LIMITE_PEND     0.015f
#define LIMITE_MAX      90
#define UMBRAL_LINEA    500
#define SENSOR_OMITIDO  10
#define BIF_DETECTADOS  4
#define BIF_HUECO       4
#define T_PERDIDA_MS    150
#define VEL_CALIBRAR    60
#define T_CALIBRAR_GIRO_MS 300
#define T_CALIBRAR_EXTRA_MS 40
#define TS_MS           2

#define TURBINA_APAGADA_US  1000
#define TURBINA_US_MAX      2000
#define TURBINA_US_ARRANQUE 1200
#define TURBINA_US_RECTA    1800
#define TURBINA_US_CURVA    1800
#define TURBINA_ERROR_CURVA 3000.0f
#define TURBINA_SUBE_US     0.5f
#define TURBINA_BAJA_US     0.1f
#define T_TURBINA_ARRANQUE_MS 1000

#define TAREA_CONTROL_PILA      4096
#define TAREA_CONTROL_PRIORIDAD 5

#define IR_BOTON_MOTORES 0x45
#define IR_BOTON_TURBINA 0x46
#define IR_BOTON_CALIBRAR 0x47
#define T_ANTIRREBOTE_MS 500

extern float posicion_actual;
extern uint32_t tiempo_sin_linea;
extern bool turbina_encendida;
extern uint32_t t_turbina;

uint32_t millis(void);
int limitar(int valor, int minimo, int maximo);
float limitar_f(float valor, float minimo, float maximo);

void motores_init(void);
void moverRobot(int vel_izq, int vel_der);

void turbina_init(void);
void turbina(uint16_t us);
void actualizar_turbina(float error);

void sensores_init(void);
void calibrar_auto(void);
int leerSensores(void);
void probar_sensores(void);

void ir_init(void);
int ir_boton(void);

#endif
