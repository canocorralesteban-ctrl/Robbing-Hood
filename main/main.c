#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "funciones.h"

bool robot_encendido = false;
bool turbina_encendida = false;
uint32_t t_turbina = 0;
float turbina_us = TURBINA_APAGADA_US;
float error = 0;
float error_anterior = 0;

void app_main(void)
{
    motores_init();
    turbina_init();
    moverRobot(0, 0);
    sensores_init();
    ir_init();

    TickType_t ultimo_ciclo = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&ultimo_ciclo, pdMS_TO_TICKS(TS_MS));

        int boton = ir_boton();
        if (boton == IR_BOTON_MOTORES) {
            robot_encendido = !robot_encendido;
            error = 0;
            error_anterior = 0;
            tiempo_sin_linea = millis();
        }
        if (boton == IR_BOTON_TURBINA) {
            turbina_encendida = !turbina_encendida;
            turbina_us = TURBINA_US_ARRANQUE;
            t_turbina = millis();
        }
        if (boton == IR_BOTON_CALIBRAR) {
            robot_encendido = false;
            calibrar_auto();
            ir_boton();
            ultimo_ciclo = xTaskGetTickCount();
        }

        if (!turbina_encendida) {
            turbina(TURBINA_APAGADA_US);
        } else if (millis() - t_turbina < T_TURBINA_ARRANQUE_MS) {
            turbina(TURBINA_US_ARRANQUE);
        } else {
            float curva = limitar_f(fabsf(error) / TURBINA_ERROR_CURVA, 0, 1);
            float objetivo = TURBINA_US_RECTA + (TURBINA_US_CURVA - TURBINA_US_RECTA) * curva;
            turbina_us += limitar_f(objetivo - turbina_us, -TURBINA_PASO_US, TURBINA_PASO_US);
            turbina(turbina_us);
        }

        if (!robot_encendido) {
            moverRobot(0, 0);
            continue;
        }

        if (!leerSensores()) continue;

        error = -posicion_actual;

        float P = error * KP;
        float D = (error - error_anterior) * KD;
        float correccion = P + D;

        float limite = LIMITE_BASE + fabsf(error) * LIMITE_PEND;
        if (limite > LIMITE_MAX) limite = LIMITE_MAX;
        correccion = limitar_f(correccion, -limite, limite);

        error_anterior = error;

        int vel = VEL_MAXIMA - fabsf(error) * K_FRENO;
        vel = limitar(vel, VEL_MINIMA, VEL_MAXIMA);

        int vel_izq = vel - correccion;
        int vel_der = vel + correccion;

        vel_izq = limitar(vel_izq, -20, VEL_MAXIMA);
        vel_der = limitar(vel_der, -20, VEL_MAXIMA);

        moverRobot(vel_izq, vel_der);
    }
}
