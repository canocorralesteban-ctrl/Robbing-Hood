#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "funciones.h"

bool robot_encendido = false;
float error = 0;
float error_anterior = 0;

void app_main(void)
{
    turbina_init();
    motores_init();
    moverRobot(0, 0);
    sensores_init();
    ir_init();

    calibrar();
    vTaskDelay(pdMS_TO_TICKS(2000));
    ir_start();

    TickType_t ultimo_ciclo = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&ultimo_ciclo, pdMS_TO_TICKS(TS_MS));

        if (ir_start()) {
            robot_encendido = !robot_encendido;

            if (robot_encendido) {
                turbina(TURBINA_US_CARRERA);
                vTaskDelay(pdMS_TO_TICKS(T_TURBINA_SUBE_MS));
                ultimo_ciclo = xTaskGetTickCount();
                error_anterior = 0;
                tiempo_sin_linea = millis();
            }
            else
                turbina(TURBINA_APAGADA_US);
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
