#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "funciones.h"

bool robot_encendido = false;
float error = 0;
float error_anterior = 0;
float derivada = 0;
float vel = VEL_MINIMA;

static void tarea_control(void *parametro)
{
    TickType_t ultimo_ciclo = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&ultimo_ciclo, pdMS_TO_TICKS(TS_MS));

        int boton = ir_boton();
        if (boton == IR_BOTON_MOTORES) {
            robot_encendido = !robot_encendido;
            error = 0;
            error_anterior = 0;
            derivada = 0;
            vel = VEL_MINIMA;
            tiempo_sin_linea = millis();
        }
        if (boton == IR_BOTON_TURBINA) {
            turbina_encendida = !turbina_encendida;
            t_turbina = millis();
        }
        if (boton == IR_BOTON_CALIBRAR) {
            robot_encendido = false;
            calibrar_auto();
            ir_boton();
            ultimo_ciclo = xTaskGetTickCount();
        }

        actualizar_turbina(error);

        if (!robot_encendido) {
            moverRobot(0, 0);
            continue;
        }

        if (!leerSensores()) continue;

        error = -posicion_actual;

        derivada += ALFA_D * ((error - error_anterior) - derivada);
        float correccion = error * KP + derivada * KD;

        float limite = LIMITE_BASE + fabsf(error) * LIMITE_PEND;
        if (limite > LIMITE_MAX) limite = LIMITE_MAX;
        correccion = limitar_f(correccion, -limite, limite);

        error_anterior = error;

        float vel_objetivo = limitar_f(VEL_MAXIMA - fabsf(error) * K_FRENO, VEL_MINIMA, VEL_MAXIMA);
        vel += limitar_f(vel_objetivo - vel, -FRENADO, ACELERACION);

        int vel_izq = limitar(vel - correccion, -20, VEL_MAXIMA);
        int vel_der = limitar(vel + correccion, -20, VEL_MAXIMA);

        moverRobot(vel_izq, vel_der);
    }
}

void app_main(void)
{
    motores_init();
    turbina_init();
    moverRobot(0, 0);
    sensores_init();
    ir_init();

    xTaskCreate(tarea_control, "control", TAREA_CONTROL_PILA, NULL, TAREA_CONTROL_PRIORIDAD, NULL);
}
