#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "funciones.h"

bool robot_encendido = false;

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
        if (boton == IR_BOTON_MOTORES && calibrado) robot_encendido = !robot_encendido;
        if (boton == IR_BOTON_TURBINA) turbina_encender(!turbina_encendida);
        if (boton == IR_BOTON_CALIBRAR) {
            robot_encendido = false;
            calibrar_auto();
            ir_boton();
            ultimo_ciclo = xTaskGetTickCount();
        }

        actualizar_turbina();

        if (!robot_encendido) {
            detenerMotores();
            continue;
        }

        lecturaLinea();

        if (linea_perdida && millis() - t_go >= T_BLOQUEO_RECUPERACION_MS) {
            recuperarLinea();
            continue;
        }

        controlPID();
    }
}
