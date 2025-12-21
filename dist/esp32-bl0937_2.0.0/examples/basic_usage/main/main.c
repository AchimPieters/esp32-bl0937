#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bl0937.h"

void app_main(void)
{
    bl0937_init_default();

    while (1) {
        printf("V=%.1fV I=%.3fA P=%.1fW E=%.3fWh PF=%.2f Tripped=%d\n",
               bl0937_voltage(),
               bl0937_current(),
               bl0937_power(),
               bl0937_energy_wh(),
               bl0937_power_factor(),
               (int)bl0937_is_tripped());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
