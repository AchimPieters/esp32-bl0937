#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bl0973.h"

void app_main(void)
{
    bl0973_init_default();

    while (1) {
        printf("V=%.1fV I=%.3fA P=%.1fW E=%.3fWh PF=%.2f Tripped=%d\n",
               bl0973_voltage(),
               bl0973_current(),
               bl0973_power(),
               bl0973_energy_wh(),
               bl0973_power_factor(),
               (int)bl0973_is_tripped());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
