#include <stdio.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "gpio.h"
#include "comm_usb.h"
#include "hw_conf.h"
#include "led_rgb.h"
#include "ltc6803.h"

int main(void) {

    halInit();
    chSysInit();
    gpio_init();
    led_rgb_init();
    ltc6803_init();
    comm_usb_init();

    while(1)
    {
        led_rgb_set(0x00FF00);
        chThdSleepMilliseconds(100);
        led_rgb_set(0x000000);
        chThdSleepMilliseconds(100);
        uint8_t config[1][6];
        config[0][0] = 0b01100001;
        config[0][1] = 0b00000000;
        config[0][2] = 0b00000000;
        config[0][3] = 0b11111100;
        config[0][4] = 0b00000000;
        config[0][5] = 0b00000000;
        ltc6803_wrcfg(config);
        uint16_t cells[1][12];
        ltc6803_stcvad();
        chThdSleepMilliseconds(13);
        ltc6803_rdcv(cells);
        for (int i = 0; i < 12; i++)
        {
            /*chprintf((BaseSequentialStream *)&SDU1, "Cell %d: %fmV\n", i, cells[0][i] * 1.5);*/
        }
    }

}
