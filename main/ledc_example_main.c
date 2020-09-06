/* LEDC (LED Controller) fade example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

/*
 * About this example
 *
 * 1. Start with initializing LEDC module:
 *    a. Set the timer of LEDC first, this determines the frequency
 *       and resolution of PWM.
 *    b. Then set the LEDC channel you want to use,
 *       and bind with one of the timers.
 *
 * 2. You need first to install a default fade function,
 *    then you can use fade APIs.
 *
 * 3. You can also set a target duty directly without fading.
 *
 * 4. This example uses GPIO18/19/4/5 as LEDC output,
 *    and it will change the duty repeatedly.
 *
 * 5. GPIO18/19 are from high speed channel group.
 *    GPIO4/5 are from low speed channel group.
 *
 */


/* Max duty cycle is based on resolution in particular:
 * The range of the duty cycle values passed to functions depends on 
 * selected duty_resolution and should be from 0 to (2 ** duty_resolution) - 1. 
 * For example, if the selected duty resolution is 10
 * then the duty cycle values can range from 0 to 1023. This provides the resolution of ~0.1%
 */  

#define NO_FADE 0

#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE

#define LEDC_HS_CH0_GPIO       (13)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO       (12)
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1
#define LEDC_HS_CH2_GPIO       (14)
#define LEDC_HS_CH2_CHANNEL    LEDC_CHANNEL_2
#define LEDC_HS_CH3_GPIO       (27)
#define LEDC_HS_CH3_CHANNEL    LEDC_CHANNEL_3
#define LEDC_HS_CH4_GPIO       (26)
#define LEDC_HS_CH4_CHANNEL    LEDC_CHANNEL_4
#define LEDC_HS_CH5_GPIO       (25)
#define LEDC_HS_CH5_CHANNEL    LEDC_CHANNEL_5



#define LEDC_TEST_CH_NUM       (6)
#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)



/*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
    {
        .channel    = LEDC_HS_CH0_CHANNEL,
        .duty       = 0,
        .gpio_num   = LEDC_HS_CH0_GPIO,
        .speed_mode = LEDC_HS_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_HS_TIMER
    },
    {
        .channel    = LEDC_HS_CH1_CHANNEL,
        .duty       = 0,
        .gpio_num   = LEDC_HS_CH1_GPIO,
        .speed_mode = LEDC_HS_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_HS_TIMER
    },
    {
        .channel    = LEDC_HS_CH2_CHANNEL,
        .duty       = 0,
        .gpio_num   = LEDC_HS_CH2_GPIO,
        .speed_mode = LEDC_HS_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_HS_TIMER
    },
    {
        .channel    = LEDC_HS_CH3_CHANNEL,
        .duty       = 0,
        .gpio_num   = LEDC_HS_CH3_GPIO,
        .speed_mode = LEDC_HS_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_HS_TIMER
    },
    {
        .channel    = LEDC_HS_CH4_CHANNEL,
        .duty       = 0,
        .gpio_num   = LEDC_HS_CH4_GPIO,
        .speed_mode = LEDC_HS_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_HS_TIMER
    },
    {
        .channel    = LEDC_HS_CH5_CHANNEL,
        .duty       = 0,
        .gpio_num   = LEDC_HS_CH5_GPIO,
        .speed_mode = LEDC_HS_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_HS_TIMER
    },
};


int init_led_c_timer(ledc_timer_config_t *ledc_timer){

    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer->duty_resolution = LEDC_TIMER_13_BIT; // resolution of PWM duty
    ledc_timer->freq_hz = 5000;                      // frequency of PWM signal
    ledc_timer->speed_mode = LEDC_HS_MODE;           // timer mode
    ledc_timer->timer_num = LEDC_HS_TIMER;            // timer index
    ledc_timer->clk_cfg = LEDC_AUTO_CLK;              // Auto select the source clock

    return 0;

}

void app_main(void)
{
    int ch;
    ledc_timer_config_t ledc_timer;



    /*Initialize ledc_timer*/
    init_led_c_timer(&ledc_timer);

    ledc_timer.timer_num = LEDC_HS_TIMER;
    ledc_timer_config(&ledc_timer);

    
    /* Set LED Controller with previously prepared configuration*/
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }

    /* Initialize fade service.*/
    ledc_fade_func_install(0);

    while (1) {
        /*
         * The range of the duty cycle values passed to functions depends on selected duty_resolution 
         * and should be from 0 to (2 ** duty_resolution) - 1. 
         * Duty cycle depends on resolution, for example  if the selected duty resolution (LEDC_TIMER_13_BIT) is 10, then the 
         * duty cycle values can range from 0 to 1023.
         */

        for(ch=0; ch < LEDC_TEST_CH_NUM; ch++){
            ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, 8000);
            ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
