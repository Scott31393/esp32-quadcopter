/* LEDC (LED Controller) fade example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

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

#define PORT 3333

static const char *TAG = "example";


struct cmd_t{
    char buffer[128];
    unsigned int duty;
};

QueueHandle_t cmd_queue;


static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;
    struct cmd_t cmd;
    struct cmd_t *cmd_ptr;
    char *compared = "hello\n";

    cmd_ptr = &cmd;
    cmd_queue = xQueueCreate(1, sizeof(unsigned int));

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6) {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }
#endif

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        while (1) {

            ESP_LOGI(TAG, "Waiting for data");
            struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.sin6_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                } else if (source_addr.sin6_family == PF_INET6) {
                    inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);

                
                int comp_len = strlen(compared);

                for(int i =0; i<len; i++)
                    ESP_LOGI(TAG, "%c", rx_buffer[i]);
                    


                for(int i =0; i<comp_len; i++)
                    ESP_LOGI(TAG, "%c", compared[i]);


                if(strcmp(rx_buffer, compared) == 0){
                    cmd_ptr->duty = 4096;
                    xQueueSend(cmd_queue, &(cmd_ptr->duty), 0);
                    ESP_LOGI(TAG, "NICE!!");
                }else{
                    ESP_LOGI(TAG, "WRONG!!");
                }

                int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}


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
    struct cmd_t current_cmd;
    struct cmd_t *current_cmd_ptr;

    

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

#ifdef CONFIG_EXAMPLE_IPV4
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET6, 5, NULL);
#endif



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

    current_cmd_ptr = &current_cmd;
    current_cmd_ptr->duty = 0;

    while (1) {
        /*
         * The range of the duty cycle values passed to functions depends on selected duty_resolution 
         * and should be from 0 to (2 ** duty_resolution) - 1. 
         * Duty cycle depends on resolution, for example  if the selected duty resolution (LEDC_TIMER_13_BIT) is 10, then the 
         * duty cycle values can range from 0 to 1023.
         */

        

        if(xQueueReceive(cmd_queue, &(current_cmd_ptr->duty), 0) == pdPASS){
            ESP_LOGI(TAG, "Duty changed = %d", current_cmd_ptr->duty);
        }
        for(ch=0; ch < LEDC_TEST_CH_NUM; ch++){
            ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, current_cmd_ptr->duty);
            ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
        }

        
        

        
        // ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, 4096);
        // ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
