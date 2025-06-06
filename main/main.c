/* Built from the Blink Example

   This code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"


#define RED_LED_PIN				(GPIO_NUM_25)
#define GREEN_LED_PIN			(GPIO_NUM_26)
#define BLUE_LED_PIN			(GPIO_NUM_27)


static const char *TAG = "blink";

/* choose the GPIO to blink.
*/
//#define BLINK_GPIO RED_LED_PIN
#define BLINK_GPIO GREEN_LED_PIN
//#define BLINK_GPIO BLUE_LED_PIN

#define BLINK_PERIOD    (1200)
#define RED_TIME        (8000)
#define GREEN_TIME      (8000)
#define AMBER_TIME      (2000)
#define RELAY_DELAY     (80)

#define SEQUENCE_OP_SECS  (60)
#define BLINK_OP_SECS   (30)

#define LEVEL_ACTIVE    (0)
#define LEVEL_INACTIVE  (1)


static uint8_t s_led_state = LEVEL_INACTIVE;

static char sa_sequence[] = {'R', 'G', 'A'};   // red, green, amber, red, green, amber, red...
static uint16_t sa_seq_time[] = {RED_TIME, GREEN_TIME, AMBER_TIME};
static uint8_t s_seq_index      = 0;
static uint16_t s_seq_seconds   = 0;


static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}


static void configure_led(void)
{
    ESP_LOGI(TAG, "configured to blink GPIO LED!");
    
    gpio_set_level(RED_LED_PIN, LEVEL_INACTIVE);
    gpio_set_level(GREEN_LED_PIN, LEVEL_INACTIVE);
    gpio_set_level(BLUE_LED_PIN, LEVEL_INACTIVE);

    /* Set the GPIO as a push/pull output */
    gpio_set_direction(RED_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREEN_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BLUE_LED_PIN, GPIO_MODE_OUTPUT);
}


static void show_sequence(void)
{
    // show the lights in the correct sequence
    switch (sa_sequence[s_seq_index])
    {
        case    'R':      // RED
        gpio_set_level(GREEN_LED_PIN, LEVEL_INACTIVE);
        vTaskDelay(RELAY_DELAY / portTICK_PERIOD_MS);
        gpio_set_level(BLUE_LED_PIN, LEVEL_INACTIVE);
        vTaskDelay(RELAY_DELAY / portTICK_PERIOD_MS);
        gpio_set_level(RED_LED_PIN, LEVEL_ACTIVE);
        break;
        
        case    'A':      // GREEN
        gpio_set_level(RED_LED_PIN, LEVEL_INACTIVE);
        vTaskDelay(RELAY_DELAY / portTICK_PERIOD_MS);
        gpio_set_level(BLUE_LED_PIN, LEVEL_INACTIVE);
        vTaskDelay(RELAY_DELAY / portTICK_PERIOD_MS);
        gpio_set_level(GREEN_LED_PIN, LEVEL_ACTIVE);
        break;
        
        case    'G':      // BLUE
        gpio_set_level(RED_LED_PIN, LEVEL_INACTIVE);
        vTaskDelay(RELAY_DELAY / portTICK_PERIOD_MS);
        gpio_set_level(GREEN_LED_PIN, LEVEL_INACTIVE);
        vTaskDelay(RELAY_DELAY / portTICK_PERIOD_MS);
        gpio_set_level(BLUE_LED_PIN, LEVEL_ACTIVE);
        break;

        default:
        s_seq_index     = 0;
        break;
    }

    if (s_seq_seconds == 0)
    {
        ++s_seq_index;
        if (s_seq_index > (sizeof(sa_sequence) - 1))
        {
            s_seq_index = 0;
        }

        // set the sequence time ready for the next light
        s_seq_seconds   = sa_seq_time[s_seq_index] / 1000;

        ESP_LOGI(TAG, "Changing to %c", sa_sequence[s_seq_index]);
    }
    else
    {
        --s_seq_seconds;
    }
}


void app_main(void)
{
    uint8_t s_show_sequence     = 0;
    unsigned int ms_ticks       = 0;
    unsigned int seconds_ctr    = 0;

    /* Configure the peripheral according to the LED type */
    configure_led();

    s_seq_seconds       = sa_seq_time[0] / 1000;

    while (1) {
        if (s_show_sequence != 0)
        {
            show_sequence();

            // switch to blinking
            if (seconds_ctr > SEQUENCE_OP_SECS)
            {
                s_show_sequence     = 0;
                seconds_ctr         = 0;

                s_led_state         = 0;
                gpio_set_level(RED_LED_PIN, LEVEL_INACTIVE);
                vTaskDelay(RELAY_DELAY / portTICK_PERIOD_MS);
                gpio_set_level(GREEN_LED_PIN, LEVEL_INACTIVE);
                vTaskDelay(RELAY_DELAY / portTICK_PERIOD_MS);
                gpio_set_level(BLUE_LED_PIN, LEVEL_INACTIVE);
                
                ESP_LOGI(TAG, "switching to blink");
            }
        }
        else
        {
            //ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");

            blink_led();
            /* Toggle the LED state */
            s_led_state = !s_led_state;

            // switch to showing sequence
            if (seconds_ctr > BLINK_OP_SECS)
            {
                s_show_sequence     = 1;
                seconds_ctr         = 0;
                
                s_seq_index         = 0;
                s_seq_seconds       = sa_seq_time[s_seq_index] / 1000;
                
                ESP_LOGI(TAG, "switching to sequence, %c", sa_sequence[s_seq_index]);
            }
        }

        vTaskDelay(BLINK_PERIOD / portTICK_PERIOD_MS);

        // alternate between blinking and sequence
        ms_ticks += (BLINK_PERIOD);
        while (ms_ticks > 1000)
        {
            ms_ticks -= 1000;
            ++seconds_ctr;
        }
    }
}
