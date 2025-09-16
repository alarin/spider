/* TWAI Self Test Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * The following example demonstrates the self testing capabilities of the TWAI
 * peripheral by utilizing the No Acknowledgment Mode and Self Reception Request
 * capabilities. This example can be used to verify that the TWAI peripheral and
 * its connections to the external transceiver operates without issue. The example
 * will execute multiple iterations, each iteration will do the following:
 * 1) Start the TWAI driver
 * 2) Transmit and receive 100 messages using self reception request
 * 3) Stop the TWAI driver
 */

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "driver/gpio.h"

/* --------------------- Definitions and static variables ------------------ */

#define DEVICE_ID               2
#define SELF_TEST               1

#if DEVICE_ID == 1
#define TX_GPIO_NUM             GPIO_NUM_20 //GPIO_NUM_0
#define RX_GPIO_NUM             GPIO_NUM_21 //GPIO_NUM_1
#else
#define TX_GPIO_NUM             GPIO_NUM_21
#define RX_GPIO_NUM             GPIO_NUM_20
#endif


#define LED_GPIO                GPIO_NUM_8
#define NO_OF_MSGS              100
#define NO_OF_ITERS             100
#define TX_TASK_PRIO            8      //Sending task priority
#define RX_TASK_PRIO            9       //Receiving task priority
#define CTRL_TSK_PRIO           10      //Control task priority
#define MSG_ID                  0x555   //11 bit standard format ID
#define EXAMPLE_TAG             "TWAI"
#define TAG             "TWAI"

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_25KBITS();
//Filter all other IDs except MSG_ID
static const twai_filter_config_t f_config = {.acceptance_code = uint32_t(MSG_ID << 21),
                                             .acceptance_mask = ~(TWAI_STD_ID_MASK << 21),
                                             .single_filter = true};
// static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
//Set to NO_ACK mode due to self testing with single module
#ifdef SELF_TEST
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NO_ACK);
#else
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
#endif

static SemaphoreHandle_t tx_sem;
static SemaphoreHandle_t rx_sem;
static SemaphoreHandle_t ctrl_sem;
static SemaphoreHandle_t done_sem;

extern "C" {
    void app_main(void);
}

/* --------------------------- Tasks and Functions -------------------------- */

static void logStatus() {
    uint32_t alerts_triggered;
    twai_status_info_t status_info;

    
    // Wait indefinitely for alerts
    if (twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(100)) == ESP_OK) {
        // Log triggered alerts
        if (alerts_triggered & TWAI_ALERT_RX_DATA) {
            ESP_LOGI(TAG, "Alert: Received message");
        }
        if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
            ESP_LOGI(TAG, "Alert: Transmission successful");
        }
        if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
            ESP_LOGE(TAG, "Alert: Transmission failed");
        }
        if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
            ESP_LOGE(TAG, "Alert: Bus error detected");
        }
        if (alerts_triggered & TWAI_ALERT_BUS_OFF) {
            ESP_LOGE(TAG, "Alert: Bus-off state entered");
            // Initiate bus recovery if needed
            twai_initiate_recovery();
        }
        if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
            ESP_LOGW(TAG, "Alert: Error passive state");
        }
        if (alerts_triggered & TWAI_ALERT_ARB_LOST) {
            ESP_LOGW(TAG, "Alert: Arbitration lost");
        }
        if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
            ESP_LOGE(TAG, "Alert: RX queue full - data loss");
        }
    }

    // Optional: Read and log detailed status info
    if (twai_get_status_info(&status_info) == ESP_OK) {
        ESP_LOGI(TAG, "Status %d: TX err cnt=%ld, RX err cnt=%ld", 
                    status_info.state, status_info.tx_error_counter, status_info.rx_error_counter);
        ESP_LOGI(TAG, "Status %d: TX failed=%ld, RX missed=%ld", 
                    status_info.state, status_info.tx_failed_count, status_info.rx_missed_count);
    }    
}

static void blink() {
    // Turn LED ON
    gpio_set_level(LED_GPIO, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);  // Delay 1 second

    // Turn LED OFF
    gpio_set_level(LED_GPIO, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);  // Delay 1 second    
}

static void twai_transmit_task(void *arg)
{
    twai_message_t tx_msg;
    tx_msg.data_length_code = 2;
    tx_msg.identifier = MSG_ID;
    #ifdef SELF_TEST
    tx_msg.flags = TWAI_MSG_FLAG_SELF;
    #endif
    for (int iter = 0; iter < NO_OF_ITERS; iter++) {
        xSemaphoreTake(tx_sem, portMAX_DELAY);
        for (int i = 0; i < NO_OF_MSGS; i++) {
            //Transmit messages using self reception request
            tx_msg.data[0] = DEVICE_ID;
            tx_msg.data[1] = i;
            ESP_ERROR_CHECK(twai_transmit(&tx_msg, 1000));
            ESP_LOGE(EXAMPLE_TAG, "SENT - Data = %d: %d", tx_msg.data[0], tx_msg.data[1]);
            logStatus();
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    vTaskDelete(NULL);
}

static void twai_receive_task(void *arg)
{
    twai_message_t rx_message;

    for (int iter = 0; iter < NO_OF_ITERS; iter++) {
        xSemaphoreTake(rx_sem, portMAX_DELAY);
        for (int i = 0; i < NO_OF_MSGS; i++) {
            //Receive message and print message data
            esp_err_t status = twai_receive(&rx_message, 100);
            if (status == ESP_OK) {
                ESP_LOGE(EXAMPLE_TAG, "RECEIVED - Data = %d: %d", rx_message.data[0], rx_message.data[1]);
                blink();
            } else if (status != ESP_ERR_TIMEOUT) {
                ESP_LOGE(TAG, "recieve error %s", esp_err_to_name(status));
            }
        }
        //Indicate to control task all messages received for this iteration
        xSemaphoreGive(ctrl_sem);
    }
    vTaskDelete(NULL);
}


static void twai_control_task(void *arg)
{
    xSemaphoreTake(ctrl_sem, portMAX_DELAY);
    for (int iter = 0; iter < NO_OF_ITERS; iter++) {
        //Start TWAI Driver for this iteration
        ESP_ERROR_CHECK(twai_start());
        ESP_LOGI(EXAMPLE_TAG, "Driver started");

        //Trigger TX and RX tasks to start transmitting/receiving
        xSemaphoreGive(rx_sem);
        xSemaphoreGive(tx_sem);
        xSemaphoreTake(ctrl_sem, portMAX_DELAY);    //Wait for TX and RX tasks to finish iteration        
        ESP_ERROR_CHECK(twai_stop());               //Stop the TWAI Driver
        ESP_LOGI(EXAMPLE_TAG, "Driver stopped");
        vTaskDelay(pdMS_TO_TICKS(1000));             //Delay then start next iteration
    }
    xSemaphoreGive(done_sem);
    vTaskDelete(NULL);
}


void app_main(void)
{    
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 1);

    //Create tasks and synchronization primitives
    tx_sem = xSemaphoreCreateBinary();
    rx_sem = xSemaphoreCreateBinary();
    ctrl_sem = xSemaphoreCreateBinary();
    done_sem = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(twai_control_task, "TWAI_ctrl", 4096, NULL, CTRL_TSK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);

    //Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");

    //Start control task
    xSemaphoreGive(ctrl_sem);
    //Wait for all iterations and tasks to complete running
    xSemaphoreTake(done_sem, portMAX_DELAY);

    //Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

    //Cleanup
    vSemaphoreDelete(tx_sem);
    vSemaphoreDelete(rx_sem);
    vSemaphoreDelete(ctrl_sem);
    vQueueDelete(done_sem);
}