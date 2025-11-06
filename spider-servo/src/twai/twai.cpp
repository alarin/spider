#include "twai.h"
#include <memory.h>


void MOTOR_TWAI::setup(uint8_t legn, uint8_t motorn, gpio_num_t tx_pin, gpio_num_t rx_pin) {
    _legn = legn;
    _motorn = motorn;
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // 500kbps (common CAN speed)

    twai_filter_config_t f_config = createMotorFilter(_legn, _motorn);
    //twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TAG, "CAN Driver installed");
    // if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    //     ESP_LOGI(TAG, "CAN Driver installed");
    // } else {
    //     ESP_LOGE(TAG, "Failed to install CAN driver");
    //     return;
    // }

    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "CAN started");
    // if (twai_start() == ESP_OK) {
    //     ESP_LOGI(TAG, "CAN started");
    // } else {
    //     ESP_LOGE(TAG, "Failed to start CAN");
    //     return;
    // }

    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED |
                                TWAI_ALERT_BUS_ERROR | TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS |
                                TWAI_ALERT_ARB_LOST | TWAI_ALERT_RX_QUEUE_FULL;
    if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
        ESP_LOGI(TAG, "Alerts configured");
    } else {
        ESP_LOGE(TAG, "Failed to configure alerts");
    }    
}

bool MOTOR_TWAI::receive(motor_command_t *cmd) {
    twai_message_t rx_msg;
    esp_err_t status;
    status = twai_receive(&rx_msg, pdMS_TO_TICKS(100));
    if (status == ESP_OK) {
        memcpy(cmd, &rx_msg.data, sizeof(motor_command_t)); 
        ESP_LOGI(TAG, "Received CAN ID: 0x%lx, payload %f", rx_msg.identifier, cmd->param);
        ESP_LOG_BUFFER_HEX(TAG, rx_msg.data, rx_msg.data_length_code);
        return true;
    } else if (status != ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "Failed to receive message %s\n", esp_err_to_name(status));
    }
    return false;
}

void MOTOR_TWAI::sendStatus(motor_status_t status) {
    twai_message_t tx_msg;
    tx_msg.data_length_code = sizeof(status);
    tx_msg.identifier = createMsgId(MOTOR_FEEDBACK, _legn, _motorn);
    memcpy(&tx_msg.data, &status, sizeof(status));
    ESP_ERROR_CHECK(twai_transmit(&tx_msg, portMAX_DELAY));
    ESP_LOGI(TAG, "Msg sent id: %ld", tx_msg.identifier);
    ESP_LOG_BUFFER_HEX(TAG, tx_msg.data, tx_msg.data_length_code);
}

void MOTOR_TWAI::logStatus() {
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
            //twai_initiate_recovery();
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