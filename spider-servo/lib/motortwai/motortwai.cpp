#include "motortwai.h"
#include <memory.h>


void MotorTWAI::setup(uint8_t legn, uint8_t motorn, gpio_num_t tx_pin, gpio_num_t rx_pin, bool selfTest) {
    _legn = legn;
    _motorn = motorn;

    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = tx_pin,
            .rx = rx_pin,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = {
            .bitrate = 500000,
        },
        .tx_queue_depth = 5,
        .intr_priority = 0,
    };

    if (selfTest) {
        node_config.flags = {
            .enable_self_test = 1, 
            .enable_loopback = 1,
        };
    }

    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &node_hdl));

    //.callback = [](void* arg){ static_cast<MotorDriver*>(arg)->compute(); },
    twai_event_callbacks_t user_cbs = {
        .on_rx_done = staticRxWrapper,
        .on_error = staticErrorWrapper
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node_hdl, &user_cbs, this));

    ESP_ERROR_CHECK(twai_node_enable(node_hdl));    
    //-----
//    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
//    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // 500kbps (common CAN speed)

    // twai_filter_config_t f_config = createMotorFilter(_legn, _motorn);
    // //twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    // ESP_LOGI(TAG, "CAN Driver installed");
    // // if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    // //     ESP_LOGI(TAG, "CAN Driver installed");
    // // } else {
    // //     ESP_LOGE(TAG, "Failed to install CAN driver");
    // //     return;
    // // }

    // ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "CAN started");


    // if (twai_start() == ESP_OK) {
    //     ESP_LOGI(TAG, "CAN started");
    // } else {
    //     ESP_LOGE(TAG, "Failed to start CAN");
    //     return;
    // }

    // uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED |
    //                             TWAI_ALERT_BUS_ERROR | TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS |
    //                             TWAI_ALERT_ARB_LOST | TWAI_ALERT_RX_QUEUE_FULL;
    // if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    //     ESP_LOGI(TAG, "Alerts configured");
    // } else {
    //     ESP_LOGE(TAG, "Failed to configure alerts");
    // }    
}

bool MotorTWAI::staticErrorWrapper(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *user_ctx) {
    ESP_EARLY_LOGE("T", "static Error wrapper");
    MotorTWAI* instance = static_cast<MotorTWAI*>(user_ctx);    
    return instance->_errorCallback(handle, edata);
}

bool MotorTWAI::staticRxWrapper(twai_node_handle_t handle, 
                            const twai_rx_done_event_data_t *edata, 
                            void *user_ctx) {
    MotorTWAI* instance = static_cast<MotorTWAI*>(user_ctx);
    return instance->_rxCallback(handle, edata);
}

void MotorTWAI::setOnMotorCommandCallback(std::function<void(motor_command_t)> callback) {
    _onMotorCommandCallback = callback;
}

bool MotorTWAI::_rxCallback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata) {
    ESP_LOGI(TAG, "rxCallback invoked");
    motor_command_t motorCmd;
    twai_frame_t rx_frame = {
        .buffer = reinterpret_cast<uint8_t*>(&motorCmd),
        .buffer_len = sizeof(motor_command_t),
    };
    if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
        if (_onMotorCommandCallback) {
            _onMotorCommandCallback(motorCmd);
        }
        ESP_LOGI(TAG, "Received CAN ID: 0x%lx, payload %f", rx_frame.header.id, motorCmd.param);
        ESP_LOG_BUFFER_HEX(TAG, rx_frame.buffer, rx_frame.buffer_len);
    }
    return false;
}

bool MotorTWAI::_errorCallback(twai_node_handle_t handle, const twai_error_event_data_t *edata)
{
    ESP_EARLY_LOGE(TAG, "error flags arblost %ld, bit_err %ld, form_err %ld, stuff_err  %ld, ack_err  %ld", edata->err_flags.arb_lost, edata->err_flags.bit_err, edata->err_flags.form_err, edata->err_flags.stuff_err, edata->err_flags.ack_err);
    return false;
}

// bool MOTOR_TWAI::receive(motor_command_t *cmd) {
//     twai_message_t rx_msg;
//     esp_err_t status;
//     status = twai_receive(&rx_msg, pdMS_TO_TICKS(100));
//     if (status == ESP_OK) {
//         memcpy(cmd, &rx_msg.data, sizeof(motor_command_t)); 
//         ESP_LOGI(TAG, "Received CAN ID: 0x%lx, payload %f", rx_msg.identifier, cmd->param);
//         ESP_LOG_BUFFER_HEX(TAG, rx_msg.data, rx_msg.data_length_code);
//         return true;
//     } else if (status != ESP_ERR_TIMEOUT) {
//         ESP_LOGE(TAG, "Failed to receive message %s\n", esp_err_to_name(status));
//     }
//     return false;
// }

void MotorTWAI::sendStatus(motor_status_t status) {
    twai_frame_t tx_msg = {
        .header = {
            .id = createMsgId(MOTOR_FEEDBACK, _legn, _motorn)
        },
        .buffer = reinterpret_cast<uint8_t*>(&status),
        .buffer_len = sizeof(status)
    };
    ESP_ERROR_CHECK(twai_node_transmit(node_hdl, &tx_msg, 0));  // Timeout = 0: returns immediately if queue is full    
    ESP_LOGI(TAG, "Msg sent id: %ld", tx_msg.header.id);
    ESP_LOG_BUFFER_HEX(TAG, tx_msg.buffer, tx_msg.buffer_len);
}

// Function just for UNIT test, send command for this receiver
// FIXME: use function from central controller
void MotorTWAI::sendMotorCommand(motor_command_t cmd) {
    twai_frame_t tx_msg = {
        .header = {
            .id = createMsgId(MOTOR_COMMAND, _legn, _motorn),
        },
        .buffer = reinterpret_cast<uint8_t*>(&cmd),
        .buffer_len = sizeof(cmd)
    };
    ESP_ERROR_CHECK(twai_node_transmit(node_hdl, &tx_msg, 0));  // Timeout = 0: returns immediately if queue is full    
    ESP_LOGI(TAG, "Msg sent id: %ld", tx_msg.header.id);
    ESP_LOG_BUFFER_HEX(TAG, tx_msg.buffer, tx_msg.buffer_len);
}

void MotorTWAI::logStatus() {
    twai_node_status_t status;
    twai_node_record_t stats;
    twai_node_get_info(node_hdl, &status, &stats);
    ESP_LOGI(TAG, "TWAI status %d, bus err cnt=%ld, TX err cnt=%ld, RX err cnt=%ld", status.state, stats.bus_err_num, status.tx_error_count, status.rx_error_count);
    // uint32_t alerts_triggered;
    // twai_status_info_t status_info;

    
    // // Wait indefinitely for alerts
    // if (twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(100)) == ESP_OK) {
    //     // Log triggered alerts
    //     if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    //         ESP_LOGI(TAG, "Alert: Received message");
    //     }
    //     if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
    //         ESP_LOGI(TAG, "Alert: Transmission successful");
    //     }
    //     if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
    //         ESP_LOGE(TAG, "Alert: Transmission failed");
    //     }
    //     if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    //         ESP_LOGE(TAG, "Alert: Bus error detected");
    //     }
    //     if (alerts_triggered & TWAI_ALERT_BUS_OFF) {
    //         ESP_LOGE(TAG, "Alert: Bus-off state entered");
    //         // Initiate bus recovery if needed
    //         //twai_initiate_recovery();
    //     }
    //     if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    //         ESP_LOGW(TAG, "Alert: Error passive state");
    //     }
    //     if (alerts_triggered & TWAI_ALERT_ARB_LOST) {
    //         ESP_LOGW(TAG, "Alert: Arbitration lost");
    //     }
    //     if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
    //         ESP_LOGE(TAG, "Alert: RX queue full - data loss");
    //     }
    // }

    // // Optional: Read and log detailed status info
    // if (twai_get_status_info(&status_info) == ESP_OK) {
    //     ESP_LOGI(TAG, "Status %d: TX err cnt=%ld, RX err cnt=%ld", 
    //                 status_info.state, status_info.tx_error_counter, status_info.rx_error_counter);
    //     ESP_LOGI(TAG, "Status %d: TX failed=%ld, RX missed=%ld", 
    //                 status_info.state, status_info.tx_failed_count, status_info.rx_missed_count);
    // }    
}