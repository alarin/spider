#include "motortwai.h"
#include <memory.h>

void MotorTWAI::setup(uint8_t legn, uint8_t motorn, gpio_num_t tx_pin, gpio_num_t rx_pin, bool selfTest) {
    _legn = legn;
    _motorn = motorn;

    _rxQueue = xQueueCreate(RX_Q_SIZE, sizeof(motor_command_t));

    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = tx_pin,
            .rx = rx_pin,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = {
            .bitrate = 250000,
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

    twai_event_callbacks_t user_cbs = {
        .on_rx_done = staticRxWrapper,
        // .on_error = staticErrorWrapper
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node_hdl, &user_cbs, this));

    twai_mask_filter_config_t mfilter_cfg = createMaskMotorFilter(_legn, _motorn);
    ESP_ERROR_CHECK(twai_node_config_mask_filter(node_hdl, 0, &mfilter_cfg));   // Configure on filter 0    

    ESP_ERROR_CHECK(twai_node_enable(node_hdl));    

    ESP_LOGI(TAG, "CAN started");
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

bool MotorTWAI::_rxCallback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata) {
    motor_command_t motorCmd;    
    twai_frame_t rx_frame = {
        .buffer = reinterpret_cast<uint8_t*>(&motorCmd),
        .buffer_len = sizeof(motor_command_t),
    };
    BaseType_t xHigherPriorityTaskWoken;
    if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
        xQueueSendFromISR(_rxQueue, &motorCmd, &xHigherPriorityTaskWoken);
        //ESP_EARLY_LOGI(TAG, "Received CAN ID: 0x%lx, payload %f", rx_frame.header.id, motorCmd.param);
        //ESP_EARLY_LOG_BUFFER_HEX(TAG, rx_frame.buffer, rx_frame.buffer_len);
        return xHigherPriorityTaskWoken == pdTRUE;
    }
    return false;
}

bool MotorTWAI::_errorCallback(twai_node_handle_t handle, const twai_error_event_data_t *edata)
{
    ESP_EARLY_LOGE(TAG, "error flags arblost %ld, bit_err %ld, form_err %ld, stuff_err  %ld, ack_err  %ld", edata->err_flags.arb_lost, edata->err_flags.bit_err, edata->err_flags.form_err, edata->err_flags.stuff_err, edata->err_flags.ack_err);
    return false;
}

void MotorTWAI::sendStatus(motor_status_t status) {
    twai_frame_t tx_msg = {
        .header = {
            .id = createMsgId(MOTOR_FEEDBACK, _legn, _motorn),
            .fdf = 1,
        },
        .buffer = reinterpret_cast<uint8_t*>(&status),
        .buffer_len = sizeof(status)
    };
    ESP_ERROR_CHECK(twai_node_transmit(node_hdl, &tx_msg, 0));
    ESP_LOGD(TAG, "Msg sent id: %ld", tx_msg.header.id);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, tx_msg.buffer, tx_msg.buffer_len, ESP_LOG_DEBUG);
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
}

bool MotorTWAI::rxFromQueue(motor_command_t *cmd, TickType_t xTicksToWait) {
    return xQueueReceive(_rxQueue, cmd, xTicksToWait) == pdTRUE;
}
