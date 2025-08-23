#include "twai.h"
#include <memory.h>


void MOTOR_TWAI::setup(uint8_t legn, uint8_t motorn, gpio_num_t tx_pin, gpio_num_t rx_pin) {
    _legn = legn;
    _motorn = motorn;
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_25KBITS(); // 500kbps (common CAN speed)
    static const twai_filter_config_t f_config = {.acceptance_code = uint32_t(0x49 << 21),
                                             .acceptance_mask = ~(TWAI_STD_ID_MASK << 21),
                                             .single_filter = true};

    //twai_filter_config_t f_config = createMotorFilter(_legn, _motorn);
    //twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        ESP_LOGI(TAG, "CAN Driver installed");
    } else {
        ESP_LOGE(TAG, "Failed to install CAN driver");
        return;
    }

    if (twai_start() == ESP_OK) {
        ESP_LOGI(TAG, "CAN started");
    } else {
        ESP_LOGE(TAG, "Failed to start CAN");
        return;
    }
}

bool MOTOR_TWAI::receive(motor_commant_t *cmd) {
    twai_message_t rx_msg;
    if (twai_receive(&rx_msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
        ESP_LOGI(TAG, "Received CAN ID: 0x%lx", rx_msg.identifier);
        ESP_LOG_BUFFER_HEX(TAG, rx_msg.data, rx_msg.data_length_code);
        cmd->command = rx_msg.data[0];
        memcpy(&cmd->param, &rx_msg.data[1], sizeof(cmd->param)); 
        return true;
    }
    return false;
}