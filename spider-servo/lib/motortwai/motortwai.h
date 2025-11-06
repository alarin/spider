#ifndef _MOTORTWAI_H_
#define _MOTORTWAI_H_

#include <functional>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include "twai_proto.h"


class MotorTWAI {
    public:
        void setup(uint8_t legn, uint8_t motorn, gpio_num_t tx_pin, gpio_num_t rx_pin, bool selfTest = false);        
        void sendStatus(motor_status_t status);
        void sendMotorCommand(motor_command_t cmd);
        void logStatus();
        bool rxFromQueue(motor_command_t *cmd, TickType_t xTicksToWait = portMAX_DELAY);

        static bool staticErrorWrapper(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *user_ctx);
        static bool staticRxWrapper(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx);

    private:
        static constexpr const char* TAG = "MTRTWAI";
        static constexpr const uint8_t RX_Q_SIZE = 10;

        std::function<void()> _onTXCallback; 
    
        bool _rxCallback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata);
        bool _errorCallback(twai_node_handle_t handle, const twai_error_event_data_t *edata);

        QueueHandle_t _rxQueue;
  
        twai_node_handle_t node_hdl;
        uint8_t _legn;
        uint8_t _motorn;
};

#endif