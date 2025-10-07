#include "led.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#ifdef TWAI_TEST_FAST_LED
#endif

void ledInit() {
    #ifdef TWAI_TEST_FAST_LED
    #else
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 1);
    #endif
}

void turnOn() {
    #ifdef TWAI_TEST_FAST_LED
    #else
    gpio_set_level(LED_GPIO, 0);
    #endif
}
void turnOff() {
    #ifdef TWAI_TEST_FAST_LED
    #else
    gpio_set_level(LED_GPIO, 1);
    #endif
}

void blink() {
    turnOn();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    turnOff();
    vTaskDelay(500 / portTICK_PERIOD_MS);
}

