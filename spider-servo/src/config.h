#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <stdlib.h>
#include "driver/gpio.h"

static const uint8_t LEG_ID = 1; //1-6
static const uint8_t MOTOR_ID = 2; //1-3, 1 is near body

static const uint16_t MOTOR_MIN_ANGLE = 20;
static const uint16_t MOTOR_MAX_ANGLE = 150;

static constexpr float P = 7;
static constexpr float I = 28.36;
static constexpr float D = 0.54;
static constexpr float CURRENT_LIMIT = 5;

#ifdef SPIDER_SERVO_C3
static const gpio_num_t TWAI_TX_PIN = GPIO_NUM_21;
static const gpio_num_t TWAI_RX_PIN = GPIO_NUM_20;

static constexpr adc1_channel_t PIN_CURRENT_SENSOR = ADC1_CHANNEL_0;
static constexpr gpio_num_t PIN_MOTOR_PWM = GPIO_NUM_1;
static constexpr gpio_num_t PIN_MOTOR_DIR = GPIO_NUM_2;
static constexpr gpio_num_t PIN_MOTOR_BRAKE = GPIO_NUM_3;

static constexpr gpio_num_t PIN_MT6701_MISO = GPIO_NUM_8;
static constexpr gpio_num_t PIN_MT6701_SCLK = GPIO_NUM_9;
static constexpr gpio_num_t PIN_MT6701_CS = GPIO_NUM_10;

#endif 

#ifdef SPIDER_SERVO_S3
static const gpio_num_t TWAI_TX_PIN = GPIO_NUM_7;
static const gpio_num_t TWAI_RX_PIN = GPIO_NUM_8;

static constexpr adc1_channel_t PIN_CURRENT_SENSOR = ADC1_CHANNEL_5;
static constexpr gpio_num_t PIN_MOTOR_PWM = GPIO_NUM_1;
static constexpr gpio_num_t PIN_MOTOR_DIR = GPIO_NUM_2;
static constexpr gpio_num_t PIN_MOTOR_BRAKE = GPIO_NUM_3;

static constexpr gpio_num_t PIN_MT6701_MISO = GPIO_NUM_13;
static constexpr gpio_num_t PIN_MT6701_SCLK = GPIO_NUM_12;
static constexpr gpio_num_t PIN_MT6701_CS = GPIO_NUM_11; 
#endif 

#endif