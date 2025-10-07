#ifndef MY_DEFINES_H
#define MY_DEFINES_H

#define DEVICE_ID               3
#define SELF_TEST            

#if DEVICE_ID == 1
#define LED_GPIO                GPIO_NUM_8
#define TX_GPIO_NUM             GPIO_NUM_20 //GPIO_NUM_0
#define RX_GPIO_NUM             GPIO_NUM_21 //GPIO_NUM_1
#elif DEVICE_ID == 2
#define LED_GPIO                GPIO_NUM_8
#define TX_GPIO_NUM             GPIO_NUM_21
#define RX_GPIO_NUM             GPIO_NUM_20
#elif DEVICE_ID == 3
#define LED_GPIO                GPIO_NUM_21
#define TWAI_TEST_FAST_LED
#define TX_GPIO_NUM             GPIO_NUM_7
#define RX_GPIO_NUM             GPIO_NUM_8
#endif

#endif