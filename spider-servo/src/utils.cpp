#include "utils.h"

uint64_t micros() {
    return (uint64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS * 1000);
}

uint64_t millis() {
    return (uint64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}
