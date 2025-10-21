#include "utils.h"

uint64_t Mmicros() {
    return (uint64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS * 1000);
}

uint64_t Mmillis() {
    return (uint64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}
