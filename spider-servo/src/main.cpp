#include <Arduino.h>
#include "mt6701/mt6701.h"

#define SPI_MISO   3
#define SPI_SCLK   2
#define SPI_CS     5

SPISensor sensor;

void setup() {
    Serial.begin(115200);
    Serial.println("Activating sensor...");
    sensor.begin(SPI_SCLK, SPI_MISO, SPI_CS);
}

void loop() {
    float angle;
    bool result;

    result = sensor.read(&angle, NULL, NULL, NULL);
    if (!result) {
        Serial.println("CRC ERROR");
    } else {
        Serial.print("angle: ");
        Serial.print(angle);
        Serial.println();
    }

    delay(1000);
}