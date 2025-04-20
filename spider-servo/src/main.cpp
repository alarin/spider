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

#define CRC_MASK             0b000000000000000000111111
#define MFC_MASK             0b000000000000001111000000
#define MFC_STRENGTH_MASK    0b000000000000000011000000
#define ANGLE_MASK           0b111111111111110000000000
//           

bool isCRCValid(uint32_t raw_data) {
    uint8_t crc = 0; // Initialize CRC to 0
    uint16_t original_crc = raw_data&CRC_MASK;
    uint32_t data = raw_data >> 6;
  
    // Process each bit from MSB (bit 17) to LSB (bit 0)
    for (int i = 17; i >= 0; i--) {
        // Extract the current bit
        uint8_t bit = (data >> i) & 1;
        
        // Calculate feedback: XOR of CRC's MSB and current data bit
        uint8_t feedback = ((crc >> 5) & 1) ^ bit;
        
        // Shift CRC left by 1 bit
        crc = (crc << 1) & 0x3F; // Mask to 6 bits
        
        // Apply polynomial if feedback is 1
        if (feedback) {
            crc ^= 0x03; // Polynomial X⁶ + X + 1 (0x03)
        }
    }
    
    return crc == original_crc;
}
  
void loop() {
    // //uint32_t dataWord = sensor.read24();
    
    // uint16_t dataWord = sensor.readWord();
    // uint16_t angle_data = (dataWord>>1)&0x3FFF;
    // float angle = ( angle_data / (float)16384.0f ) * TWO_PI;
    
    // uint32_t printVal = 0b000000000000000000000000;
    // printVal += dataWord;
    // Serial.print(">Raw data: ");
    // Serial.print(printVal, BIN);
    
    // Serial.print(", Angle: ");
    // Serial.println(angle);


    // uint16_t angle_raw;
    // SPISensor::mt6701_status_t status;

    // sensor.mt6701_read_raw(&angle_raw, &status, NULL, NULL);

    // angle = ( angle_raw / (float)16384.0f ) * TWO_PI;

    // Serial.print("angle: ");
    // Serial.print(angle);

    // Serial.print(" status: ");
    // Serial.println(status);
    //  1100 1011 1011 01
    //  1100 1011 1001 00 11
    //   1100 1011 1001 00 1
    //0b   1111 1111 1111 11
    uint32_t data = sensor.read24();

    if (! isCRCValid(data)) {
        Serial.println("WRONG CRC");
    }

    uint32_t printVal = 0b1000000000000000000000000;
    printVal += data;
    Serial.print(">Raw data: ");
    Serial.print(printVal, BIN);


    float angle = ( ((data>>10)&0x3FFF) / (float)16384.0f ) * TWO_PI;
    Serial.print(", angle: ");
    Serial.print(angle);

    // uint16_t crcVal = 0b100000000;
    // crcVal += computeCRC(data);
    // Serial.print(", crc: ");
    // Serial.print(crcVal, BIN);

    Serial.println();
    // 1100 1011 0101 01 0100 001111
    // 1100 1011 1000 10 1100 011111
    // 1100 1011 0000 01 01
    delay(1000);
}