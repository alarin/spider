#include <Arduino.h>
#include "mt6701/mt6701.h"
#include "QuickPID.h"

#define SPI_MISO   3
#define SPI_SCLK   2
#define SPI_CS     5

#define MOTOR_PWM 7
#define MOTOR_DIR 6
#define MOTOR_BRAKE 8

//Define Variables we'll be connecting to
float Setpoint, Input, Output;

//Specify the links and initial tuning parameters
// float Kp = 2, Ki = 5, Kd = 1;
float Kp = 1, Ki = 0, Kd = 0;

SPISensor sensor;
QuickPID positionPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, QuickPID::DIRECT);

void setup() {
    Serial.begin(115200);
    Serial.println("Activating sensor...");
    sensor.begin(SPI_SCLK, SPI_MISO, SPI_CS);

    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(MOTOR_DIR, OUTPUT);
    analogWrite(MOTOR_DIR, false);
    pinMode(MOTOR_BRAKE, OUTPUT);

    Setpoint = 5.0;
    float angle;
    sensor.read(&angle, NULL, NULL, NULL);
    Input = angle;
    positionPID.SetSampleTimeUs(10000);
    positionPID.SetMode(QuickPID::AUTOMATIC);
    // positionPID.AutoTune(tuningMethod::ZIEGLER_NICHOLS_PID);
}

void loop() {
    float angle;
    bool result;

    result = sensor.read(&angle, NULL, NULL, NULL);
    if (!result) {
        Serial.println("CRC ERROR");
        analogWrite(MOTOR_PWM, 0);
    } else {
        Input = angle;
        positionPID.Compute();
        analogWrite(MOTOR_PWM, (uint8_t) Output);

        Serial.print(">angle: ");
        Serial.print(angle);
        Serial.print(", output: ");
        Serial.print( (uint8_t) Output);
        Serial.println();
    }
    delay(1);
}