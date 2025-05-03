#include <Arduino.h>
//#include "mt6701/mt6701.h"
#include "mt6701/mt6701_emulator.h"
#include "QuickPID.h"

#define SPI_MISO   5
#define SPI_SCLK   4
#define SPI_CS     7

#define PIN_CURRENT_SENSOR 0

#define MOTOR_PWM 1
#define MOTOR_DIR 2
#define MOTOR_BRAKE 3

//Define Variables we'll be connecting to
float Setpoint = 100.0, Input, Output;

//Specify the links and initial tuning parameters
// float Kp = 2, Ki = 5, Kd = 1;
//float Kp = 8.78, Ki = 0.75, Kd = 3.77;

//float Kp = 300, Ki = 20, Kd = 10;
//float Kp = 17, Ki = 0, Kd = 0.5;
// float Kp = 17, Ki = 10, Kd = 0;
// float Kp = 10, Ki = 0.5, Kd = 0.5;
float Kp = 10, Ki = 0.01, Kd = 0.5;

float POn = 1.0;          // proportional on Error to Measurement ratio (0.0-1.0), default = 1.0
float DOn = 0.0;          // derivative on Error to Measurement ratio (0.0-1.0), default = 0.0
bool pidLoop = true;
const uint32_t sampleTimeUs = 100;

byte outputStep = 10;
byte hysteresis = 1;
int setpoint = 100;       // 1/3 of range for symetrical waveform
int output = 255;       // 1/3 of range for symetrical waveform

MT6701 sensor;
QuickPID positionPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, QuickPID::DIRECT);

#define OUTPUT_MID_POINT 255
#define INPUT_SCALE 100

void setSpeedAndDirection() {
    //sensor.setSpeed(Output);    

    bool direction;
    uint8_t speed;

    if (Output > OUTPUT_MID_POINT) {
        direction = true;
        speed = Output - OUTPUT_MID_POINT;
    } else {
        direction = false;
        speed = OUTPUT_MID_POINT - Output - 2;
    }
    // speed = speed * 2;
    if ((speed) < 20) {
        speed = 0;
    }
    digitalWrite(MOTOR_DIR, direction);
    analogWrite(MOTOR_PWM, speed);
    // Serial.print(direction);
    // Serial.print(" ");
    // Serial.println(speed);
}

float avg(int inputVal) {
    static int arrDat[16];
    static int pos;
    static long sum;
    pos++;
    if (pos >= 16) pos = 0;
    sum = sum - arrDat[pos] + inputVal;
    arrDat[pos] = inputVal;
    return (float)sum / 16.0;
}
  
void taskComputePid(void * parameter){
    for(;;){
        bool result;

        // sensor.computePosition();
        result = sensor.read(&Input, NULL, NULL, NULL);
        Input = Input * INPUT_SCALE;
        
        if (!result) {
            Serial.println("CRC ERROR");
            analogWrite(MOTOR_PWM, 0);
        } else {
            if (positionPID.autoTune) // Avoid dereferencing nullptr after _myPID.clearAutoTune()
            {
              switch (positionPID.autoTune->autoTuneLoop()) {
                case positionPID.autoTune->AUTOTUNE:
                  setSpeedAndDirection();
                  break;
          
                case positionPID.autoTune->TUNINGS:
                    positionPID.autoTune->setAutoTuneConstants(&Kp, &Ki, &Kd); // set new tunings
                    positionPID.SetMode(QuickPID::AUTOMATIC); // setup PID
                    positionPID.SetSampleTimeUs(sampleTimeUs);
                    positionPID.SetTunings(Kp, Ki, Kd, POn, DOn); // apply new tunings to PID
                    Setpoint = setpoint;
                  break;
          
                case positionPID.autoTune->CLR:
                  if (!pidLoop) {
                    positionPID.clearAutoTune(); // releases memory used by AutoTune object
                    pidLoop = true;
                  }
                  break;
              }
            }
            if (pidLoop) {
                positionPID.Compute();
                setSpeedAndDirection();
            }
            // sensor.setSpeed(Output);
            // sensor.computePosition();
        }    
        vTaskDelay(1/ portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("Activating sensor...");
    sensor.begin(SPI_SCLK, SPI_MISO, SPI_CS);

    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(MOTOR_DIR, OUTPUT);    
    pinMode(MOTOR_BRAKE, OUTPUT);
    analogWrite(MOTOR_PWM, 0);

    Setpoint = random(0,TWO_PI * INPUT_SCALE);
    positionPID.SetSampleTimeUs(sampleTimeUs);
    positionPID.SetOutputLimits(0, 255*2);
    positionPID.SetMode(QuickPID::AUTOMATIC);
    // positionPID.AutoTune(tuningMethod::ZIEGLER_NICHOLS_PID);
    // positionPID.autoTune->autoTuneConfig(outputStep, hysteresis, setpoint, output, QuickPID::DIRECT, 1, sampleTimeUs);

    xTaskCreate(
        taskComputePid,    // Function that should be called
        "Compute PID",   // Name of the task (for debugging)
        1000,            // Stack size (bytes)
        NULL,            // Parameter to pass
        1,               // Task priority
        NULL             // Task handle
    );    
}

static String dataBuffer = "";

void loop() {
    uint8_t readByte;
    if (Serial.available()) {
        readByte = Serial.read();
        if (readByte == '\r') {
            Setpoint = dataBuffer.toFloat();
            dataBuffer = "";
        } else {
            dataBuffer += (char) readByte;
        }
        //Serial.print(dataBuffer);
    }
    Serial.print(">angle: ");
    Serial.print(Input);
    Serial.print(", target: ");
    Serial.print(Setpoint);
    // Serial.print(", output_scaled: ");
    // Serial.print(Output/40);
    Serial.print(", output: ");
    Serial.print(Output);
    Serial.println();
    delay(100);
}