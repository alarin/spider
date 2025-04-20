#include <Arduino.h>
#include <SPI.h>
#include "SimpleKalmanFilter.h"

class SPISensor {
    public:
      typedef enum{
        MT6701_STATUS_NORM			= 0x0,
        MT6701_STATUS_FIELD_STRONG	= 0x1,
        MT6701_STATUS_FIELD_WEAK	= 0x2,
        MT6701_STATUS_FIELD_ERROR	= 0x3,
      } mt6701_status_t;
    
      // SPISensor(): angleKF(1, 1, 0.01) {
      // }

      void begin(int8_t sck, int8_t miso, int8_t ss);
  
      bool read(float *angle, mt6701_status_t *field_status, bool *button_pushed, bool *track_loss );
      bool getFilteredAngle(float *angle);
    private:
      SimpleKalmanFilter angleKF{1, 1, 0.01};
      uint8_t _csPin;
      uint8_t _sckPin;
      uint8_t _misoPin;
      void _spiSetup();      
    
      uint32_t readData();
      bool isCRCValid(uint32_t raw_data);
  };
