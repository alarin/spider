// https://github.com/I-AM-ENGINEER/MT6701-arduino/blob/main/src/mt6701/mt6701.c
#include <Arduino.h>
#include <SPI.h>

class SPISensor {
    public:
      typedef enum{
        MT6701_STATUS_NORM			= 0x0,
        MT6701_STATUS_FIELD_STRONG	= 0x1,
        MT6701_STATUS_FIELD_WEAK	= 0x2,
        MT6701_STATUS_FIELD_ERROR	= 0x3,
      } mt6701_status_t;
    
      void begin(int8_t sck, int8_t miso, int8_t ss);
      uint8_t readByte();
      uint16_t readWord();
      uint32_t read24();
      void readSSI(uint8_t* data);
  
      void mt6701_read_raw(uint16_t *angle_raw, mt6701_status_t *field_status, bool *button_pushed, bool *track_loss );
  
    private:
      uint8_t computeCRC(uint16_t D, uint8_t Mg);
      uint8_t _csPin;
      uint8_t _sckPin;
      uint8_t _misoPin;
      void _spiSetup();
  };
