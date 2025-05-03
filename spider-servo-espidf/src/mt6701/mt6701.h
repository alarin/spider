#include <stdio.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"

class MT6701 {
    public:
      typedef enum{
        MT6701_STATUS_NORM			= 0x0,
        MT6701_STATUS_FIELD_STRONG	= 0x1,
        MT6701_STATUS_FIELD_WEAK	= 0x2,
        MT6701_STATUS_FIELD_ERROR	= 0x3,
      } mt6701_status_t;
    
      // SPISensor(): angleKF(1, 1, 0.01) {
      // }

      void begin(gpio_num_t sck, gpio_num_t miso, gpio_num_t ss);
  
      bool read(double *angle, mt6701_status_t *field_status, bool *button_pushed, bool *track_loss );
    protected:
      uint32_t readData();
    private:
      gpio_num_t _csPin;
      gpio_num_t _sckPin;
      gpio_num_t _misoPin;
      spi_device_handle_t spi;
      void _spiSetup();      
    
      bool isCRCValid(uint32_t raw_data);
  };
