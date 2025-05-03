#include "mt6701.h"
#include "math.h"

#define CRC_MASK             0b000000000000000000111111
#define MFC_MASK             0b000000000000001111000000
#define MFC_STRENGTH_MASK    0b000000000000000011000000
#define ANGLE_MASK           0b111111111111110000000000

void MT6701::begin(gpio_num_t sck, gpio_num_t miso, gpio_num_t ss) {
    _csPin = ss;
    _sckPin = sck;
    _misoPin = miso;
    
    gpio_set_direction(_csPin, GPIO_MODE_OUTPUT);
    gpio_set_level(_csPin, 1);
    //SPI.begin(_sckPin, _misoPin, _csPin);
    _spiSetup();
}

void MT6701::_spiSetup() {
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = _misoPin,
        .sclk_io_num = _sckPin
    };
    spi_device_interface_config_t devcfg = {
        .mode = 1,
        .clock_speed_hz = 1000000,        
        .spics_io_num = _csPin
    };
    //Initialize the SPI bus
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);   
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);     
    // SPI.setBitOrder(MSBFIRST);
    // // SPI.setDataMode(SPI_MODE2);
    // // SPI.setFrequency(1000000);
    // SPI.setDataMode(SPI_MODE1);
    // SPI.setFrequency(1000000);
}

uint32_t MT6701::readData() {
  uint8_t tx[4] = {0x00, 0x00, 0x00, 0xFF};
  uint8_t rx[4] = {0};
  
  spi_transaction_t trans = {
      .length = 24,
      .tx_buffer = tx,
      .rx_buffer = rx
  };
  
  esp_err_t ret = spi_device_transmit(spi, &trans);
  if(ret != ESP_OK) {
      // Handle error
      return 0;
  }
  
  return (rx[0] << 16) | (rx[1] << 8) | rx[2];
}

bool MT6701::isCRCValid(uint32_t raw_data) {
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

bool MT6701::read(double *angle, mt6701_status_t *field_status, bool *button_pushed, bool *track_loss ){
  uint32_t data;
  uint8_t status;
  double _angle;

  data = readData();

  // uint32_t printVal = 0b1000000000000000000000000;
  // printVal += data;
  // Serial.print(">Raw data: ");
  // Serial.print(printVal, BIN);

  if (data == 0 || !isCRCValid(data)) {
    return false;
  }

  _angle = ( ((data>>10)&0x3FFF) / (double)16384.0f ) * 360;
  if(angle != NULL){
    *angle = _angle; 
  }

  status = (data >> 6) & 0b1111;

  if(field_status != NULL){
    *field_status = (mt6701_status_t) (status & 0x03);
  }

  if(button_pushed != NULL){
    if(status & 0x04){
      *button_pushed = true; 
    }else{
      *button_pushed = false;
    }
  }
  
  if(track_loss != NULL){
    if(status & 0x08){
      *track_loss = true;
    }else{
      *track_loss = false;
    }
  }

  return true;
}