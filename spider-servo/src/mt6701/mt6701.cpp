#include "mt6701.h"
#include "math.h"
#include "esp_log.h"

#define CRC_MASK             0b000000000000000000111111
#define MFC_MASK             0b000000000000001111000000
#define MFC_STRENGTH_MASK    0b000000000000000011000000
#define ANGLE_MASK           0b111111111111110000000000

#define TAG "MT6701-driver"

// uint64_t micros() {
//   return (uint64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS * 1000);
// }

void MT6701::begin(gpio_num_t sck, gpio_num_t miso, gpio_num_t ss) {
    ESP_LOGI(TAG, "Setting up...");
    _csPin = ss;
    _sckPin = sck;
    _misoPin = miso;
    
    gpio_set_direction(_csPin, GPIO_MODE_OUTPUT);
    gpio_set_level(_csPin, 1);
    //SPI.begin(_sckPin, _misoPin, _csPin);
    _spiSetup();
}

void MT6701::_spiSetup() {
    ESP_LOGI(TAG, "SPI setting up...");
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .mosi_io_num = -1,
        .miso_io_num = _misoPin,        
        .sclk_io_num = _sckPin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1        
    };
    // spi_device_interface_config_t devcfg = {
    //     .mode = 1,
    //     .clock_speed_hz = 1000000,        
    //     .spics_io_num = _csPin,
    //     .queue_size = 1
    // };
    spi_device_interface_config_t devcfg = {
      .command_bits=0,
      .address_bits=0,
      .dummy_bits=0,
      .mode=1,
      .duty_cycle_pos=0,
      .cs_ena_pretrans=4,
      .cs_ena_posttrans=0,
      .clock_speed_hz=1000000,
      .input_delay_ns=0,
      .spics_io_num=_csPin,
      .flags = 0,
      .queue_size=1,
      .pre_cb=NULL,
      .post_cb=NULL,
  } ;    
    //Initialize the SPI bus
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &_spi);
    ESP_ERROR_CHECK(ret);

    _spiTransaction.flags = SPI_TRANS_USE_RXDATA;
    _spiTransaction.length = 24;
    _spiTransaction.rxlength = 24;
    _spiTransaction.tx_buffer = NULL;
    _spiTransaction.rx_buffer = NULL;
}

uint32_t MT6701::readData() {
  esp_err_t ret=spi_device_polling_transmit(_spi, &_spiTransaction);
  ESP_ERROR_CHECK(ret);

  return (_spiTransaction.rx_data[0] << 16) | (_spiTransaction.rx_data[1] << 8) | _spiTransaction.rx_data[2];
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

  // char binaryFormat[4 * 8 + 1];
  // binaryFormat[4 * 8] = '\0'; 
  // for (int i = 0; i < 4 * 8; i++) {
  //     if ((data >> i) && 0b1) {
  //         binaryFormat[i] = '1';
  //     } else {
  //         binaryFormat[i] = '0';
  //     }
  // }
  // ESP_LOGE(TAG, "encoder val %s", binaryFormat);

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