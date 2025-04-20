#include "mt6701.h"

#define CRC_MASK             0b000000000000000000111111
#define MFC_MASK             0b000000000000001111000000
#define MFC_STRENGTH_MASK    0b000000000000000011000000
#define ANGLE_MASK           0b111111111111110000000000

void SPISensor::begin(int8_t sck, int8_t miso, int8_t ss) {
    _csPin = ss;
    _sckPin = sck;
    _misoPin = miso;

    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    SPI.begin(_sckPin, _misoPin, _csPin);
    _spiSetup();
}

void SPISensor::_spiSetup() {
    SPI.setBitOrder(MSBFIRST);
    // SPI.setDataMode(SPI_MODE2);
    // SPI.setFrequency(1000000);
    SPI.setDataMode(SPI_MODE1);
    SPI.setFrequency(1000000);
}

uint8_t SPISensor::readByte() {
    digitalWrite(_csPin, LOW);
    uint8_t val = SPI.transfer(0x00);
    digitalWrite(_csPin, HIGH);
    return val;
}

uint16_t SPISensor::readWord() {
    digitalWrite(_csPin, LOW);
    uint16_t val = SPI.transfer16(0x0000);
    digitalWrite(_csPin, HIGH);
    return val;
}
uint32_t SPISensor::read24() {
  digitalWrite(_csPin, LOW);
  uint32_t val = SPI.transfer(0x00);
  val <<= 8;
  val |=  SPI.transfer(0x00);  
  val <<= 8;
  val |=  SPI.transfer(0x00);
  SPI.transfer(0xFF);  
  digitalWrite(_csPin, HIGH);
  return val;
}

void SPISensor::readSSI(uint8_t* data){
	uint8_t tmp[3];
    digitalWrite(_csPin, LOW);
	tmp[0] = SPI.transfer(0xFF);
	tmp[1] = SPI.transfer(0xFF);
	tmp[2] = SPI.transfer(0xFF);
	memcpy(data, &tmp, 3);
    digitalWrite(_csPin, HIGH);
}

uint8_t SPISensor::computeCRC(uint16_t D, uint8_t Mg) {
  // Combine D (14 bits) and Mg (4 bits) into an 18-bit value
  uint32_t data = ((uint32_t)D << 4) | (Mg & 0x0F);
  uint8_t crc = 0; // Initialize CRC to 0

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
  
  return crc; // Returns 6-bit CRC
}

void SPISensor::mt6701_read_raw(uint16_t *angle_raw, mt6701_status_t *field_status, bool *button_pushed, bool *track_loss ){
	uint8_t res;
	uint8_t data[3];
	uint8_t status;
	uint16_t angle_u16;
  uint8_t crc;

  readSSI(data);

  angle_u16  = (uint16_t)(data[1] >> 2);
  angle_u16 |= ((uint16_t)data[0] << 6);
  
  status  = (data[2] >> 6);
  status |= (data[1] & 0x03) << 2;

  crc = data[2] & 0b00111111;

  // I dont check CRC6, becouse i soo stupid to implement this rare shit

  if (computeCRC(angle_u16, status) != crc) {
    uint8_t printval = 0b00000000;
    Serial.print("WRONG CRC ");
    printval += computeCRC(angle_u16, status);
    Serial.print(printval, BIN);
    Serial.print(" received: ");
    printval = 0b00000000;
    printval += crc;
    Serial.println(printval, BIN);
  }

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

  if(angle_raw != NULL){
    *angle_raw = angle_u16; 
  }
}
