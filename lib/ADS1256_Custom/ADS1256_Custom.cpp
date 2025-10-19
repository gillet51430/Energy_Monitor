// Version: 1.1.0
#include "ADS1256_Custom.h"

#define ADS1256_MAX_VALUE 8388607.0 
#define SPI_SPEED 1920000 
#define f_CLKIN 7.68 
#define tau 130.21 
#define t6 5
#define t10 8
#define t11a 4
#define t11b 4

ADS1256::ADS1256(uint8_t cs_pin, uint8_t drdy_pin, uint8_t pdwn_pin, double vref_volts, SPIClass& spi)
  : _cs_pin(cs_pin),
    _drdy_pin(drdy_pin),
    _pdwn_pin(pdwn_pin),
    _vref_volts(vref_volts),
    _spi(spi)
{
  _dataReady = false;
  _current_pga_gain_code = ADS1256_ADCON_PGA_1;
  updatePGAGainValue();
}

void ADS1256::begin() {
  pinMode(_cs_pin, OUTPUT);
  digitalWrite(_cs_pin, LOW);
  delayMicroseconds(t10*tau);
  pinMode(_drdy_pin, INPUT);

  if (_pdwn_pin != 255) { 
      pinMode(_pdwn_pin, OUTPUT);
      digitalWrite(_pdwn_pin, HIGH);
  }
  
  attachInterruptArg(digitalPinToInterrupt(_drdy_pin), checkDReady_ISR, this, FALLING);

  pinMode(_drdy_pin, INPUT);
  pinMode(_pdwn_pin, OUTPUT);
  digitalWrite(_pdwn_pin, LOW);
  delay(100);
  digitalWrite(_pdwn_pin, HIGH);
  delay(500);

  _spi.begin();
}

void ADS1256::waitForDRDY()
{
  unsigned long timeout = millis();
  while (_dataReady == false)
  {
    if (millis() - timeout > 1000)
    { 
      Serial.println("DRDY timeout!");
      return;
    }
  }
  _dataReady = false; // Reset flag
}

void IRAM_ATTR ADS1256::checkDReady_ISR(void* arg) {
  ADS1256* instance = static_cast<ADS1256*>(arg);
  instance->_dataReady = true;
}

void ADS1256::writeRegister(uint8_t reg, uint8_t value)
{
  _spi.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  digitalWrite(_cs_pin, LOW);
  _spi.transfer(ADS1256_CMD_WREG | reg);
  _spi.transfer(0x00);
  _spi.transfer(value);
  digitalWrite(_cs_pin, HIGH);
  _spi.endTransaction();
  delayMicroseconds(t11a*tau);
}

uint8_t ADS1256::readRegister(uint8_t reg)
{
  _spi.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  digitalWrite(_cs_pin, LOW);
  _spi.transfer(ADS1256_CMD_RREG | reg);
  _spi.transfer(0x00);
  delayMicroseconds(t6);
  uint8_t value = _spi.transfer(0xFF);
  digitalWrite(_cs_pin, HIGH);
  _spi.endTransaction();
  delayMicroseconds(t11a*tau);
  return value;
}

void ADS1256::reset()
{
  _spi.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  digitalWrite(_cs_pin, LOW);
  _spi.transfer(ADS1256_CMD_RESET);
  digitalWrite(_cs_pin, HIGH);
  _spi.endTransaction();
  delay(10);
  waitForDRDY(); // Wait for DRDY to go low after a reset
}

void ADS1256::selfCalibration()
{
  _spi.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  digitalWrite(_cs_pin, LOW);
  _spi.transfer(ADS1256_CMD_SELFCAL);
  digitalWrite(_cs_pin, HIGH);
  _spi.endTransaction();
  delay(1000); // Calibration time depends on DRATE, 1s is safe
  waitForDRDY();
}

void ADS1256::updatePGAGainValue()
{
    // Simple lookup for gain value
    _current_pga_gain_value = 1 << _current_pga_gain_code;
}

int32_t ADS1256::readRawData() {
  _spi.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  waitForDRDY();
  digitalWrite(_cs_pin, LOW);
  _spi.transfer(ADS1256_CMD_RDATA);
  delayMicroseconds(t6*tau); 

  uint32_t result = 0;
  result |= (uint32_t)_spi.transfer(0x00) << 16;
  result |= (uint32_t)_spi.transfer(0x00) << 8;
  result |= (uint32_t)_spi.transfer(0x00);

  digitalWrite(_cs_pin, HIGH);
  _spi.endTransaction();

  if (result & 0x00800000) {
    result |= 0xFF000000; 
  }

  return (int32_t)result;
}

// =================== MODIFICATION START ===================
// NOUVELLE FONCTION OPTIMISÉE
void ADS1256::readMultipleSamples(int32_t* buffer, uint32_t num_samples) {
    startContinuousConversion();
    for (uint32_t i = 0; i < num_samples; ++i) {
        waitForDRDY();
        
        uint32_t result = 0;
        result |= (uint32_t)_spi.transfer(0x00) << 16;
        result |= (uint32_t)_spi.transfer(0x00) << 8;
        result |= (uint32_t)_spi.transfer(0x00);
        
        if (result & 0x00800000) {
            result |= 0xFF000000;
        }
        buffer[i] = (int32_t)result;
    }
    stopContinuousConversion();
}
// =================== MODIFICATION END ===================

float ADS1256::convertToVoltage(int32_t rawData) {
    return (float)rawData / ADS1256_MAX_VALUE * (_vref_volts * 2.0) / _current_pga_gain_value;
}

void ADS1256::setPGA(uint8_t pga_gain_code) {
  uint8_t adcon_value = readRegister(ADS1256_REG_ADCON);
  adcon_value &= 0xF8;
  adcon_value |= pga_gain_code;
  writeRegister(ADS1256_REG_ADCON, adcon_value);
  _current_pga_gain_code = pga_gain_code;
  updatePGAGainValue();
  reset();
}

void ADS1256::setBuffer(bool enable) {
  uint8_t status_value = readRegister(ADS1256_REG_STATUS);
  if (enable) {
    status_value |= ADS1256_STATUS_BUFFER_ENABLE;
  } else {
    status_value &= ~ADS1256_STATUS_BUFFER_ENABLE;
  }
  writeRegister(ADS1256_REG_STATUS, status_value);
}

void ADS1256::setAutoCalibration(bool enable) {
  uint8_t status_value = readRegister(ADS1256_REG_STATUS);
  if (enable) {
    status_value |= ADS1256_STATUS_AUTOCAL_ENABLE;
  } else {
    status_value &= ~ADS1256_STATUS_AUTOCAL_ENABLE;
  }
  writeRegister(ADS1256_REG_STATUS, status_value);
}

void ADS1256::setDataRate(uint8_t drate_code) {
  writeRegister(ADS1256_REG_DRATE, drate_code);
  reset();
}

void ADS1256::differentialChannelValue(uint8_t channelN, uint8_t channelP) {
  if (channelN > 7 || channelP > 7) {
    Serial.println("Error: Channel numbers must be between 0 and 7.");
  } else {
    writeRegister(ADS1256_REG_MUX, (channelP << 4) | channelN);
  }
}

// =================== MODIFICATION START ===================
// Ces fonctions sont maintenant privées et optimisées
void ADS1256::startContinuousConversion() {
    _spi.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
    digitalWrite(_cs_pin, LOW);
    _spi.transfer(ADS1256_CMD_SYNC);
    delayMicroseconds(t11b);
    _spi.transfer(ADS1256_CMD_WAKEUP);
    delayMicroseconds(t11b);
    _spi.transfer(ADS1256_CMD_RDATAC);
    delayMicroseconds(t6);
}

void ADS1256::stopContinuousConversion() {
    waitForDRDY();
    digitalWrite(_cs_pin, LOW); // SDATAC needs CS to be low
    _spi.transfer(ADS1256_CMD_SDATAC);
    digitalWrite(_cs_pin, HIGH);
    _spi.endTransaction();
}
// =================== MODIFICATION END ===================

String ADS1256::binaryToString(uint8_t val) {
  String result = "";
  for (int i = 7; i >= 0; i--) {
    result += (bitRead(val, i) == 1) ? "1" : "0";
  }
  return result;
}

void ADS1256::printRegister(const char* regName, uint8_t regValue, bool lrlf) {
  if (lrlf) {
    Serial.printf("%-8s: %s | (HEX: 0x%02X)\n", regName, binaryToString(regValue).c_str(), regValue);
  } else {
    Serial.printf("%-8s: %s | (HEX: 0x%02X)", regName, binaryToString(regValue).c_str(), regValue);
  }
}

void ADS1256::printAllRegisters() {
  Serial.println("\n--- Reading Register Values ---");
  printRegister("STATUS", readRegister(ADS1256_REG_STATUS) & 0x0F, true);
  printRegister("MUX", readRegister(ADS1256_REG_MUX),true);
  printRegister("ADCON", readRegister(ADS1256_REG_ADCON),true);
  printRegister("DRATE", readRegister(ADS1256_REG_DRATE),true);
}