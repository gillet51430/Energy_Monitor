#include "ADS1256_Custom.h"

// Define the nominal LSB value based on 24-bit ADC (2^23 for signed)
#define ADS1256_MAX_VALUE 8388607.0 // (2^23 - 1) for signed 24-bit

// 1.92 MHz SPI speed for ADS1256 (max 20MHz, but lower is safer)
#define SPI_SPEED 1920000 

// Define the Clock rate
#define f_CLKIN 7.68 // in MHz
#define tau 130.21 // in s (typical value from datasheet 1-f_CLKIN)

// define waiting values from the datasheet
#define t6 5     // Delay to wait when reading a registry
#define t10 8    // Delay to wait after a reset
#define t11a 4    // Delay to wait after RREG, WREG, RDATA
#define t11b 4    // Delay to wait after RDATAC, SYNC

ADS1256::ADS1256(uint8_t cs_pin, uint8_t drdy_pin, uint8_t pdwn_pin, double vref_volts, SPIClass& spi)
  : _cs_pin(cs_pin),
    _drdy_pin(drdy_pin),
    _pdwn_pin(pdwn_pin),
    _vref_volts(vref_volts),
    _spi(spi)
{
  // Le reste de l'initialisation se fait ici
  _dataReady = false;

  // Default PGA and gain value (will be set in begin())
    _current_pga_gain_code = ADS1256_ADCON_PGA_1;
    updatePGAGainValue();
}

void ADS1256::begin() {
  // Configuration des broches
  pinMode(_cs_pin, OUTPUT);
  digitalWrite(_cs_pin, LOW); // CS bas pour commencer
  delayMicroseconds(t10*tau);         // See datasheet for timing requirements
  pinMode(_drdy_pin, INPUT); // DRDY est une sortie du capteur, donc une entrée pour l'ESP32

  if (_pdwn_pin != 255) { 
      pinMode(_pdwn_pin, OUTPUT);
      digitalWrite(_pdwn_pin, HIGH);
  }
  
  // Attache la fonction d'interruption statique en lui passant un pointeur
  // vers l'instance actuelle de l'objet ('this')
  attachInterruptArg(digitalPinToInterrupt(_drdy_pin), checkDReady_ISR, this, FALLING);

  //DRDY
  pinMode(_drdy_pin, INPUT);
  //Reset
  pinMode(_pdwn_pin, OUTPUT);
  //We do a manual chip reset on the ADS1256 using the pin - Datasheet Page 27/ RESET
  digitalWrite(_pdwn_pin, LOW);
  delay(100);
  digitalWrite(_pdwn_pin, HIGH); //RESET is set to high
  delay(500);

  _spi.begin();
}

void ADS1256::waitForDRDY()
{
  unsigned long timeout = millis();
  // Wait for DRDY to go low (active low)
  while (_dataReady == false)
  {
    if (millis() - timeout > 1000)
    { 
      // Timeout after 1 second
      Serial.println("DRDY timeout!");
      // Handle error, maybe reset or put device to power down
      return;
    }
  }
  if (_dataReady) {
      _dataReady = false;
  }
}

void IRAM_ATTR ADS1256::checkDReady_ISR(void* arg) {
  // 1. On reçoit le pointeur 'this' sous forme de void*
  // 2. On le "cast" (reconvertit) en pointeur du type de notre classe
  ADS1256* instance = static_cast<ADS1256*>(arg);

  // 3. On utilise ce pointeur pour accéder à la variable membre de la bonne instance
  instance->_dataReady = true;
}

void ADS1256::writeRegister(uint8_t reg, uint8_t value)
{
  _spi.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1)); // initialize SPI with  clock, MSB first, SPI Mode1
  digitalWrite(_cs_pin, LOW);            // Pull CS low
  _spi.transfer(ADS1256_CMD_WREG | reg); // Send WREG command with register address
  _spi.transfer(0x00);                   // Number of registers to write - 1
  _spi.transfer(value);                  // Send data
  digitalWrite(_cs_pin, HIGH);           // Pull CS high
  _spi.endTransaction();
  delayMicroseconds(t11a*tau);           // t11 in datasheet
}

uint8_t ADS1256::readRegister(uint8_t reg)
{
  _spi.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1)); // initialize SPI with  clock, MSB first, SPI Mode1
  digitalWrite(_cs_pin, LOW);            // Pull CS low
  _spi.transfer(ADS1256_CMD_RREG | reg); // Send RREG command with register address
  _spi.transfer(0x00);                   // Number of registers to read - 1
  delayMicroseconds(t6);                 // t6 in datasheet
  uint8_t value = _spi.transfer(0xFF);   // Read data (8bits) by sending dummy byte
  digitalWrite(_cs_pin, HIGH);           // Pull CS high
  _spi.endTransaction();
  delayMicroseconds(t11a*tau);           // t11 in datasheet
  return value;
}

void ADS1256::reset()
{
  _spi.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1)); // initialize SPI with  clock, MSB first, SPI Mode1
  digitalWrite(_cs_pin, LOW);             // Pull CS low
  _spi.transfer(ADS1256_CMD_RESET);       // Send RESET command
  digitalWrite(_cs_pin, HIGH);            // Pull CS high
  _spi.endTransaction();
  delay(10);                              // Give time to the reset to start
  while (digitalRead(_drdy_pin) == HIGH); // It's mandatory to wait for DRDY to go low after a reset
}

void ADS1256::selfCalibration()
{
  _spi.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1)); // initialize SPI with  clock, MSB first, SPI Mode1
  digitalWrite(_cs_pin, LOW);             // Pull CS low
  _spi.transfer(ADS1256_CMD_SELFCAL);     // Send SELF_CALIBRATION command
  digitalWrite(_cs_pin, HIGH);            // Pull CS high
  _spi.endTransaction();
  delay(1000);                            // Give time to the calibration. The time is depending from the rate with a max of 800.3ms at 2.5 SPS
  while (digitalRead(_drdy_pin) == HIGH); // It's mandatory to wait for DRDY to go low after a reset
}

void ADS1256::updatePGAGainValue()
{
    switch (_current_pga_gain_code)
    {
    case ADS1256_ADCON_PGA_1:
        _current_pga_gain_value = 1.0;
        break;
    case ADS1256_ADCON_PGA_2:
        _current_pga_gain_value = 2.0;
        break;
    case ADS1256_ADCON_PGA_4:
        _current_pga_gain_value = 4.0;
        break;
    case ADS1256_ADCON_PGA_8:
        _current_pga_gain_value = 8.0;
        break;
    case ADS1256_ADCON_PGA_16:
        _current_pga_gain_value = 16.0;
        break;
    case ADS1256_ADCON_PGA_32:
        _current_pga_gain_value = 32.0;
        break;
    case ADS1256_ADCON_PGA_64:
        _current_pga_gain_value = 64.0;
        break;
    default:
        _current_pga_gain_value = 1.0;
        break; // Should not happen
    }
}

int32_t ADS1256::readRawData() {
  _spi.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  waitForDRDY();
  digitalWrite(_cs_pin, LOW);
  _spi.transfer(ADS1256_CMD_RDATA);
  delayMicroseconds(t6*tau); 

  uint32_t result = 0;
  result |= (uint32_t)_spi.transfer(0x00) << 16; // MSB
  result |= (uint32_t)_spi.transfer(0x00) << 8;  // Mid-Byte
  result |= (uint32_t)_spi.transfer(0x00);       // LSB

  digitalWrite(_cs_pin, HIGH);
  _spi.endTransaction();

  // Manage negative values (two's complement)
  if (result & 0x00800000) {
    result |= 0xFF000000; 
  }

  return (int32_t)result;
}

float ADS1256::convertToVoltage(int32_t rawData) {
    // La formule est : Voltage = (Valeur_ADC / Résolution) * (Plage_de_tension / Gain)
    return (float)rawData / ADS1256_MAX_VALUE * (_vref_volts * 2.0) / _current_pga_gain_value;
}

void ADS1256::setPGA(uint8_t pga_gain_code) {
  uint8_t adcon_value = readRegister(ADS1256_REG_ADCON);
  
  adcon_value &= 0xF8;
  adcon_value |= pga_gain_code;
  writeRegister(ADS1256_REG_ADCON, adcon_value);
  _current_pga_gain_code = pga_gain_code;
  updatePGAGainValue(); // Cette fonction met à jour _current_pga_gain_value
  reset(); // Reset the ADC to apply the new PGA setting
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
  reset(); // Un reset est souvent nécessaire pour appliquer de nouveaux paramètres de timing
}

void ADS1256::differentialChannelValue(uint8_t channelN, uint8_t channelP) {
  if (channelN > 7 || channelP > 7)
  {
    Serial.println("Error: Channel numbers must be between 0 and 7.");
  } else {
    writeRegister(ADS1256_REG_MUX, (channelP << 4) | channelN);
  }
}

void ADS1256::startContinuousConversion()
{
    // Continuous mode (RDATAC) requires a SYNC and WAKEUP command before the first RDATAC
    // and then only RDATAC for subsequent reads.
    _spi.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
    _spi.transfer(ADS1256_CMD_SYNC);   // Sync the ADC
    _spi.transfer(ADS1256_CMD_WAKEUP); // Wake up from standby if needed (often auto-synced)
    waitForDRDY();                              // Wait for first conversion to be ready
    digitalWrite(_cs_pin, LOW);                 // Pull CS low for continuous read
    _spi.transfer(ADS1256_CMD_RDATAC);          // Send RDATAC command
    delayMicroseconds(t6);                 // t6 in datasheet (delay before 1st data get available)
    _spi.endTransaction();
    // DRDY will pulse low for each new conversion data
}

void ADS1256::stopContinuousConversion()
{
    _spi.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
    waitForDRDY();
    _spi.transfer(ADS1256_CMD_SDATAC); // Send SDATAC command
    digitalWrite(_cs_pin, HIGH);       // Pull CS high to stop continuous mode
    // delay(50);                         // Give it time to stop
    _spi.endTransaction();
}

int32_t ADS1256::readContinuousData_LSB()
{
    uint32_t raw_data = 0;
    waitForDRDY(); // Wait for DRDY to go low for new data
    // No CS low/high here because we are already in RDATAC mode
    // Read 3 bytes of data
    raw_data = ((long)_spi.transfer(0x00) << 16);
    raw_data |= ((long)_spi.transfer(0x00) << 8);
    raw_data |= _spi.transfer(0x00);

    // Convert 24-bit two's complement to signed long
    if (raw_data & 0x800000)
    {                           // If MSB is set, it's a negative number
        raw_data |= 0xFF000000; // Sign-extend to 32 bits
    }
    return raw_data;
}

void readContinuousTest() //Reads the recently selected channel using RDATAC
{
  byte outputBuffer[3]; //3-byte (24-bit) buffer for the fast acquisition - Single-channel, continuous
  
  _spi.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1)); // initialize SPI with  clock, MSB first, SPI Mode1
  digitalWrite(_cs_pin, LOW); //REF: P34: "CS must stay low during the entire command sequence"

  //These variables serve only testing purposes!
  //uint32_t loopcounter = 0;
  //StartTime = micros();
  //--------------------------------------------

  while (_dataReady == false) {} //Wait until DRDY does low, then issue the command
  _spi.transfer(B00000011);  //Issue RDATAC (0000 0011) command after DRDY goes low
  delayMicroseconds(7); //Wait t6 time (~6.51 us) REF: P34, FIG:30.

  while (Serial.read() != 's')
  {
    //while (GPIOA->regs->IDR & 0x0004){} //direct port access to A2 (DRDY) pin - less reliable polling alternative
    while (_dataReady == false) {} //waiting for the dataReady ISR
    //Reading a single input continuously using the RDATAC
    //step out the data: MSB | mid-byte | LSB
    outputBuffer[0] = _spi.transfer(0); // MSB comes in
    outputBuffer[1] = _spi.transfer(0); // Mid-byte
    outputBuffer[2] = _spi.transfer(0); // LSB - final conversion result
    //After this, DRDY should go HIGH automatically
    Serial.write(outputBuffer, sizeof(outputBuffer)); //this buffer is [3]
    _dataReady = false; //reset dataReady manually

    /*
      //These variables only serve test purposes!
      loopcounter++;
      //if(micros() - StartTime >= 5000000) //5 s
      if(loopcounter >= 150000)
      {
             Serial.print(" Loops: ");
             Serial.println(loopcounter++);
             Serial.println(micros() - StartTime);
             break; //exit the whole thing
      }
    */
  }
  _spi.transfer(B00001111); //SDATAC stops the RDATAC - the received 's' just breaks the while(), this stops the acquisition
  digitalWrite(_cs_pin, HIGH); //We finished the command sequence, so we switch it back to HIGH
  _spi.endTransaction();
}

// ---------------------------------------------------
// Helper functions
// ---------------------------------------------------
String ADS1256::binaryToString(uint8_t val) {
  String result = "";
  for (int i = 7; i >= 0; i--) {
    result += (bitRead(val, i) == 1) ? "1" : "0";
  }
  return result;
}

void ADS1256::printRegister(const char* regName, uint8_t regValue, bool lrlf) {
  // Use %-8s to left-align the register name in an 8-character wide field.
  if (lrlf) {
    Serial.printf("%-8s: %s | (HEX: 0x%02X)\n", regName, binaryToString(regValue).c_str(), regValue);
  } else {
    Serial.printf("%-8s: %s | (HEX: 0x%02X)", regName, binaryToString(regValue).c_str(), regValue);
  }
}

void ADS1256::printAllRegisters() {
  Serial.println("\n--- Reading Register Values ---");
  printRegister("STATUS", readRegister(ADS1256_REG_STATUS) & 0x0F, true); // Mask reserved bits
  printRegister("MUX", readRegister(ADS1256_REG_MUX),true);
  printRegister("ADCON", readRegister(ADS1256_REG_ADCON),true);
  printRegister("DRATE", readRegister(ADS1256_REG_DRATE),true);
}