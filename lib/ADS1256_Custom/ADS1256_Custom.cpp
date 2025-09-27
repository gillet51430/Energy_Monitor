#include "ADS1256_Custom.h"

// Define the nominal LSB value based on 24-bit ADC (2^23 for signed)
#define ADS1256_MAX_VALUE 8388607.0 // (2^23 - 1) for signed 24-bit

// Define the Clock rate
#define f_CLKIN 7.68 // in MHz
#define tau 130.2 // in ns (typical value from datasheet 1-f_CLKIN)

// define waiting values from the datasheet
#define t6 5     // Delay to wait when reading a registry
#define t10 8    // Delay to wait after a reset

// Le constructeur utilise une liste d'initialisation (plus efficace)
ADS1256::ADS1256(uint8_t cs_pin, uint8_t drdy_pin, uint8_t pdwn_pin, double vref_volts, SPIClass& spi)
    : _cs_pin(cs_pin),
      _drdy_pin(drdy_pin),
      _pdwn_pin(pdwn_pin),
      _vref_volts(vref_volts),
      _spi(spi)
{
    // Le reste de l'initialisation se fait ici
    _dataReady = false;
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

// Implémentation de fonction de consomation de l'interruption
bool ADS1256::isDataReady() {
    if (_dataReady) {
        _dataReady = false; // On réinitialise l'indicateur pour la prochaine donnée
        return true;
    }
    return false;
}

// Implémentation de la fonction d'interruption statique
void IRAM_ATTR ADS1256::checkDReady_ISR(void* arg) {
    // 1. On reçoit le pointeur 'this' sous forme de void*
    // 2. On le "cast" (reconvertit) en pointeur du type de notre classe
    ADS1256* instance = static_cast<ADS1256*>(arg);

    // 3. On utilise ce pointeur pour accéder à la variable membre de la bonne instance
    instance->_dataReady = true;
}

void ADS1256::reset()
{
    SPI.beginTransaction(SPISettings(1920000, MSBFIRST, SPI_MODE1)); // initialize SPI with  clock, MSB first, SPI Mode1

    // Send the reset command to the ADS1256
    digitalWrite(_cs_pin, LOW);         // Pull CS low
    delayMicroseconds(t10*tau);         // See datasheet for timing requirements
    while (_dataReady == false) {}      //Wait for DRDY to go LOW
    _spi.transfer(ADS1256_CMD_RESET);   // Send RESET command
    isDataReady();

    // Wait for the reset to complete
    while (_dataReady == false) {}      //Wait for DRDY to go LOW
    digitalWrite(_cs_pin, HIGH);        // Pull CS high
    isDataReady();                      // Wait for DRDY to go low indicating readiness}

    SPI.endTransaction();
}

void ADS1256::writeRegister(uint8_t reg, uint8_t value)
{
    _spi.beginTransaction(SPISettings(1920000, MSBFIRST, SPI_MODE1)); // initialize SPI with  clock, MSB first, SPI Mode1
    digitalWrite(_cs_pin, LOW);            // Pull CS low
    delayMicroseconds(t10*tau);            // See datasheet for timing requirements
    _spi.transfer(ADS1256_CMD_WREG | reg); // Send WREG command with register address
    _spi.transfer(0x00);                   // Number of registers to write - 1
    _spi.transfer(value);                  // Send data
    digitalWrite(_cs_pin, HIGH);           // Pull CS high
    isDataReady();                         // Wait for DRDY to go low indicating readiness
    _spi.endTransaction();
}

uint8_t ADS1256::readRegister(uint8_t reg)
{
    _spi.beginTransaction(SPISettings(1920000, MSBFIRST, SPI_MODE1)); // initialize SPI with  clock, MSB first, SPI Mode1
    digitalWrite(_cs_pin, LOW);            // Pull CS low
    delayMicroseconds(t10*tau);            // See datasheet for timing requirements
    _spi.transfer(ADS1256_CMD_RREG | reg); // Send RREG command with register address
    _spi.transfer(0x00);                   // Number of registers to read - 1
    delayMicroseconds(t6);                 // t6 in datasheet (RREG command delay)
    uint8_t value = _spi.transfer(0xFF);   // Read data
    digitalWrite(_cs_pin, HIGH);           // Pull CS high
    isDataReady();                         // Wait for DRDY to go low indicating readiness
    _spi.endTransaction();
    return value;
}















// Helper function to convert a byte to a binary string for printf
String ADS1256::binaryToString(uint8_t val) {
  String result = "";
  for (int i = 7; i >= 0; i--) {
    result += (bitRead(val, i) == 1) ? "1" : "0";
  }
  return result;
}

// Helper function to print a register's name and value in a consistent, aligned format
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