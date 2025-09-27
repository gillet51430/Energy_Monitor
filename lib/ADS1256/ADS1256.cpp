#include "ADS1256.h"

// Define the nominal LSB value based on 24-bit ADC (2^23 for signed)
#define ADS1256_MAX_VALUE 8388607.0 // (2^23 - 1) for signed 24-bit

// Define delay to wait
#define t6 7    //Delay to wait when reading a regitry

ADS1256::ADS1256(uint8_t cs_pin, uint8_t drdy_pin, uint8_t pdwn_pin, double vref_volts, SPIClass &spi)
    : _cs_pin(cs_pin), _drdy_pin(drdy_pin), _pdwn_pin(pdwn_pin), _vref_volts(vref_volts), _spi(spi)
{
    // Default PGA and gain value (will be set in begin())
    _current_pga_gain_code = ADS1256_ADCON_PGA_1;
    _current_pga_gain_value = 1.0;
    this->_cs_pin = cs_pin;
}

void ADS1256::begin()
{
    pinMode(_cs_pin, OUTPUT);
    pinMode(_drdy_pin, INPUT);
    pinMode(_pdwn_pin, OUTPUT);

    digitalWrite(_cs_pin, HIGH);  // CS high initially
    digitalWrite(_pdwn_pin, LOW); // Hold PWDN low to ensure power down before waking up

    delay(1000);                    // Allow time for device to power up and stabilize
    digitalWrite(_pdwn_pin, HIGH); // Bring PWDN high to wake up the ADS1256
    delay(1000);                     // Small delay after waking up

    _spi.setFrequency(2000000);  // Vitesse d'horloge SPI à 2 MHz (max 20MHz pour ADS1256)
    _spi.setDataMode(SPI_MODE1); // Mode SPI 1 (CPOL=0, CPHA=1)
    _spi.setBitOrder(MSBFIRST);  // Ordre des bits : MSB en premier

    reset(); // Reset the ADS1256 to default settings
    delay(2000);

    // Configure initial settings
    writeRegister(ADS1256_REG_STATUS, ADS1256_STATUS_BUFFER_ENABLE | ADS1256_STATUS_AUTOCAL_DISABLE);
    writeRegister(ADS1256_REG_MUX, 0x10);
    writeRegister(ADS1256_REG_ADCON, ADS1256_ADCON_PGA_1);
    _current_pga_gain_code = ADS1256_ADCON_PGA_1;
    updatePGAGainValue();

    writeRegister(ADS1256_REG_DRATE, ADS1256_DRATE_7500SPS);

    // selfCalibrate(); // Perform a self-calibration
}

void ADS1256::reset()
{
    digitalWrite(_cs_pin, LOW);       // Pull CS low
    waitForDRDY();
    _spi.transfer(ADS1256_CMD_RESET); // Send RESET command
    digitalWrite(_cs_pin, HIGH);      // Pull CS high
    delay(10);                        // Wait for the reset to complete
    waitForDRDY();                    // Wait for DRDY to go low indicating readiness
}

void ADS1256::waitForDRDY()
{
    unsigned long timeout = millis();
    // Wait for DRDY to go low (active low)
    while (digitalRead(_drdy_pin) == HIGH)
    {
        if (millis() - timeout > 1000)
        { // Timeout after 1 second
            Serial.println("DRDY timeout!");
            // Handle error, maybe reset or put device to power down
            return;
        }
    }
}

void ADS1256::writeRegister(uint8_t reg, uint8_t value)
{
    digitalWrite(_cs_pin, LOW);            // Pull CS low
    _spi.transfer(ADS1256_CMD_WREG | reg); // Send WREG command with register address
    _spi.transfer(0x00);                   // Number of registers to write - 1
    _spi.transfer(value);                  // Send data
    digitalWrite(_cs_pin, HIGH);           // Pull CS high
    waitForDRDY();                         // Wait for DRDY to go low after write
}

uint8_t ADS1256::readRegister(uint8_t reg)
{
    digitalWrite(_cs_pin, LOW);            // Pull CS low
    _spi.transfer(ADS1256_CMD_RREG | reg); // Send RREG command with register address
    _spi.transfer(0x00);                   // Number of registers to read - 1
    delayMicroseconds(t6);                 // t6 in datasheet (RREG command delay)
    uint8_t value = _spi.transfer(0x00);   // Read data
    digitalWrite(_cs_pin, HIGH);           // Pull CS high
    waitForDRDY();                         // Wait for DRDY to go low indicating readiness
    return value;
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

void ADS1256::selfCalibrate()
{
    digitalWrite(_cs_pin, LOW);         // Pull CS low
    _spi.transfer(ADS1256_CMD_SELFCAL); // Send Self Calibration command
    digitalWrite(_cs_pin, HIGH);        // Pull CS high
    waitForDRDY();                      // Wait for DRDY to go low after calibration
}

void ADS1256::startContinuousConversion(uint8_t positive_input, uint8_t negative_input)
{
    // Continuous mode (RDATAC) requires a SYNC and WAKEUP command before the first RDATAC
    // and then only RDATAC for subsequent reads.
    uint8_t mux_value = (positive_input << 4) | negative_input;
    writeRegister(ADS1256_REG_MUX, mux_value);
    _spi.transfer(ADS1256_CMD_SYNC);   // Sync the ADC
    _spi.transfer(ADS1256_CMD_WAKEUP); // Wake up from standby if needed (often auto-synced)
    // waitForDRDY();                              // Wait for first conversion to be ready
    digitalWrite(_cs_pin, LOW);                 // Pull CS low for continuous read
    _spi.transfer(ADS1256_CMD_RDATAC);          // Send RDATAC command
    delayMicroseconds(t6);                 // t6 in datasheet (delay before 1st data get available)
    // DRDY will pulse low for each new conversion data
}

void ADS1256::stopContinuousConversion()
{
    digitalWrite(_cs_pin, HIGH);       // Pull CS high to stop continuous mode
    waitForDRDY();
    _spi.transfer(ADS1256_CMD_SDATAC); // Send SDATAC command
}

long ADS1256::readContinuousData_LSB()
{
    waitForDRDY(); // Wait for DRDY to go low for new data
    // No CS low/high here because we are already in RDATAC mode
    // Read 3 bytes of data
    delayMicroseconds(20);
    long raw_data = 0;
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

double ADS1256::convertToVolts(long raw_data)
{
    // Voltage = (Raw Data / (2^23 - 1)) * VREF / PGA_Gain
    // Or (Raw Data / 2^23) * VREF / PGA_Gain for better precision.
    // ADS1256_MAX_VALUE is (2^23 - 1)
    return ((double)raw_data / ADS1256_MAX_VALUE) * (_vref_volts / _current_pga_gain_value);
}

long ADS1256::readData()
{
    waitForDRDY();                    // Wait for DRDY to go low before reading
    digitalWrite(_cs_pin, LOW);       // Pull CS low
    _spi.transfer(ADS1256_CMD_RDATA); // Send RDATA command
    delayMicroseconds(7);             // t6 in datasheet (RDATA command delay)

    // Read 3 bytes of data
    long raw_data = 0;
    raw_data = ((long)_spi.transfer(0x00) << 16);
    raw_data |= ((long)_spi.transfer(0x00) << 8);
    raw_data |= _spi.transfer(0x00);

    digitalWrite(_cs_pin, HIGH); // Pull CS high

    // Convert 24-bit two's complement to signed long
    if (raw_data & 0x800000)
    {                           // If MSB is set, it's a negative number
        raw_data |= 0xFF000000; // Sign-extend to 32 bits
    }
    return raw_data;
}

void ADS1256::setChannel(uint8_t positive_input, uint8_t negative_input)
{
    uint8_t mux_value = (positive_input << 4) | negative_input;
    writeRegister(ADS1256_REG_MUX, mux_value);
    delayMicroseconds(50);             // Small delay after MUX change
    _spi.transfer(ADS1256_CMD_SYNC);   // Sync the ADC
    _spi.transfer(ADS1256_CMD_WAKEUP); // Wake up from standby if needed (often auto-synced)
    waitForDRDY();                     // Wait for DRDY to go low after MUX change and sync
}

double ADS1256::readVoltageDifferential(uint8_t positive_input, uint8_t negative_input)
{
    setChannel(positive_input, negative_input); // Set differential channel AIN0-AIN1
    long raw_data = readData();                 // Read single conversion data
    return convertToVolts(raw_data);
}

/*
void ADS1256::setPGA(uint8_t pga_setting)
{
    writeRegister(ADS1256_REG_ADCON, pga_setting | ADS1256_ADCON_BUFFER_ENABLE);
    _current_pga_gain_code = pga_setting;
    updatePGAGainValue(); // Update the actual gain value
    selfCalibrate();      // Recalibrate after changing PGA
}

void ADS1256::setDataRate(uint8_t drate_setting)
{
    writeRegister(ADS1256_REG_DRATE, drate_setting);
    waitForDRDY(); // Wait for DRDY to go low after changing data rate
}

void ADS1256::powerDown()
{
    digitalWrite(_pdwn_pin, LOW); // Pull PWDN low
    // The device goes into power-down mode. SPI communication is disabled.
}

void ADS1256::wakeUp()
{
    digitalWrite(_pdwn_pin, HIGH); // Bring PWDN high
    delay(50);                     // Allow time for device to wake up
    reset();                       // Reset and re-initialize after waking up
}

void ADS1256::startContinuousConversion()
{
    // Continuous mode (RDATAC) requires a SYNC and WAKEUP command before the first RDATAC
    // and then only RDATAC for subsequent reads.
    setChannel(ADS1256_MUX_AIN0, ADS1256_MUX_AIN1); // Ensure channel is set before continuous conversion
    _spi.transfer(ADS1256_CMD_SYNC);                // Sync before RDATAC
    _spi.transfer(ADS1256_CMD_WAKEUP);              // Wake up if in standby
    waitForDRDY();                                  // Wait for first conversion to be ready
    digitalWrite(_cs_pin, LOW);                     // Pull CS low for continuous read
    _spi.transfer(ADS1256_CMD_RDATAC);              // Send RDATAC command
    // DRDY will pulse low for each new conversion data
} 

void ADS1256::stopContinuousConversion()
{
    digitalWrite(_cs_pin, HIGH);       // Pull CS high to stop continuous mode
    _spi.transfer(ADS1256_CMD_SDATAC); // Send SDATAC command
    delay(50);                         // Give it time to stop
}

double ADS1256::readVoltageDifferential_AIN0_AIN1()
{
    setChannel(ADS1256_MUX_AIN0, ADS1256_MUX_AIN1); // Set differential channel AIN0-AIN1
    long raw_data = readData();                     // Read single conversion data
    return convertToVolts(raw_data);
}
 */