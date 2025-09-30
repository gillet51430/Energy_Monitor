#ifndef ADS1256_Custom_H
#define ADS1256_Custom_H

#include <Arduino.h>
#include <SPI.h>

// ADS1256 Register Map
#define ADS1256_REG_STATUS 0x00 // Status Register
#define ADS1256_REG_MUX 0x01    // MUX Register
#define ADS1256_REG_ADCON 0x02  // A/D Control Register (PGA, Buffer)
#define ADS1256_REG_DRATE 0x03  // Data Rate Register
#define ADS1256_REG_IO 0x04     // I/O Register
#define ADS1256_REG_OFC0 0x05   // Offset Calibration Register (LSB)
#define ADS1256_REG_OFC1 0x06   // Offset Calibration Register
#define ADS1256_REG_OFC2 0x07   // Offset Calibration Register (MSB)
#define ADS1256_REG_FSC0 0x08   // Full-Scale Calibration Register (LSB)
#define ADS1256_REG_FSC1 0x09   // Full-Scale Calibration Register
#define ADS1256_REG_FSC2 0x0A   // Full-Scale Calibration Register (MSB)

// ADS1256 Command Definitions
#define ADS1256_CMD_WAKEUP 0x00   // Completes SYNC and exits standby mode
#define ADS1256_CMD_RDATA 0x01    // Read Data
#define ADS1256_CMD_RDATAC 0x03   // Read Data Continuously
#define ADS1256_CMD_SDATAC 0x0F   // Stop Read Data Continuously
#define ADS1256_CMD_RREG 0x10     // Read Register
#define ADS1256_CMD_WREG 0x50     // Write Register
#define ADS1256_CMD_SELFCAL 0xF0  // Self Calibration
#define ADS1256_CMD_SELFOCAL 0xF1 // Self Offset Calibration
#define ADS1256_CMD_SELFGCAL 0xF2 // Self Gain Calibration
#define ADS1256_CMD_SYSOCAL 0xF3  // System Offset Calibration
#define ADS1256_CMD_SYSGCAL 0xF4  // System Gain Calibration
#define ADS1256_CMD_SYNC 0xFC     // Synchronize the A/D converter
#define ADS1256_CMD_STANDBY 0xFD  // Begin Standby Mode
#define ADS1256_CMD_RESET 0xFE    // Reset to Power-Up Values

// MUX Register
#define ADS1256_MUX_AINCOM 0x01 // AINCOM (common input)
#define ADS1256_MUX_AIN0 0x00
#define ADS1256_MUX_AIN1 0x01
#define ADS1256_MUX_AIN2 0x02
#define ADS1256_MUX_AIN3 0x03
#define ADS1256_MUX_AIN4 0x04
#define ADS1256_MUX_AIN5 0x05
#define ADS1256_MUX_AIN6 0x06
#define ADS1256_MUX_AIN7 0x07

// ADCON Register (PGA)
#define ADS1256_ADCON_PGA_1 0x00  // PGA Gain 1
#define ADS1256_ADCON_PGA_2 0x01  // PGA Gain 2
#define ADS1256_ADCON_PGA_4 0x02  // PGA Gain 4
#define ADS1256_ADCON_PGA_8 0x03  // PGA Gain 8
#define ADS1256_ADCON_PGA_16 0x04 // PGA Gain 16
#define ADS1256_ADCON_PGA_32 0x05 // PGA Gain 32
#define ADS1256_ADCON_PGA_64 0x06 // PGA Gain 64

// STATUS Buffer Enable Bit (Common interpretation for LCtech)
#define ADS1256_STATUS_BUFFER_ENABLE 0x02      // This is STATUS bit 2
#define ADS1256_STATUS_AUTOCAL_DISABLE 0x00    // This is STATUS bit 3
#define ADS1256_STATUS_AUTOCAL_ENABLE 0x04     // This is STATUS bit 3

// DRATE Register (Data Rate)
#define ADS1256_DRATE_30000SPS 0xF0 // 30000 SPS
#define ADS1256_DRATE_15000SPS 0xE0 // 15000 SPS
#define ADS1256_DRATE_7500SPS 0xD0  // 7500 SPS
#define ADS1256_DRATE_3750SPS 0xC0  // 3750 SPS
#define ADS1256_DRATE_2000SPS 0xB0  // 2000 SPS
#define ADS1256_DRATE_1000SPS 0xA1  // 1000 SPS
#define ADS1256_DRATE_500SPS 0x92   // 500 SPS
#define ADS1256_DRATE_100SPS 0x83   // 100 SPS
#define ADS1256_DRATE_60SPS 0x73    // 60 SPS
#define ADS1256_DRATE_50SPS 0x63    // 50 SPS
#define ADS1256_DRATE_30SPS 0x53    // 30 SPS
#define ADS1256_DRATE_25SPS 0x43    // 25 SPS
#define ADS1256_DRATE_15SPS 0x33    // 15 SPS
#define ADS1256_DRATE_10SPS 0x23    // 10 SPS
#define ADS1256_DRATE_5SPS 0x13     // 5 SPS
#define ADS1256_DRATE_2_5SPS 0x03   // 2.5 SPS


class ADS1256 {
public:
    /**
     * @brief Constructeur de la classe ADS1256.
     * @param cs_pin Pin de Chip Select (CS).
     * @param drdy_pin Pin de Data Ready (DRDY), doit être compatible avec les interruptions.
     * @param pdwn_pin Pin de Power Down (PDWN) ou RESET.
     * @param vref_volts Tension de référence en Volts.
     * @param spi Instance de l'objet SPI à utiliser (ex: SPI, VSPI).
     */
    ADS1256(uint8_t cs_pin, uint8_t drdy_pin, uint8_t pdwn_pin, double vref_volts, SPIClass& spi);

    void begin();
    void reset();
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    String binaryToString(uint8_t val);
    void printRegister(const char *regName, uint8_t regValue, bool lrlf);
    void printAllRegisters();

    int32_t readRawData();
    float convertToVoltage(int32_t rawData);
    void setPGA(uint8_t pga_gain_code);
    void setBuffer(bool enable);
    void setAutoCalibration(bool enable);

    void differentialChannelValue(uint8_t channelN, uint8_t channelP);

private:
    // --- Broches et configuration ---
    uint8_t _cs_pin;
    uint8_t _drdy_pin;
    uint8_t _pdwn_pin;
    double _vref_volts;
    SPIClass& _spi;

    uint8_t _current_pga_gain_code;     // Stores the current PGA setting code (e.g., ADS1256_ADCON_PGA_1)
    double _current_pga_gain_value;     // Stores the actual gain value (e.g., 1.0, 2.0, 4.0)

    /**
     * @brief Indicateur de donnée prête.
     * 'volatile' est CRUCIAL pour que le compilateur ne fasse pas d'optimisations
     * incorrectes, car cette variable est modifiée dans une interruption (ISR).
     */
    volatile bool _dataReady;

    /**
     * @brief La fonction d'interruption (ISR).
     * Doit être 'static' pour être utilisée avec attachInterruptArg.
     * IRAM_ATTR est une optimisation pour ESP32.
     * @param arg Pointeur vers l'instance de la classe ('this').
     */
    static void IRAM_ATTR checkDReady_ISR(void* arg);
    
    void waitForDRDY();
    void updatePGAGainValue();
};

#endif // ADS1256_CUSTOM_H