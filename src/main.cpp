#include <Arduino.h>
#include <SPI.h>
#include "ADS1256_Custom.h"

// --- Configuration des broches ---
#define SPI_SCK 14
#define SPI_MISO 12
#define SPI_MOSI 13
#define CS_PIN 15
#define DRDY_PIN 16
#define PWDN_PIN 4

// --- Paramètres de mesure ---
#define VREF_VOLTS 2.5 // Tension de référence de votre carte ADS1256

//Values for registers
uint8_t registerAddress;      //address of the register, both for reading and writing - selects the register
uint8_t registerValueR;       //this is used to READ a register
uint8_t registerValueW;       //this is used to WRITE a register
int32_t registerData;         //this is used to store the data read from the register (for the AD-conversion)
uint8_t directCommand;        //this is used to store the direct command for sending a command to the ADS1256
String PrintMessage;          //this is used to concatenate stuff into before printing it out.

byte outputBuffer[3]; //3-byte (24-bit) buffer for the fast acquisition - Single-channel, continuous
byte differentialBuffer[12]; //4x3-byte buffer for the fast differential-channel acquisition -
byte singleBuffer[24]; //8x3-byte buffer for the fast single-ended-channel acquisition

// --- Instances ---
SPIClass ADS1256_SPI(VSPI);
ADS1256 adc(CS_PIN, DRDY_PIN, PWDN_PIN, VREF_VOLTS, ADS1256_SPI);

void setup() {
  Serial.begin(115200);
  while (!Serial);  

  adc.begin();
  delay(1000);
  adc.reset();
  delay(1000);

  adc.writeRegister(ADS1256_REG_STATUS, ADS1256_STATUS_BUFFER_ENABLE | ADS1256_STATUS_AUTOCAL_DISABLE);
  adc.writeRegister(ADS1256_REG_MUX, 0x10);

  Serial.println("\n--- Reading Initial Register Values ---");
  adc.printRegister("STATUS", adc.readRegister(ADS1256_REG_STATUS) & 0x0F, true); // Mask reserved bits
  adc.printRegister("MUX", adc.readRegister(ADS1256_REG_MUX), true);
  adc.printRegister("ADCON", adc.readRegister(ADS1256_REG_ADCON), true);
  adc.printRegister("DRATE", adc.readRegister(ADS1256_REG_DRATE), true);

  Serial.println("--------------------------------------------------");
  Serial.println("Configuration terminée. Début des mesures...");
  Serial.println("--------------------------------------------------");
}

void loop() {
  // put your main code here, to run repeatedly:
}
