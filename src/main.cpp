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

  ADS1256_SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1);

  adc.begin();
  delay(500);
  adc.reset();
  // delay(1000);

  Serial.println("Registres avant configuration :");
  adc.printAllRegisters();

  adc.differentialChannelValue(ADS1256_MUX_AIN0, ADS1256_MUX_AIN1);
  adc.setBuffer(true);
  adc.setAutoCalibration(true);
  adc.setPGA(ADS1256_ADCON_PGA_1);

  adc.reset();

  Serial.println("\nRegistres après configuration et calibration :");
  adc.printAllRegisters();

  Serial.println("--------------------------------------------------");
  Serial.println("Configuration terminée. Début des mesures...");
  Serial.println("--------------------------------------------------");
}

void loop() {
  // Lire la valeur brute de la conversion
  int32_t raw_adc = adc.readRawData();

  // Convertir la valeur brute en tension
  float voltage = adc.convertToVoltage(raw_adc);

  // Afficher les résultats sur le moniteur série
  Serial.printf("Channel 1: %.4f V | Channel 2: %.4f V | Channel 3: %.4f V | Channel 4: %.4f V\n", voltage, 0.0, 0.0, 0.0);


  delay(500); // Pause de 500ms entre les mesures
}
