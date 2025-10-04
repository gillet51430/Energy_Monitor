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

bool acquisition_active = false;
unsigned long last_measurement_time = 0;
const long measurement_interval = 50; // Intervalle en ms

void handleSerialCommands();
void sendReadySignal();

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

  // Serial.println("Registres avant configuration :");
  // adc.printAllRegisters();

  adc.differentialChannelValue(ADS1256_MUX_AIN1, ADS1256_MUX_AIN0);
  // adc.setBuffer(true);
  // adc.setAutoCalibration(true);
  // adc.setPGA(ADS1256_ADCON_PGA_1);
  // adc.reset();

  // Serial.println("\nRegistres après configuration et calibration :");
  // adc.printAllRegisters();

  // Serial.println("--------------------------------------------------");
  // Serial.println("Configuration terminée. Début des mesures...");
  // Serial.println("--------------------------------------------------");

  sendReadySignal(); // Indique que l'ESP est prêt après le démarrage
}

void loop() {
  handleSerialCommands();

  if (acquisition_active && (millis() - last_measurement_time >= measurement_interval)) {
    last_measurement_time = millis();
    int32_t raw_adc = adc.readRawData();
    float voltage = adc.convertToVoltage(raw_adc);
    Serial.printf("Channel 1: %.4f V | Channel 2: %.4f V | Channel 3: %.4f V | Channel 4: %.4f V\n", voltage, 0.0, 0.0, 0.0);
  }
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    Serial.print("Commande reçue : ");
    Serial.println(command);

    if (command.startsWith("GAIN:")) {
      int gain = command.substring(5).toInt();
      switch(gain) {
        case 1: adc.setPGA(ADS1256_ADCON_PGA_1); break;
        case 2: adc.setPGA(ADS1256_ADCON_PGA_2); break;
        case 4: adc.setPGA(ADS1256_ADCON_PGA_4); break;
        case 8: adc.setPGA(ADS1256_ADCON_PGA_8); break;
        case 16: adc.setPGA(ADS1256_ADCON_PGA_16); break;
        case 32: adc.setPGA(ADS1256_ADCON_PGA_32); break;
        case 64: adc.setPGA(ADS1256_ADCON_PGA_64); break;
      }
      Serial.println("Gain mis à jour.");
      sendReadySignal();
    }
    else if (command.startsWith("BUFFER:")) {
      String state = command.substring(7);
      if (state == "ON") {
        adc.setBuffer(true);
        Serial.println("Buffer activé.");
        sendReadySignal();
      } else if (state == "OFF") {
        adc.setBuffer(false);
        Serial.println("Buffer désactivé.");
        sendReadySignal();
      }
    }
    else if (command == "CAL:SELF") {
        Serial.println("Lancement de l'auto-calibration...");
        adc.selfCalibration(); 
        Serial.println("Calibration terminée.");
        sendReadySignal();
    }
    else if (command == "RESET") {
        Serial.println("Lancement du reset...");
        adc.reset(); 
        Serial.println("Reset terminé.");
        sendReadySignal();
    }
    else if (command == "START") {
      acquisition_active = true;
      Serial.println("Démarrage de l'acquisition.");
    }
    else if (command == "STOP") {
      acquisition_active = false;
      Serial.println("Arrêt de l'acquisition.");
    }
  }
}

void sendReadySignal(){
  delay(100); // Petite pause pour s'assurer que le buffer série est prêt
  Serial.println("ESP32 Ready");
}