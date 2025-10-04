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
#define OFFSET_VOLTAGE 0.004

bool acquisition_active = false;
unsigned long last_measurement_time = 0;
const long measurement_interval = 50; // Intervalle en ms

void handleSerialCommands();
void sendStatus(String message);

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
  adc.setBuffer(true);
  adc.setAutoCalibration(true);
  adc.setPGA(ADS1256_ADCON_PGA_1);
  adc.reset();

  // Serial.println("\nRegistres après configuration et calibration :");
  // adc.printAllRegisters();

  // Serial.println("--------------------------------------------------");
  // Serial.println("Configuration terminée. Début des mesures...");
  // Serial.println("--------------------------------------------------");

  sendStatus("ESP ready");
}

void loop() {
  handleSerialCommands();

  if (acquisition_active && (millis() - last_measurement_time >= measurement_interval)) {
    last_measurement_time = millis();

    // Lecture des 4 canaux différentiels
    // Canal 1 (AIN0-AIN1)
    adc.differentialChannelValue(ADS1256_MUX_AIN1, ADS1256_MUX_AIN0);
    delay(1); // Petit délai pour le changement de MUX
    float voltage1 = adc.convertToVoltage(adc.readRawData());

    // Canal 2 (AIN2-AIN3)
    adc.differentialChannelValue(ADS1256_MUX_AIN3, ADS1256_MUX_AIN2);
    delay(1);
    float voltage2 = adc.convertToVoltage(adc.readRawData());
    voltage2 -= voltage1 + OFFSET_VOLTAGE;

    // // Canal 3 (AIN4-AIN5)
    // adc.differentialChannelValue(ADS1256_MUX_AIN5, ADS1256_MUX_AIN4);
    // delay(1);
    // float voltage3 = adc.convertToVoltage(adc.readRawData());
    
    // // Canal 4 (AIN6-AIN7)
    // adc.differentialChannelValue(ADS1256_MUX_AIN7, ADS1256_MUX_AIN6);
    // delay(1);
    // float voltage4 = adc.convertToVoltage(adc.readRawData());

    float voltage3 = 0.0;
    float voltage4 = 0.0;

    Serial.printf("Channel 1: %.4f V | Channel 2: %.4f V | Channel 3: %.4f V | Channel 4: %.4f V\n", voltage1, voltage2, voltage3, voltage4);
  }
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    Serial.print("Received command: ");
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
      sendStatus("Gain updated.");
    }
    else if (command.startsWith("BUFFER:")) {
      String state = command.substring(7);
      if (state == "ON") {
        adc.setBuffer(true);
        sendStatus("Buffer enabled.");
      } else if (state == "OFF") {
        adc.setBuffer(false);
        sendStatus("Buffer disabled.");
      }
    }
    else if (command.startsWith("DRATE:")) {
        int rate = command.substring(6).toInt();
        uint8_t rate_code;
        // Correspondance entre la valeur en SPS et le code du registre
        if (rate == 30000) rate_code = ADS1256_DRATE_30000SPS;
        else if (rate >= 15000) rate_code = ADS1256_DRATE_15000SPS;
        else if (rate >= 7500) rate_code = ADS1256_DRATE_7500SPS;
        else if (rate >= 3750) rate_code = ADS1256_DRATE_3750SPS;
        else if (rate >= 2000) rate_code = ADS1256_DRATE_2000SPS;
        else if (rate >= 1000) rate_code = ADS1256_DRATE_1000SPS;
        else if (rate >= 500) rate_code = ADS1256_DRATE_500SPS;
        // ... ajoutez les autres valeurs ici
        else { rate_code = ADS1256_DRATE_30000SPS; } // Default
        
        adc.setDataRate(rate_code);
        sendStatus("Data Rate updated.");
    }
    else if (command == "CAL:SELF") {
        sendStatus("Starting self-calibration...");
        adc.selfCalibration(); 
        sendStatus("Calibration finished.");
    }
    else if (command == "RESET") {
        sendStatus("Resetting ADC...");
        adc.reset(); 
        sendStatus("Reset finished.");
    }
    else if (command == "START") {
      acquisition_active = true;
      sendStatus("Acquisition started.");
    }
    else if (command == "STOP") {
      acquisition_active = false;
      sendStatus("Acquisition stopped.");
    }
  }
}

void sendStatus(String message){
  Serial.print("Status:");
  Serial.println(message);
}