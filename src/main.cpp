// Version: 2.1.1
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

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() == 0) {
      return;
    } else {
      test = command.toFloat();
    }
  }
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

        Serial.println("\n--- Moniteur d'Energie ESP32 ---");

    // Initialisation de l'ADC
    ADS1256_SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1);
    adc.begin();
    delay(500);
    adc.reset();

    // Configuration des paramètres de l'ADC
    adc.setDataRate(ADS1256_DRATE_30000SPS);
    adc.setBuffer(true);
    adc.setPGA(ADS1256_ADCON_PGA_1);
    
    // Configurer le canal différentiel pour la lecture du capteur de courant
    adc.differentialChannelValue(ADS1256_MUX_AIN1, ADS1256_MUX_AIN0);

    Serial.println("Initialisation terminee.");
}

void loop() {
  
  handleSerialCommands();
   delay(1000); // Attendre 2 secondes avant la prochaine mesure
   
}