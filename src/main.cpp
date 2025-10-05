#include <Arduino.h>
#include <SPI.h>
#include <vector>
#include "ADS1256_Custom.h"

// --- Configuration des broches ---
#define SPI_SCK 14
#define SPI_MISO 12
#define SPI_MOSI 13
#define CS_PIN 15
#define DRDY_PIN 16
#define PWDN_PIN 4

// --- Paramètres de mesure ---
#define VREF_VOLTS 2.5

const unsigned int SAMPLING_DURATION_MS = 10000;

void handleSerialCommands();
void sendReadySignal();
void performACMeasurementAndSend();

SPIClass ADS1256_SPI(VSPI);
ADS1256 adc(CS_PIN, DRDY_PIN, PWDN_PIN, VREF_VOLTS, ADS1256_SPI);

void setup() {
  Serial.begin(115200);
  while (!Serial);  

  ADS1256_SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1);

  adc.begin();
  delay(500);
  adc.reset();
  
  // Appliquer les réglages par défaut au démarrage
  adc.setDataRate(ADS1256_DRATE_1000SPS);
  adc.setBuffer(true);
  adc.setPGA(ADS1256_ADCON_PGA_1);

  sendReadySignal(); 
}

void loop() {
  handleSerialCommands();
  delay(10); 
}

void performACMeasurementAndSend() {
    adc.differentialChannelValue(ADS1256_MUX_AIN1, ADS1256_MUX_AIN0);
    delay(50);
    long long offset_sum = 0;
    for(int i = 0; i < 100; i++) {
      offset_sum += adc.readRawData();
    }
    int32_t offset_raw = offset_sum / 100;

    adc.differentialChannelValue(ADS1256_MUX_AIN3, ADS1256_MUX_AIN2);
    delay(50);

    std::vector<int32_t> samples;
    unsigned long start_time = millis();
    while(millis() - start_time < SAMPLING_DURATION_MS) {
        samples.push_back(adc.readRawData());
    }

    Serial.println("START_DATA");
    Serial.print("OFFSET:");
    Serial.println(offset_raw);

    for(size_t i = 0; i < samples.size(); ++i) {
        Serial.print(samples[i]);
        if (i < samples.size() - 1) {
            Serial.print(",");
        }
    }
    Serial.println();
    Serial.println("END_DATA");
}


void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() == 0) return;

    if (command == "GET_SAMPLES") {
        performACMeasurementAndSend();
        sendReadySignal();
    }
    else if (command.startsWith("GAIN:")) {
      int gain = command.substring(5).toInt();
      switch(gain) {
        case 1: adc.setPGA(ADS1256_ADCON_PGA_1); break;
        case 64: adc.setPGA(ADS1256_ADCON_PGA_64); break;
      }
      sendReadySignal();
    }
    // --- MODIFICATION: Gestion de la commande DRATE ---
    else if (command.startsWith("DRATE:")) {
        int sps = command.substring(6).toInt();
        if (sps == 30000) adc.setDataRate(ADS1256_DRATE_30000SPS);
        else if (sps == 15000) adc.setDataRate(ADS1256_DRATE_15000SPS);
        else if (sps == 7500) adc.setDataRate(ADS1256_DRATE_7500SPS);
        else if (sps == 3750) adc.setDataRate(ADS1256_DRATE_3750SPS);
        else if (sps == 2000) adc.setDataRate(ADS1256_DRATE_2000SPS);
        else if (sps == 1000) adc.setDataRate(ADS1256_DRATE_1000SPS);
        else if (sps == 500) adc.setDataRate(ADS1256_DRATE_500SPS);
        else if (sps == 100) adc.setDataRate(ADS1256_DRATE_100SPS);
        else if (sps == 60) adc.setDataRate(ADS1256_DRATE_60SPS);
        else if (sps == 50) adc.setDataRate(ADS1256_DRATE_50SPS);
        else if (sps == 30) adc.setDataRate(ADS1256_DRATE_30SPS);
        else if (sps == 25) adc.setDataRate(ADS1256_DRATE_25SPS);
        else if (sps == 15) adc.setDataRate(ADS1256_DRATE_15SPS);
        else if (sps == 10) adc.setDataRate(ADS1256_DRATE_10SPS);
        else if (sps == 5) adc.setDataRate(ADS1256_DRATE_5SPS);
        else if (sps == 2) adc.setDataRate(ADS1256_DRATE_2_5SPS); // Note: 2.5 devient 2
        sendReadySignal();
    }
    else if (command.startsWith("BUFFER:")) {
      adc.setBuffer(command.substring(7) == "ON");
      sendReadySignal();
    }
    else if (command == "CAL:SELF") {
        adc.selfCalibration(); 
        sendReadySignal();
    }
    else if (command == "RESET") {
        adc.reset(); 
        sendReadySignal();
    }
  }
}

void sendReadySignal(){
  delay(100);
  Serial.println("ESP32 Ready");
}