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
// --- MODIFICATION: Renommage et nouvelle fonction ---
void performACMeasurementStandard();
void performACMeasurementAdvanced();
int32_t measureOffset(); // Fonction utilitaire

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

int32_t measureOffset() {
    adc.differentialChannelValue(ADS1256_MUX_AIN1, ADS1256_MUX_AIN0);
    long long offset_sum = 0;
    for(int i = 0; i < 100; i++) {
      offset_sum += adc.readRawData();
    }
    return offset_sum / 100;
}

void performACMeasurementStandard() {
    // 1. Mesure de l'offset
    int32_t offset_raw = measureOffset();

    // 2. Mesure du signal avec soustraction de l'offset
    adc.differentialChannelValue(ADS1256_MUX_AIN3, ADS1256_MUX_AIN2);
    std::vector<int32_t> samples;
    unsigned long start_time = millis();
    while(millis() - start_time < SAMPLING_DURATION_MS) {
        samples.push_back(adc.readRawData());
    }

    Serial.println("START_DATA");
    for(size_t i = 0; i < samples.size(); ++i) {
        Serial.print(samples[i]);
        if (i < samples.size() - 1) {
            Serial.print(",");
        }
    }
    Serial.println();
    Serial.println("END_DATA");
}

void performACMeasurementAdvanced() {
    // 2. Mesure du signal avec soustraction de l'offset
    std::vector<int32_t> samples;
    unsigned long start_time = millis();
    while(millis() - start_time < SAMPLING_DURATION_MS) {
        adc.differentialChannelValue(ADS1256_MUX_AIN1, ADS1256_MUX_AIN0);
        int32_t offset1_raw = adc.readRawData(); 
        adc.differentialChannelValue(ADS1256_MUX_AIN3, ADS1256_MUX_AIN2);
        int32_t Data_raw = adc.readRawData();
        adc.differentialChannelValue(ADS1256_MUX_AIN1, ADS1256_MUX_AIN0);
        int32_t offset2_raw = adc.readRawData(); 
        samples.push_back(Data_raw - (offset1_raw + offset2_raw) / 2);
    }

    // 2. Envoi des données
    Serial.println("START_DATA");
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

    // --- MODIFICATION: Gestion des nouvelles commandes de mesure ---
    if (command == "GET_SAMPLES_STD") {
        performACMeasurementStandard();
        sendReadySignal();
    }
    else if (command == "GET_SAMPLES_ADV") {
        performACMeasurementAdvanced();
        sendReadySignal();
    }
    else if (command.startsWith("GAIN:")) {
      int gain = command.substring(5).toInt();
      switch(gain) {
        case 1: adc.setPGA(ADS1256_ADCON_PGA_1); break;
        // --- NOTE: Ajoutez les autres gains si vous les utilisez dans le futur ---
        case 2: adc.setPGA(ADS1256_ADCON_PGA_2); break;
        case 4: adc.setPGA(ADS1256_ADCON_PGA_4); break;
        case 8: adc.setPGA(ADS1256_ADCON_PGA_8); break;
        case 16: adc.setPGA(ADS1256_ADCON_PGA_16); break;
        case 32: adc.setPGA(ADS1256_ADCON_PGA_32); break;
        case 64: adc.setPGA(ADS1256_ADCON_PGA_64); break;
      }
      sendReadySignal();
    }
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
        else if (sps == 2) adc.setDataRate(ADS1256_DRATE_2_5SPS);
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