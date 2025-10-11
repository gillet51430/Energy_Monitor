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
#define OFFSET_SAMPLE_SIZE 100

const unsigned int SAMPLING_DURATION_MS = 10000;

// =================== MODIFICATION START ===================
// AJOUT: Variable globale pour suivre la vitesse d'échantillonnage (SPS)
// Initialisée à 1000, qui est la valeur par défaut dans setup().
unsigned int currentSps = 1000;
// =================== MODIFICATION END ===================

void handleSerialCommands();
void sendReadySignal();
void performACMeasurement();
void performACMeasurement_Binary();
int32_t measureOffset();

SPIClass ADS1256_SPI(VSPI);
ADS1256 adc(CS_PIN, DRDY_PIN, PWDN_PIN, VREF_VOLTS, ADS1256_SPI);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  ADS1256_SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1);

  adc.begin();
  delay(500);
  adc.reset();

  adc.setDataRate(ADS1256_DRATE_1000SPS);
  adc.setBuffer(true);
  adc.setPGA(ADS1256_ADCON_PGA_1);
  adc.differentialChannelValue(ADS1256_MUX_AIN1, ADS1256_MUX_AIN0);
  int32_t test = adc.readRawData();

  sendReadySignal();
}

void loop() {
  handleSerialCommands();
  delay(10);
}

int32_t measureOffset() {
  adc.differentialChannelValue(ADS1256_MUX_AIN1, ADS1256_MUX_AIN0);
  long long offset_sum = 0;
  for (int i = 0; i < OFFSET_SAMPLE_SIZE; i++) {
    offset_sum += adc.readRawData();
  }
  return offset_sum / OFFSET_SAMPLE_SIZE;
}

void performACMeasurement() {
    // 1. Calculer et envoyer la durée estimée pour la mesure de l'offset.
    //    Durée (ms) = (Nombre d'échantillons / SPS) * 1000
    unsigned int offset_duration_ms = (OFFSET_SAMPLE_SIZE * 1000) / currentSps;
    Serial.printf("TASK:OFFSET\nDURATION:%u\n", offset_duration_ms);
    Serial.flush(); // S'assurer que le message part immédiatement

    // 2. Effectuer la mesure de l'offset.
    int32_t offset_raw = measureOffset();

    // 3. Envoyer la durée pour l'échantillonnage principal.
    Serial.printf("TASK:SAMPLING\nDURATION:%u\n", SAMPLING_DURATION_MS);
    Serial.flush();

    // 4. Effectuer l'échantillonnage principal.
    adc.differentialChannelValue(ADS1256_MUX_AIN3, ADS1256_MUX_AIN2);
    std::vector<int32_t> samples;
    unsigned long start_time = millis();
    while (millis() - start_time < SAMPLING_DURATION_MS) {
        samples.push_back(adc.readRawData());
    }

    // 5. Envoyer les données (inchangé).
    Serial.println("START_DATA");
    Serial.print("OFFSET:");
    Serial.println(offset_raw);

    for (size_t i = 0; i < samples.size(); ++i) {
        Serial.print(samples[i]);
        if (i < samples.size() - 1) {
            Serial.print(",");
        }
    }
    Serial.println();
    Serial.println("END_DATA");
    Serial.printf("COUNT:%d\n", samples.size());
}

void performACMeasurement_Binary() {
    unsigned int offset_duration_ms = (OFFSET_SAMPLE_SIZE * 1000) / currentSps;
    Serial.printf("TASK:OFFSET\nDURATION:%u\n", offset_duration_ms);
    Serial.flush();

    int32_t offset_raw = measureOffset();

    Serial.printf("TASK:SAMPLING\nDURATION:%u\n", SAMPLING_DURATION_MS);
    Serial.flush();

    adc.differentialChannelValue(ADS1256_MUX_AIN3, ADS1256_MUX_AIN2);
    std::vector<int32_t> samples;
    unsigned long start_time = millis();
    while (millis() - start_time < SAMPLING_DURATION_MS) {
        samples.push_back(adc.readRawData());
    }

    Serial.println("START_BINARY");
    Serial.printf("OFFSET:%d\n", offset_raw);
    Serial.printf("COUNT:%d\n", samples.size());
    Serial.write((uint8_t*)samples.data(), samples.size() * sizeof(int32_t));
    Serial.flush();
    Serial.println("END_BINARY");
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() == 0) return;

    if (command == "START") {
        performACMeasurement();
        sendReadySignal();
    } else if (command == "START_BINARY") {
        performACMeasurement_Binary();
        sendReadySignal();
    } else if (command.startsWith("GAIN:")) {
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
      sendReadySignal();
    } else if (command.startsWith("DRATE:")) {
        int sps = command.substring(6).toInt();
        if (sps == 30000) { adc.setDataRate(ADS1256_DRATE_30000SPS); currentSps = 30000; }
        else if (sps == 15000) { adc.setDataRate(ADS1256_DRATE_15000SPS); currentSps = 15000; }
        else if (sps == 7500) { adc.setDataRate(ADS1256_DRATE_7500SPS); currentSps = 7500; }
        else if (sps == 3750) { adc.setDataRate(ADS1256_DRATE_3750SPS); currentSps = 3750; }
        else if (sps == 2000) { adc.setDataRate(ADS1256_DRATE_2000SPS); currentSps = 2000; }
        else if (sps == 1000) { adc.setDataRate(ADS1256_DRATE_1000SPS); currentSps = 1000; }
        else if (sps == 500) { adc.setDataRate(ADS1256_DRATE_500SPS); currentSps = 500; }
        else if (sps == 100) { adc.setDataRate(ADS1256_DRATE_100SPS); currentSps = 100; }
        else if (sps == 60) { adc.setDataRate(ADS1256_DRATE_60SPS); currentSps = 60; }
        else if (sps == 50) { adc.setDataRate(ADS1256_DRATE_50SPS); currentSps = 50; }
        else if (sps == 30) { adc.setDataRate(ADS1256_DRATE_30SPS); currentSps = 30; }
        else if (sps == 25) { adc.setDataRate(ADS1256_DRATE_25SPS); currentSps = 25; }
        else if (sps == 15) { adc.setDataRate(ADS1256_DRATE_15SPS); currentSps = 15; }
        else if (sps == 10) { adc.setDataRate(ADS1256_DRATE_10SPS); currentSps = 10; }
        else if (sps == 5) { adc.setDataRate(ADS1256_DRATE_5SPS); currentSps = 5; }
        else if (sps == 2) { adc.setDataRate(ADS1256_DRATE_2_5SPS); currentSps = 2; } // Le cas 2.5 SPS est envoyé comme '2' par l'UI
        sendReadySignal();
    } else if (command.startsWith("BUFFER:")) {
      adc.setBuffer(command.substring(7) == "ON");
      sendReadySignal();
    } else if (command == "CAL:SELF") {
        adc.selfCalibration();
        sendReadySignal();
    } else if (command == "RESET") {
        adc.reset();
        sendReadySignal();
    }
  }
}

void sendReadySignal(){
  delay(100);
  Serial.println("ESP32 Ready");
}