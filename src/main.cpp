// Version: 2.1.1
#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include "ADS1256_Custom.h"
#include <numeric> // Pour std::accumulate

// --- Configuration des broches ---
#define SPI_SCK 14
#define SPI_MISO 12
#define SPI_MOSI 13
#define CS_PIN 15
#define DRDY_PIN 16
#define PWDN_PIN 4

// --- Paramètres de mesure ---
#define VREF_VOLTS 2.5                             // Tension de référence précise de l'ADC
#define MAINS_VOLTAGE 232.6                        // Tension du réseau électrique (Volts)
#define CURRENT_SENSOR_RATIO 50.0                  // Ratio du SCT013-050 (50A / 1V)
#define VOLTAGE_SENSOR_RATIO MAINS_VOLTAGE         // Ratio du diviseur de tension (ex: 232.6V / 2.5V)

// Seuil en Ampères sous lequel on considère la lecture comme du bruit et on la force à 0.
// À ajuster légèrement au-dessus de la valeur lue à vide (ex: 0.007 A -> seuil = 0.01 A)
#define CURRENT_NOISE_THRESHOLD 0.00

// --- Paramètres d'échantillonnage ---
#define SAMPLES_PER_CYCLE 600                                     // Nombre d'échantillons par cycle de 50Hz (30000 SPS / 50 Hz = 600)
#define NUM_CYCLES_TO_SAMPLE 10                                   // Nombre de cycles à moyenner pour une lecture stable
#define SAMPLE_COUNT (SAMPLES_PER_CYCLE * NUM_CYCLES_TO_SAMPLE)   // Nombre total d'échantillons à lire
#define SAMPLE_COUNT_OFFSET (SAMPLES_PER_CYCLE)                   // Nombre d'échantillons pour le calcul de l'offset (à lire rapidement)

// --- Buffer pour les échantillons ---
int32_t samples[SAMPLE_COUNT];

// --- Variables globales ---
uint32_t sample_times_us = 1000000;   // Temps pris pour lire les échantillons
uint32_t nbsample = 0;        // Nombre d'échantillons lus
float_t puissance_active = 0; // Puissance active calculée 
float test = 0.0;               // Variable de test modifiable via le moniteur série

// --- Initialisation de l'ADC ---
SPIClass ADS1256_SPI(VSPI);
ADS1256 adc(CS_PIN, DRDY_PIN, PWDN_PIN, VREF_VOLTS, ADS1256_SPI);

int32_t calculate_offset() {
    long long sum = 0;
    adc.readMultipleSamples(samples, SAMPLE_COUNT_OFFSET, &sample_times_us); // Lecture rapide de 500 points
    for (int i = 0; i < SAMPLE_COUNT_OFFSET; i++) {
        sum += samples[i];
    }
    return sum / SAMPLE_COUNT_OFFSET;
}

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

void sampling(uint32_t* nbsample, float* puissance_active,uint32_t sample_times_us) {
  unsigned long timer = micros();
  *nbsample = 0;
  *puissance_active = 0.0;
  if (micros() - timer < sample_times_us)
  {
    // 1. Calculer l'offset à chaque mesure pour plus de précision
    adc.differentialChannelValue(ADS1256_MUX_AIN1, ADS1256_MUX_AIN0);
    int32_t offset = calculate_offset();

    // 2. Lire la valeur du courant et de la tension
    adc.differentialChannelValue(ADS1256_MUX_AIN3, ADS1256_MUX_AIN2);
    int32_t value_I = adc.readRawData(); 
    adc.differentialChannelValue(ADS1256_MUX_AIN5, ADS1256_MUX_AIN4);
    int32_t value_V = adc.readRawData();

    // 3. Suppression de l'offset
    value_I -= offset;
    value_V -= 4175001; // Note: This is a hardcoded offset, consider making it dynamic like the current offset.

    // 4. Application du seuil de bruit
    if (fabs(value_I) < CURRENT_NOISE_THRESHOLD + test) {
        value_I = 0.0;
    }
    if (fabs(value_V) < CURRENT_NOISE_THRESHOLD + test) {
        value_V = 0.0;
    }

    // 5. Conversion en Volts
    float current_I = adc.convertion(value_I);
    float voltage_V = adc.convertion(value_V);

    // 6. Calibration en Courant (A) et Tension (V)
    float current = current_I * CURRENT_SENSOR_RATIO; 
    float voltage = voltage_V * VOLTAGE_SENSOR_RATIO;

    // 7. Calcul de la puissance instantanée
    float instantaneous_power = voltage * current;

    // 8. Accumulation pour le calcul de la puissance active
    *puissance_active += instantaneous_power;
    
    *nbsample ++;
    Serial.printf("Offset: %d | value_I: %d | value_V: %d | current_I: %.4f A | voltage_V: %.4f V | current: %.4f A | voltage: %.2f V | Puissance Instantanee: %.2f W\n", offset, value_I, value_V, current_I, voltage_V, current, voltage, instantaneous_power);
  }  
}

void setupWiFi(const char* ssid, const char* password, int timeout_ms = 20000) {
  
  Serial.println(); // Saut de ligne pour la clarté
  Serial.print("Connexion au réseau : ");
  Serial.println(ssid);

  // Définit l'ESP32 en mode Station (client WiFi)
  WiFi.mode(WIFI_STA);
  // Lance la connexion
  WiFi.begin(ssid, password);

  unsigned long startTime = millis(); // Temps de départ
  Serial.print("Tentative de connexion");

  // Boucle tant que la connexion n'est pas établie ET que le timeout n'est pas dépassé
  while (WiFi.status() != WL_CONNECTED) {
    
    // Vérification du timeout
    if (millis() - startTime > timeout_ms) {
      Serial.println("\nÉchec de la connexion WiFi ! (Timeout)");
      // Optionnel : Vous pourriez redémarrer l'ESP ici si la connexion est critique
      // ESP.restart(); 
      return; // Quitte la fonction en échec
    }
    
    delay(500);
    Serial.print(".");
  }

  // Si on sort de la boucle, c'est que la connexion est établie
  Serial.println("\nConnexion WiFi établie !");
  Serial.print("Adresse IP assignée : ");
  Serial.println(WiFi.localIP());
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // String Wifi_ssid[] = {"FreeboxWifi_2G_IOT","*IOTWifiNetworkAtHome1"};
    // setupWiFi(Wifi_ssid[0].c_str(), Wifi_ssid[1].c_str());

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
    
    // Calibration de l'offset
    Serial.println("Calibration de l'offset en cours... ");
    // Assurez-vous qu'aucun courant ne traverse le capteur à ce moment !
    int32_t offset_raw = calculate_offset();

    // Conversion de la valeur brute de l'offset en Volts
    float offset_volts = adc.convertion(offset_raw);
    Serial.printf("Offset: %d | Valeur en tension: %.4f V\n", offset_raw, offset_volts);
}

void loop() {
  sampling(&nbsample, &puissance_active, sample_times_us);
  // float result =  puissance_active / nbsample;
  // Serial.printf("Puissance Active: %d | Nombre d'échantillons: %d | Puissance Active en Ws: %.2f\n",puissance_active, nbsample, result);

  /*
      // 1. Calculer l'offset à chaque mesure pour plus de précision
      adc.differentialChannelValue(ADS1256_MUX_AIN1, ADS1256_MUX_AIN0);
      int32_t offset = calculate_offset();

      // 2. Acquérir les échantillons du signal de courant
      adc.differentialChannelValue(ADS1256_MUX_AIN3, ADS1256_MUX_AIN2);
      adc.readMultipleSamples(samples, SAMPLE_COUNT, &sample_times_us);

      // 3. Calculer la somme des carrés des échantillons (en retirant l'offset)
      double sum_of_squares = 0;
      for (int i = 0; i < SAMPLE_COUNT; i++) {
          // Soustraire l'offset pour centrer le signal sur zéro
          double sample_zeroed = samples[i] - offset;
          // Ajouter le carré à la somme
          sum_of_squares += sample_zeroed * sample_zeroed;
      }

      // 4. Calculer la valeur RMS (efficace) en unités ADC
      double rms_adc_raw = sqrt(sum_of_squares / SAMPLE_COUNT);

      // 5. Convertir la valeur RMS de l'ADC en Volts
      // Formule: Voltage = (Valeur_ADC / Résolution_Max) * (Plage_de_Tension / Gain)
      // Plage de tension = VREF * 2 (car bipolaire de -VREF à +VREF)
      double rms_voltage = adc.convertToVoltage(rms_adc_raw);

      // 6. Convertir la tension RMS en Courant RMS
      double rms_current = rms_voltage * CURRENT_SENSOR_RATIO;

      // 7. Appliquer le seuil de bruit
      if (rms_current < CURRENT_NOISE_THRESHOLD + test) {
          rms_current = 0.0;
      }

      // 8. Calculer la Puissance Apparente (en Volt-Ampères)
      double apparent_power = MAINS_VOLTAGE * rms_current;

      // 9. Afficher les résultats
      Serial.printf("CURRENT_NOISE_THRESHOLD: %.3f | Courant: %.3f A | Puissance Apparente: %.1f VA\n", CURRENT_NOISE_THRESHOLD + test, rms_current, apparent_power);
  */
  handleSerialCommands();
   delay(1000); // Attendre 2 secondes avant la prochaine mesure
   
}