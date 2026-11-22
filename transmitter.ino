
#include <SPI.h>
#include <RadioLib.h>

// ==========================================
// TRANSMITTER PINS
// ==========================================

#define LORA_CS    5
#define LORA_DIO0  26
#define LORA_RST   14

#define LORA_SCK   32
#define LORA_MISO  33
#define LORA_MOSI  25

// ==========================================
// LORA MODULE
// ==========================================

SX1278 radio = new Module(
  LORA_CS,
  LORA_DIO0,
  LORA_RST,
  -1,
  SPI
);

// ==========================================
// ORIGINAL BEACON CONFIGURATION
// ==========================================

unsigned long packetCounter = 0;

const unsigned long BEACON_INTERVAL = 500;

unsigned long previousTransmit = 0;

// ==========================================
// SETUP
// ==========================================

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("================================");
  Serial.println("433 MHz LORA TRANSMITTER");
  Serial.println("================================");

  SPI.begin(
    LORA_SCK,
    LORA_MISO,
    LORA_MOSI,
    LORA_CS
  );

  ConfigLoRa_t config = {};

  config.frequency = RADIOLIB_UNIT_MEGA(433.92);
  config.bandwidth = RADIOLIB_UNIT_KILO(125);
  config.spreadingFactor = 9;
  config.codingRate = 7;
  config.syncWord = RADIOLIB_LORA_SYNC_WORD_PRIVATE;
  config.power = 10;
  config.preambleLength = 8;

  Serial.println("Initializing LoRa...");

  int state = radio.begin(config);

  if (state == RADIOLIB_ERR_NONE) {

    Serial.println("TRANSMITTER READY");
    Serial.println("Frequency: 433.92 MHz");
    Serial.println("Bandwidth: 125 kHz");
    Serial.println("Spreading Factor: 9");
    Serial.println("Coding Rate: 4/7");
    Serial.println("TX Power: 10 dBm");
    Serial.println("Beacon interval: 500 ms");

  } else {

    Serial.print("Initialization FAILED: ");
    Serial.println(state);

    while (true) {
      delay(1000);
    }
  }

  previousTransmit = millis();

  Serial.println("BEACON ACTIVE");
  Serial.println("================================");
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop() {

  unsigned long currentTime = millis();

  if (currentTime - previousTransmit >= BEACON_INTERVAL) {

    previousTransmit = currentTime;

    packetCounter++;

    String message =
      "TX01,BEACON," +
      String(packetCounter) +
      ",ACTIVE";

    int state = radio.transmit(message);

    if (state == RADIOLIB_ERR_NONE) {

      Serial.print("TX #");
      Serial.print(packetCounter);
      Serial.print(" | ");
      Serial.println(message);

    } else {

      Serial.print("TX ERROR: ");
      Serial.println(state);
    }
  }
}
