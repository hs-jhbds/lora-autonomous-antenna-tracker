
#include <SPI.h>
#include <RadioLib.h>
#include <ESP32Servo.h>
#include <Stepper.h>
#include <math.h>
#include <string.h>

// ==========================================
// LORA RECEIVER PINS
// ==========================================

#define LORA_CS    5
#define LORA_DIO0  26
#define LORA_RST   14

#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23

// ==========================================
// STEPPER MOTOR PINS
// ==========================================

#define STEP_IN1 16
#define STEP_IN2 17
#define STEP_IN3 21
#define STEP_IN4 22

#define STEPS_PER_REV 2048
#define STEPPER_RPM 10

Stepper azimuthMotor(
  STEPS_PER_REV,
  STEP_IN1,
  STEP_IN3,
  STEP_IN2,
  STEP_IN4
);

// ==========================================
// AZIMUTH CONFIGURATION
// ==========================================

#define AZIMUTH_MIN 0
#define AZIMUTH_MAX 180
#define AZIMUTH_INCREMENT 20
#define AZIMUTH_POSITIONS 10

long currentAzimuthSteps = 0;
int currentAzimuthAngle = 0;

// ==========================================
// SERVO CONFIGURATION
// ==========================================

#define SERVO_PIN 27

#define ELEVATION_MIN 45
#define ELEVATION_MAX 135
#define ELEVATION_INCREMENT 15
#define ELEVATION_POSITIONS 7

Servo tiltServo;
int currentElevationAngle = 45;

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
// TRACKING PARAMETERS
// ==========================================

#define SCAN_RSSI_SAMPLES 2
#define MONITOR_RSSI_SAMPLES 3

#define RSSI_DROP_THRESHOLD 5.0
#define DROP_CONFIRMATION_COUNT 3

const unsigned long SETTLE_TIME = 500;

// ==========================================
// SCAN STORAGE
// ==========================================

float azimuthRSSI[AZIMUTH_POSITIONS];
int azimuthAngle[AZIMUTH_POSITIONS];

float elevationRSSI[ELEVATION_POSITIONS];
int elevationAngle[ELEVATION_POSITIONS];

// ==========================================
// TRACKING VARIABLES
// ==========================================

float baselineRSSI = 0;

int consecutiveDrops = 0;

bool baselineEstablished = false;
bool trackingInProgress = false;

const char* dashboardStatus = "STARTING";

// ==========================================
// NON-BLOCKING LORA FLAGS
// ==========================================

volatile bool receivedFlag = false;

void setFlag() {
  receivedFlag = true;
}

unsigned long lastDashboardHeartbeat = 0;

// ==========================================
// DASHBOARD JSON TELEMETRY
// ==========================================

void sendDashboardData(float currentRSSI = NAN) {

  Serial.print("{\"type\":\"telemetry\"");

  Serial.print(",\"rssi\":");

  if (isnan(currentRSSI)) {
    Serial.print("null");
  } else {
    Serial.print(currentRSSI, 1);
  }

  Serial.print(",\"baseline\":");

  if (baselineEstablished) {
    Serial.print(baselineRSSI, 1);
  } else {
    Serial.print("null");
  }

  Serial.print(",\"azimuth\":");
  Serial.print(currentAzimuthAngle);

  Serial.print(",\"elevation\":");
  Serial.print(currentElevationAngle);

  Serial.print(",\"status\":\"");
  Serial.print(dashboardStatus);
  Serial.print("\"");

  Serial.print(",\"threshold\":");
  Serial.print(RSSI_DROP_THRESHOLD, 1);

  Serial.print(",\"dropCount\":");
  Serial.print(consecutiveDrops);

  Serial.print(",\"dropLimit\":");
  Serial.print(DROP_CONFIRMATION_COUNT);

  Serial.print(",\"scanSamples\":");
  Serial.print(SCAN_RSSI_SAMPLES);

  Serial.print(",\"monitorSamples\":");
  Serial.print(MONITOR_RSSI_SAMPLES);

  Serial.print(",\"frequency\":433.92");

  Serial.println("}");
}

// ==========================================
// RECEIVE ONE VALID BEACON
// ==========================================

bool receiveBeaconRSSI(float &rssiValue) {

  if (!receivedFlag) {
    return false;
  }

  receivedFlag = false;

  String receivedData;

  int state = radio.readData(receivedData);

  // Restart LoRa reception immediately.
  int restartState = radio.startReceive();

  if (restartState != RADIOLIB_ERR_NONE) {

    Serial.print("LoRa restart error: ");
    Serial.println(restartState);
  }

  if (state == RADIOLIB_ERR_NONE) {

    if (receivedData.startsWith("TX01,BEACON,")) {

      rssiValue = radio.getRSSI();

      return true;
    }
  }

  else {

    Serial.print("LoRa read error: ");
    Serial.println(state);
  }

  return false;
}

// ==========================================
// COLLECT AVERAGE RSSI
// ==========================================

bool collectAverageRSSI(
  float &averageRSSI,
  int sampleCount
) {

  float totalRSSI = 0;
  int samples = 0;

  Serial.print("Collecting ");
  Serial.print(sampleCount);
  Serial.println(" beacon packets...");

  while (samples < sampleCount) {

    float currentRSSI;

    if (receiveBeaconRSSI(currentRSSI)) {

      totalRSSI += currentRSSI;
      samples++;

      Serial.print("Sample ");
      Serial.print(samples);
      Serial.print(": ");
      Serial.print(currentRSSI);
      Serial.println(" dBm");

      sendDashboardData(currentRSSI);
    }

    // Dashboard heartbeat every second.
    if (millis() - lastDashboardHeartbeat >= 1000) {

      if (!baselineEstablished &&
          !trackingInProgress) {

        dashboardStatus = "WAITING_FOR_BEACON";
      }

      sendDashboardData();

      lastDashboardHeartbeat = millis();
    }

    delay(5);
  }

  averageRSSI = totalRSSI / sampleCount;

  Serial.print("Average RSSI: ");
  Serial.print(averageRSSI);
  Serial.println(" dBm");

  sendDashboardData(averageRSSI);

  return true;
}

// ==========================================
// MOVE STEPPER TO AZIMUTH
// ==========================================

void moveToAzimuth(int targetAngle) {

  targetAngle = constrain(
    targetAngle,
    AZIMUTH_MIN,
    AZIMUTH_MAX
  );

  long targetSteps = lround(
    (double)targetAngle * STEPS_PER_REV / 360.0
  );

  long movementSteps =
    targetSteps - currentAzimuthSteps;

  if (movementSteps != 0) {

    Serial.print("Moving azimuth to ");
    Serial.print(targetAngle);
    Serial.println(" degrees");

    azimuthMotor.step(movementSteps);
  }

  currentAzimuthSteps = targetSteps;
  currentAzimuthAngle = targetAngle;

  sendDashboardData();
}

// ==========================================
// MOVE SERVO TO ELEVATION
// ==========================================

void moveToElevation(int targetAngle) {

  targetAngle = constrain(
    targetAngle,
    ELEVATION_MIN,
    ELEVATION_MAX
  );

  tiltServo.write(targetAngle);

  currentElevationAngle = targetAngle;

  Serial.print("Moving elevation to ");
  Serial.print(targetAngle);
  Serial.println(" degrees");

  sendDashboardData();

  delay(SETTLE_TIME);
}

// ==========================================
// FULL AZIMUTH SCAN
// ==========================================

bool scanAzimuth(int &bestAzimuth, float &bestRSSI) {

  dashboardStatus = "SCANNING_AZIMUTH";
  sendDashboardData();

  Serial.println();
  Serial.println("================================");
  Serial.println("AUTOMATIC AZIMUTH SEARCH");
  Serial.println("================================");

  moveToAzimuth(0);
  delay(SETTLE_TIME);

  int count = 0;

  for (
    int angle = AZIMUTH_MIN;
    angle <= AZIMUTH_MAX;
    angle += AZIMUTH_INCREMENT
  ) {

    Serial.println();
    Serial.println("--------------------------------");

    Serial.print("AZIMUTH: ");
    Serial.print(angle);
    Serial.println(" degrees");

    moveToAzimuth(angle);
    delay(SETTLE_TIME);

    float averageRSSI;

    if (collectAverageRSSI(
          averageRSSI,
          SCAN_RSSI_SAMPLES
        )) {

      if (count < AZIMUTH_POSITIONS) {

        azimuthAngle[count] = angle;
        azimuthRSSI[count] = averageRSSI;

        count++;
      }
    }
  }

  if (count == 0) {

    Serial.println("AZIMUTH SCAN FAILED");
    return false;
  }

  int bestIndex = 0;

  for (int i = 0; i < count; i++) {

    Serial.print("Angle: ");
    Serial.print(azimuthAngle[i]);

    Serial.print(" | RSSI: ");
    Serial.println(azimuthRSSI[i]);

    if (azimuthRSSI[i] > azimuthRSSI[bestIndex]) {
      bestIndex = i;
    }
  }

  bestAzimuth = azimuthAngle[bestIndex];
  bestRSSI = azimuthRSSI[bestIndex];

  Serial.println("BEST AZIMUTH FOUND");
  Serial.println(bestAzimuth);

  return true;
}

// ==========================================
// FULL ELEVATION SCAN
// ==========================================

bool scanElevation(int &bestElevation, float &bestRSSI) {

  dashboardStatus = "SCANNING_ELEVATION";
  sendDashboardData();

  Serial.println();
  Serial.println("================================");
  Serial.println("AUTOMATIC ELEVATION SEARCH");
  Serial.println("================================");

  int count = 0;

  for (
    int angle = ELEVATION_MIN;
    angle <= ELEVATION_MAX;
    angle += ELEVATION_INCREMENT
  ) {

    Serial.println();
    Serial.println("--------------------------------");

    Serial.print("ELEVATION: ");
    Serial.print(angle);
    Serial.println(" degrees");

    moveToElevation(angle);

    float averageRSSI;

    if (collectAverageRSSI(
          averageRSSI,
          SCAN_RSSI_SAMPLES
        )) {

      if (count < ELEVATION_POSITIONS) {

        elevationAngle[count] = angle;
        elevationRSSI[count] = averageRSSI;

        count++;
      }
    }
  }

  if (count == 0) {

    Serial.println("ELEVATION SCAN FAILED");
    return false;
  }

  int bestIndex = 0;

  for (int i = 0; i < count; i++) {

    Serial.print("Angle: ");
    Serial.print(elevationAngle[i]);

    Serial.print(" | RSSI: ");
    Serial.println(elevationRSSI[i]);

    if (elevationRSSI[i] > elevationRSSI[bestIndex]) {
      bestIndex = i;
    }
  }

  bestElevation = elevationAngle[bestIndex];
  bestRSSI = elevationRSSI[bestIndex];

  Serial.println("BEST ELEVATION FOUND");
  Serial.println(bestElevation);

  return true;
}

// ==========================================
// AUTOMATIC TRACKING SEARCH
// ==========================================

void performAutomaticTracking() {

  if (trackingInProgress) {
    return;
  }

  trackingInProgress = true;

  dashboardStatus = baselineEstablished
                    ? "RESCANNING"
                    : "INITIAL_SCAN";

  sendDashboardData();

  Serial.println();
  Serial.println("================================");
  Serial.println("STARTING AUTOMATIC TRACKING");
  Serial.println("================================");

  int bestAzimuth = currentAzimuthAngle;
  int bestElevation = currentElevationAngle;

  float bestAzimuthRSSI = 0;
  float bestElevationRSSI = 0;

  // STEP 1: AZIMUTH SCAN

  bool azimuthOK = scanAzimuth(
    bestAzimuth,
    bestAzimuthRSSI
  );

  if (!azimuthOK) {

    dashboardStatus = "SCAN_ERROR";
    sendDashboardData();

    trackingInProgress = false;
    return;
  }

  // STEP 2: MOVE TO BEST AZIMUTH

  moveToAzimuth(bestAzimuth);
  delay(SETTLE_TIME);

  // STEP 3: ELEVATION SCAN

  bool elevationOK = scanElevation(
    bestElevation,
    bestElevationRSSI
  );

  if (!elevationOK) {

    dashboardStatus = "SCAN_ERROR";
    sendDashboardData();

    trackingInProgress = false;
    return;
  }

  // STEP 4: RETURN TO BEST POSITION

  moveToAzimuth(bestAzimuth);
  moveToElevation(bestElevation);

  // STEP 5: MEASURE FINAL RSSI BASELINE

  Serial.println("Measuring final RSSI baseline...");

  float finalRSSI;

  if (collectAverageRSSI(
        finalRSSI,
        MONITOR_RSSI_SAMPLES
      )) {

    baselineRSSI = finalRSSI;
    baselineEstablished = true;
  }

  consecutiveDrops = 0;

  dashboardStatus = "LOCKED";

  sendDashboardData(
    baselineEstablished ? baselineRSSI : NAN
  );

  Serial.println("SCAN COMPLETED");
  Serial.println("ANTENNA LOCKED");

  trackingInProgress = false;
}

// ==========================================
// INITIAL AUTOMATIC SCAN
// ==========================================

void performInitialScan() {

  Serial.println();
  Serial.println("================================");
  Serial.println("INITIAL AUTOMATIC SCAN");
  Serial.println("================================");

  performAutomaticTracking();
}

// ==========================================
// MONITOR TRANSMITTER SIGNAL
// ==========================================

void monitorSignal() {

  if (!baselineEstablished || trackingInProgress) {
    return;
  }

  float currentRSSI;

  if (!collectAverageRSSI(
        currentRSSI,
        MONITOR_RSSI_SAMPLES
      )) {
    return;
  }

  Serial.println();
  Serial.println("================================");

  Serial.print("Current RSSI: ");
  Serial.println(currentRSSI);

  Serial.print("Locked baseline: ");
  Serial.println(baselineRSSI);

  float triggerLevel =
    baselineRSSI - RSSI_DROP_THRESHOLD;

  if (currentRSSI <= triggerLevel) {

    consecutiveDrops++;

    Serial.print("RSSI DROP CONFIRMATION: ");
    Serial.print(consecutiveDrops);
    Serial.print("/");
    Serial.println(DROP_CONFIRMATION_COUNT);
  }

  else {

    consecutiveDrops = 0;

    Serial.println("Signal stable or stronger.");
    Serial.println("Motors remain stationary.");

    if (currentRSSI > baselineRSSI) {

      baselineRSSI =
        (baselineRSSI * 0.95) +
        (currentRSSI * 0.05);
    }
  }

  if (consecutiveDrops >= DROP_CONFIRMATION_COUNT) {

    consecutiveDrops = 0;

    performAutomaticTracking();
  }

  sendDashboardData(currentRSSI);

  Serial.println("================================");
}

// ==========================================
// SETUP
// ==========================================

void setup() {

  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("================================");
  Serial.println("AUTOMATIC LORA ANTENNA TRACKER");
  Serial.println("================================");

  // INITIALIZE SPI

  SPI.begin(
    LORA_SCK,
    LORA_MISO,
    LORA_MOSI,
    LORA_CS
  );

  // INITIALIZE LORA

  ConfigLoRa_t config = {};

  config.frequency = RADIOLIB_UNIT_MEGA(433.92);
  config.bandwidth = RADIOLIB_UNIT_KILO(125);
  config.spreadingFactor = 9;
  config.codingRate = 7;
  config.syncWord = RADIOLIB_LORA_SYNC_WORD_PRIVATE;
  config.power = 10;
  config.preambleLength = 8;

  Serial.println("Initializing LoRa receiver...");

  int state = radio.begin(config);

  if (state != RADIOLIB_ERR_NONE) {

    Serial.print("LoRa initialization failed: ");
    Serial.println(state);

    dashboardStatus = "LORA_INIT_ERROR";
    sendDashboardData();

    while (true) {
      delay(1000);
    }
  }

  Serial.println("LoRa receiver initialized.");

  // START NON-BLOCKING LORA RECEPTION

  radio.setDio0Action(setFlag, RISING);

  int receiveState = radio.startReceive();

  if (receiveState != RADIOLIB_ERR_NONE) {

    Serial.print("LoRa startReceive failed: ");
    Serial.println(receiveState);

    dashboardStatus = "LORA_RX_ERROR";
    sendDashboardData();

    while (true) {
      delay(1000);
    }
  }

  Serial.println("LoRa continuous receive started.");

  // INITIALIZE STEPPER

  azimuthMotor.setSpeed(STEPPER_RPM);

  currentAzimuthSteps = 0;
  currentAzimuthAngle = 0;

  Serial.println("Stepper initialized.");
  Serial.println("Assumed physical azimuth: 0 degrees");

  // INITIALIZE SERVO

  tiltServo.setPeriodHertz(50);

  tiltServo.attach(
    SERVO_PIN,
    500,
    2500
  );

  tiltServo.write(ELEVATION_MIN);

  currentElevationAngle = ELEVATION_MIN;

  delay(1000);

  Serial.println("Servo initialized at 45 degrees.");

  // INITIAL DASHBOARD MESSAGE

  dashboardStatus = "INITIAL_SCAN";
  sendDashboardData();

  // INITIAL AUTOMATIC SCAN

  performInitialScan();

  Serial.println();
  Serial.println("================================");
  Serial.println("AUTOMATIC TRACKING ACTIVE");
  Serial.println("================================");

  Serial.println("Azimuth range: 0 to 180 degrees");
  Serial.println("Azimuth increment: 20 degrees");

  Serial.println("Elevation range: 45 to 135 degrees");
  Serial.println("Elevation increment: 15 degrees");

  Serial.println("RSSI scan samples: 2");
  Serial.println("RSSI monitor samples: 3");

  Serial.println("RSSI drop threshold: 5 dB");
  Serial.println("Confirmation windows: 3");

  Serial.println("================================");

  sendDashboardData(
    baselineEstablished ? baselineRSSI : NAN
  );
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop() {

  if (!trackingInProgress) {

    if (baselineEstablished) {
      monitorSignal();
    }

    else {

      dashboardStatus = "WAITING_FOR_BEACON";

      if (millis() - lastDashboardHeartbeat >= 1000) {

        sendDashboardData();

        lastDashboardHeartbeat = millis();
      }

      // Process incoming packets while waiting.
      float ignoredRSSI;

      receiveBeaconRSSI(ignoredRSSI);

      delay(5);
    }
  }
}
