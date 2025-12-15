#include <Arduino.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

// ==== Config ====
#define BLE_LOCAL_NAME "MagicWand_BLE"
#define UUID(val) "7843" val "-91f7-46a1-8795-173154692a6a"
#define BASE    "b000"
#define NOTIFY  "b001"

#define GESTURE_BUTTON 10
#define LEDR 22
#define LEDG 23
#define LEDB 24

// IMU / processing
#define IMU_SAMPLE_CHANNEL 6
#define IMU_SAMPLE_COUNT   30

const float BASELINE_ALPHAS[IMU_SAMPLE_CHANNEL] = {0.93, 0.93, 0.93, 0.98, 0.98, 0.98};
const float MULT_ARRAY[IMU_SAMPLE_CHANNEL]      = {15.0, 18.0, 18.0, 0.1, 0.15, 0.15};

// Pre/Mid/Post window
const int PRE_SAMPLES  = 20;
const int MID_SAMPLES  = 60;
const int POST_SAMPLES = 20;
const int TOTAL_SAMPLES = PRE_SAMPLES + MID_SAMPLES + POST_SAMPLES; // 100

// BLE notify pacing
const int NOTIFY_CHUNK = 20; // bytes per packet (default MTU 23 → payload ~20)
const int NOTIFY_DELAY_MS = 5;

// ==== Globals ====
BLEService dataService(UUID(BASE));
BLECharacteristic sensorChar(UUID(NOTIFY), BLERead | BLENotify, NOTIFY_CHUNK);

float imu_frame[IMU_SAMPLE_CHANNEL] = {0};

int8_t ring_buffer[IMU_SAMPLE_COUNT][IMU_SAMPLE_CHANNEL];
int ring_write_idx = 0;
bool ring_full = false;

float baselinesA[IMU_SAMPLE_CHANNEL] = {0};
float baselinesB[IMU_SAMPLE_CHANNEL] = {0};
uint8_t baselineToggle = 0;

uint8_t batch[TOTAL_SAMPLES][IMU_SAMPLE_CHANNEL];
int batch_count = 0;
bool capturing = false;

int prevBtn = HIGH;

// Optional: connection state
bool connectedOnce = false;
unsigned long connectedMillis = 0;

// ==== Helpers ====
static inline void clamp_int8(float v, int8_t &out) {
  if (v > 127.0f) v = 127.0f;
  if (v < -128.0f) v = -128.0f;
  out = (int8_t)lrintf(v);
}

// ==== Setup ====
void setup() {
  pinMode(GESTURE_BUTTON, INPUT_PULLUP);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  //while (!Serial) { delay(50); }

  if (!IMU.begin()) {
    Serial.println("IMU begin failed");
    digitalWrite(LEDR, LOW);
    return;
  }

  if (!BLE.begin()) {
    Serial.println("BLE begin failed");
    digitalWrite(LEDR, LOW);
    return;
  }

  BLE.setLocalName(BLE_LOCAL_NAME);
  BLE.setAdvertisedService(dataService);
  dataService.addCharacteristic(sensorChar);
  BLE.addService(dataService);
  BLE.advertise();

  Serial.println("Advertising...");
  IMU.setContinuousMode();

  // Indicate ready state
  digitalWrite(LEDB, HIGH);
  digitalWrite(LEDR, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
}

// ==== Loop (non-blocking) ====
void loop() {
  // Process BLE events frequently
  BLE.poll();

  // Track connection state (non-blocking)
  BLEDevice central = BLE.central();
  if (central && central.connected()) {
    if (!connectedOnce) {
      connectedOnce = true;
      connectedMillis = millis();
      Serial.print("Connected: ");
      Serial.println(central.address());
      // Small grace period after connect for discovery on central
    }
  } else {
    connectedOnce = false;
  }

  // Read IMU one frame when both sensors have data (no while-drain)
  while (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(imu_frame[0], imu_frame[1], imu_frame[2]);
    IMU.readGyroscope(imu_frame[3], imu_frame[4], imu_frame[5]);

    float* baselines = (baselineToggle == 0) ? baselinesA : baselinesB;
    baselineToggle ^= 1;

    // Update baselines and write residuals into ring buffer
    for (int i = 0; i < IMU_SAMPLE_CHANNEL; ++i) {
      float alpha = BASELINE_ALPHAS[i];
      baselines[i] = alpha * baselines[i] + (1.0f - alpha) * imu_frame[i];
      float delta = (imu_frame[i] - baselines[i]) * MULT_ARRAY[i];
      clamp_int8(delta, ring_buffer[ring_write_idx][i]);
    }

    // Advance ring index
    ring_write_idx = (ring_write_idx + 1) % IMU_SAMPLE_COUNT;
    if (!ring_full && ring_write_idx == 0) {
      ring_full = true;
      Serial.println("Ring READY");
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  // Button edge detection (INPUT_PULLUP: LOW = pressed)
  int curBtn = digitalRead(GESTURE_BUTTON);
  bool pressedEdge = (prevBtn == HIGH && curBtn == LOW);
  prevBtn = curBtn;

  // Start capture on edge (only if ring has enough data and we are connected)
  if (pressedEdge && !capturing && ring_full && connectedOnce) {
    capturing = true;

    // PRE_SAMPLES: copy past samples from ring
    int startIdx = (ring_write_idx - PRE_SAMPLES + IMU_SAMPLE_COUNT) % IMU_SAMPLE_COUNT;
    for (int i = 0; i < PRE_SAMPLES; ++i) {
      int idx = (startIdx + i) % IMU_SAMPLE_COUNT;
      for (int j = 0; j < IMU_SAMPLE_CHANNEL; ++j) {
        batch[i][j] = (uint8_t)ring_buffer[idx][j];
      }
    }
    batch_count = PRE_SAMPLES;
    digitalWrite(LEDB, LOW);
    Serial.println("Capture started");
  }

  // While capturing, append latest frame to batch until TOTAL_SAMPLES
  if (capturing && batch_count < TOTAL_SAMPLES) {
    // Latest written frame is at ring_write_idx - 1
    int latestIdx = (ring_write_idx - 1 + IMU_SAMPLE_COUNT) % IMU_SAMPLE_COUNT;
    for (int j = 0; j < IMU_SAMPLE_CHANNEL; ++j) {
      batch[batch_count][j] = (uint8_t)ring_buffer[latestIdx][j];
    }
    batch_count++;
  }

  // When batch is complete, send over BLE with pacing
  if (capturing && batch_count == TOTAL_SAMPLES) {
    capturing = false;
    Serial.print("Batch ready: ");
    Serial.println(batch_count);
    batch_count = 0;
    digitalWrite(LEDB, HIGH);

    // Give central a little time after connect before sending (discovery grace)
    digitalWrite(LEDR, LOW);
    if (millis() - connectedMillis < 1500) {
      delay(1500 - (millis() - connectedMillis));
    }

    // Send as 20-byte chunks
    uint8_t s = 0;
    const uint8_t* payload = reinterpret_cast<const uint8_t*>(&batch[0][0]);
    const size_t totalBytes = (size_t)TOTAL_SAMPLES * IMU_SAMPLE_CHANNEL; // 600
    for (size_t off = 0; off < totalBytes; off += NOTIFY_CHUNK) {
      size_t n = (off + NOTIFY_CHUNK <= totalBytes) ? NOTIFY_CHUNK : (totalBytes - off);
      sensorChar.setValue(payload + off, n); // peripheral: setValue triggers notify if subscribed
      BLE.poll(); // ensure BLE event processing while sending
      delay(NOTIFY_DELAY_MS*10);
      s++;
    }
    Serial.print(s);
    Serial.println(" Batch sent");
    digitalWrite(LEDR, HIGH);
  }
}