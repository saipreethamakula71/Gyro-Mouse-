#include <Arduino.h>
#include <Wire.h>
#include <BleMouse.h>

#include "mpu_config.cpp"

// ─────────────────────────────────────────────
//  Tuning knobs
// ─────────────────────────────────────────────
static constexpr float         SENSITIVITY   = 0.15f;
static constexpr unsigned long IDLE_TIMEOUT  = 3000;  // ms
static constexpr float         DEADZONE_MIN  = 1.5f;  // deg/s absolute floor
static constexpr float         DEADZONE_MAX  = 4.0f;  // deg/s cap so cal can't over-suppress

// ─────────────────────────────────────────────
//  Touch click config
//  T5 = GPIO15  ->  LEFT  click
//  T6 = GPIO14  ->  RIGHT click
// ─────────────────────────────────────────────
static constexpr int TOUCH_PIN_LEFT  = T5;
static constexpr int TOUCH_PIN_RIGHT = T6;
static constexpr int TOUCH_THRESHOLD = 40;

// ─────────────────────────────────────────────
//  Globals
// ─────────────────────────────────────────────
MPUConfig mpu;
BleMouse  bleMouse("MPU Mouse", "Sai", 100);

float         deadzone          = DEADZONE_MIN;
unsigned long lastMovementTime  = 0;
bool          idleMode          = false;
bool          lastTouchedLeft   = false;
bool          lastTouchedRight  = false;

// ─────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("=== MPU BLE Mouse ===");

    if (!mpu.begin(21, 22, 400000)) {
        Serial.println("[ERROR] MPU6050 not found - check wiring!");
        while (true) delay(1000);
    }

    mpu.calibrate();

    // Clamp deadzone between MIN and MAX so calibration noise floor
    // can't be too tight (cursor won't move) or too loose (drift remains)
    deadzone = constrain(mpu.gNoiseFloor(), DEADZONE_MIN, DEADZONE_MAX);
    Serial.printf("[INIT] Deadzone = %.3f deg/s\n", deadzone);

    bleMouse.begin();
    lastMovementTime = millis();
    Serial.println("BLE Mouse ready.");
    Serial.println("Touch GPIO15 = LEFT  |  Touch GPIO14 = RIGHT");
}

// ─────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────
void loop() {
    MPUData d;

    if (!mpu.read(d)) {
        Serial.println("[WARN] MPU read failed");
        delay(10);
        return;
    }

    // ── Gyro -> velocity ──────────────────────
    float hVel = -d.gz;   // yaw   -> horizontal
    float vVel = -d.gx;   // pitch -> vertical

    // Apply deadzone
    if (fabsf(hVel) < deadzone) hVel = 0.0f;
    if (fabsf(vVel) < deadzone) vVel = 0.0f;

    int8_t dx = (int8_t)(hVel * SENSITIVITY);
    int8_t dy = (int8_t)(vVel * SENSITIVITY);

    bool moving = (dx != 0 || dy != 0);

    // ── Idle detection ────────────────────────
    if (moving) {
        lastMovementTime = millis();
        idleMode         = false;
    } else if (millis() - lastMovementTime > IDLE_TIMEOUT) {
        idleMode = true;
    }

    // ── Touch -> Left click (GPIO15 / T5) ─────
    bool touchedLeft = (touchRead(TOUCH_PIN_LEFT) < TOUCH_THRESHOLD);
    if (bleMouse.isConnected()) {
        if (touchedLeft && !lastTouchedLeft) {
            bleMouse.press(MOUSE_LEFT);
            Serial.println("[CLICK] LEFT press");
        } else if (!touchedLeft && lastTouchedLeft) {
            bleMouse.release(MOUSE_LEFT);
            Serial.println("[CLICK] LEFT release");
        }
    }
    lastTouchedLeft = touchedLeft;

    // ── Touch -> Right click (GPIO14 / T6) ────
    bool touchedRight = (touchRead(TOUCH_PIN_RIGHT) < TOUCH_THRESHOLD);
    if (bleMouse.isConnected()) {
        if (touchedRight && !lastTouchedRight) {
            bleMouse.press(MOUSE_RIGHT);
            Serial.println("[CLICK] RIGHT press");
        } else if (!touchedRight && lastTouchedRight) {
            bleMouse.release(MOUSE_RIGHT);
            Serial.println("[CLICK] RIGHT release");
        }
    }
    lastTouchedRight = touchedRight;

    // ── BLE movement report ───────────────────
    if (bleMouse.isConnected() && !idleMode && moving) {
        bleMouse.move(dx, (int8_t)(-dy));
        Serial.printf("DX:%4d DY:%4d | gz=%.2f gx=%.2f\n",
                      dx, dy, d.gz, d.gx);
    }

    delay(idleMode ? 1000 : 10);
}