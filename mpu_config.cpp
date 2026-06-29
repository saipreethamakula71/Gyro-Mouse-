

#include <Arduino.h>
#include <Wire.h>

// constants
#define MPU_ADDR      0x68
#define CAL_SAMPLES   1000
#define MA_WINDOW     5

// ─────────────────────────────────────────────
//  Kalman filter (scalar, one per axis)
// ─────────────────────────────────────────────
struct KalmanAxis {
    float Q, R, P, x, K;

    KalmanAxis(float q, float r)
        : Q(q), R(r), P(1.0f), x(0.0f), K(0.0f) {}

    void reset(float initial = 0.0f) { x = initial; P = 1.0f; }

    float update(float z) {
        P += Q;
        K  = P / (P + R);
        x += K * (z - x);
        P  = (1.0f - K) * P;
        return x;
    }
};

// ─────────────────────────────────────────────
//  Moving-average filter
// ─────────────────────────────────────────────
struct MAFilter {
    float buf[MA_WINDOW] = {};
    int   idx   = 0;
    bool  ready = false;

    void reset() {
        for (int i = 0; i < MA_WINDOW; i++) buf[i] = 0;
        idx = 0; ready = false;
    }

    float update(float val) {
        buf[idx] = val;
        idx = (idx + 1) % MA_WINDOW;
        if (idx == 0) ready = true;
        int   n   = ready ? MA_WINDOW : idx;
        float sum = 0;
        for (int i = 0; i < n; i++) sum += buf[i];
        return sum / n;
    }
};

// ─────────────────────────────────────────────
//  Processed sensor output
// ─────────────────────────────────────────────
struct MPUData {
    float ax, ay, az;   // g
    float gx, gy, gz;   // deg/s
    float temp;         // °C
};

// ─────────────────────────────────────────────
//  MPUConfig
// ─────────────────────────────────────────────
class MPUConfig {
public:
    MPUConfig() = default;

    bool begin(uint8_t sda = 21, uint8_t scl = 22,
               uint32_t freq = 400000);

    void calibrate();
    bool read(MPUData &out);

    float gNoiseFloor() const { return _gNoise; }

private:
    bool _write(uint8_t reg, uint8_t val);
    bool _read (uint8_t reg, uint8_t *buf, uint8_t len);
    bool _readRaw(float &ax, float &ay, float &az,
                  float &gx, float &gy, float &gz,
                  float &temp);

    float _offAx=0, _offAy=0, _offAz=0;
    float _offGx=0, _offGy=0, _offGz=0;
    float _gNoise = 2.0f;   // default before calibration

    // Q=0.001 keeps filter responsive to real motion
    // R=0.03  rejects noise without killing small movements
    KalmanAxis _kAx{0.001f, 0.03f};
    KalmanAxis _kAy{0.001f, 0.03f};
    KalmanAxis _kAz{0.001f, 0.03f};
    KalmanAxis _kGx{0.001f, 0.03f};
    KalmanAxis _kGy{0.001f, 0.03f};
    KalmanAxis _kGz{0.001f, 0.03f};

    MAFilter _maAx, _maAy, _maAz;
    MAFilter _maGx, _maGy, _maGz;
};

// ─────────────────────────────────────────────
//  Inline implementation
// ─────────────────────────────────────────────

inline bool MPUConfig::_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

inline bool MPUConfig::_read(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)MPU_ADDR, len);
    if ((uint8_t)Wire.available() != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

inline bool MPUConfig::_readRaw(float &ax, float &ay, float &az,
                                float &gx, float &gy, float &gz,
                                float &temp) {
    uint8_t raw[14];
    if (!_read(0x3B, raw, 14)) {
        ax = ay = az = gx = gy = gz = temp = 0;
        return false;
    }
    ax   = (int16_t)(raw[0]  << 8 | raw[1])  / 16384.0f;
    ay   = (int16_t)(raw[2]  << 8 | raw[3])  / 16384.0f;
    az   = (int16_t)(raw[4]  << 8 | raw[5])  / 16384.0f;
    temp = (int16_t)(raw[6]  << 8 | raw[7])  / 340.0f + 36.53f;
    gx   = (int16_t)(raw[8]  << 8 | raw[9])  / 131.0f;
    gy   = (int16_t)(raw[10] << 8 | raw[11]) / 131.0f;
    gz   = (int16_t)(raw[12] << 8 | raw[13]) / 131.0f;
    return true;
}

inline bool MPUConfig::begin(uint8_t sda, uint8_t scl, uint32_t freq) {
    Wire.begin(sda, scl);
    Wire.setClock(freq);

    if (!_write(0x6B, 0x80)) return false;
    delay(500);
    _write(0x6B, 0x00);  delay(100);
    _write(0x6C, 0x00);  delay(10);
    _write(0x19, 0x00);  delay(10);
    _write(0x1A, 0x02);  delay(10);   // DLPF 94 Hz
    _write(0x1B, 0x00);  delay(10);   // gyro  ±250 °/s
    _write(0x1C, 0x00);  delay(200);  // accel ±2 g
    return true;
}

inline void MPUConfig::calibrate() {
    Serial.println("[CAL] Keep sensor FLAT and STILL for 3 s ...");
    delay(3000);

    // Pass 1 – mean offsets
    double sax=0,say=0,saz=0, sgx=0,sgy=0,sgz=0;
    float  ax,ay,az,gx,gy,gz,tp;

    for (int i = 0; i < CAL_SAMPLES; i++) {
        _readRaw(ax,ay,az,gx,gy,gz,tp);
        sax+=ax; say+=ay; saz+=az;
        sgx+=gx; sgy+=gy; sgz+=gz;
        delay(3);
        if (i % 200 == 0) Serial.print('.');
    }
    Serial.println();

    _offAx = sax/CAL_SAMPLES;
    _offAy = say/CAL_SAMPLES;
    _offAz = (saz/CAL_SAMPLES) - 1.0f;
    _offGx = sgx/CAL_SAMPLES;
    _offGy = sgy/CAL_SAMPLES;
    _offGz = sgz/CAL_SAMPLES;

    // Pass 2 – variance -> noise floor
    double vgx=0,vgy=0,vgz=0;
    for (int i = 0; i < CAL_SAMPLES; i++) {
        _readRaw(ax,ay,az,gx,gy,gz,tp);
        float dgx = gx - _offGx;
        float dgy = gy - _offGy;
        float dgz = gz - _offGz;
        vgx += dgx*dgx; vgy += dgy*dgy; vgz += dgz*dgz;
        delay(3);
    }

    float sigmaMax = fmaxf(fmaxf(sqrtf(vgx/CAL_SAMPLES),
                                  sqrtf(vgy/CAL_SAMPLES)),
                                  sqrtf(vgz/CAL_SAMPLES));
    // 2.5-sigma: enough to kill noise, not so high it kills slow motion
    _gNoise = sigmaMax * 2.5f;

    Serial.printf("[CAL] offsets  gx=%.4f gy=%.4f gz=%.4f\n", _offGx,_offGy,_offGz);
    Serial.printf("[CAL] gyro noise floor = %.4f deg/s (2.5-sigma)\n", _gNoise);

    // Reset filters with clean state
    _kAx.reset(); _kAy.reset(); _kAz.reset();
    _kGx.reset(); _kGy.reset(); _kGz.reset();
    _maAx.reset(); _maAy.reset(); _maAz.reset();
    _maGx.reset(); _maGy.reset(); _maGz.reset();
}

inline bool MPUConfig::read(MPUData &out) {
    float ax, ay, az, gx, gy, gz, temp;
    if (!_readRaw(ax,ay,az,gx,gy,gz,temp)) return false;

    ax -= _offAx;  ay -= _offAy;  az -= _offAz;
    gx -= _offGx;  gy -= _offGy;  gz -= _offGz;

    ax = _kAx.update(ax);  ay = _kAy.update(ay);  az = _kAz.update(az);
    gx = _kGx.update(gx);  gy = _kGy.update(gy);  gz = _kGz.update(gz);

    out.ax   = _maAx.update(ax);
    out.ay   = _maAy.update(ay);
    out.az   = _maAz.update(az);
    out.gx   = _maGx.update(gx);
    out.gy   = _maGy.update(gy);
    out.gz   = _maGz.update(gz);
    out.temp = temp;
    return true;
}