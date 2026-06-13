#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_BMP085 bmp;
Adafruit_MPU6050 mpu;

int32_t presionRef  = 0;
unsigned long ultimaBaro = 0;
unsigned long ultimaImu  = 0;

float roll = 0, pitch = 0, yaw = 0;
const float ALPHA = 0.96f;

void setup() {
    Serial.begin(115200);
    unsigned long t = millis();
    while (!Serial && millis() - t < 3000) { yield(); }

    Wire.begin();
    Wire.setClock(400000L);

    // ── BMP180 ────────────────────────────────────────────────────────────────
    if (!bmp.begin(BMP085_STANDARD)) {
        while (1) { Serial.println(">ERR:BMP"); delay(500); }
    }
    Serial.println(">OK:BMP");
    delay(100);
    presionRef = (bmp.readPressure() + bmp.readPressure() + bmp.readPressure()) / 3;

    // ── Escáner I2C ───────────────────────────────────────────────────────────
    Serial.println(">I2C:scan");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print(">I2C:0x"); Serial.println(addr, HEX);
        }
    }
    Serial.println(">I2C:done");

    // ── MPU-6050 @ 0x68 (ADO=GND, sin interrupción) ──────────────────────────
    if (!mpu.begin(0x68)) {
        while (1) { Serial.println(">ERR:MPU"); delay(500); }
    }
    Serial.println(">OK:MPU@0x68");

    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Calentar filtro complementario: 500 ciclos a ~10 ms = ~5 s
    Serial.println(">WARM:start");
    ultimaImu = millis();
    for (int i = 0; i < 500; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        unsigned long ahora = millis();
        float dt = (ahora - ultimaImu) / 1000.0f;
        ultimaImu = ahora;

        float accRoll  = atan2f(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
        float accPitch = atan2f(-a.acceleration.x,
                                sqrtf(a.acceleration.y * a.acceleration.y +
                                      a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;
        roll  = ALPHA * (roll  + g.gyro.x * RAD_TO_DEG * dt) + (1.0f - ALPHA) * accRoll;
        pitch = ALPHA * (pitch + g.gyro.y * RAD_TO_DEG * dt) + (1.0f - ALPHA) * accPitch;
        yaw  += g.gyro.z * RAD_TO_DEG * dt;
        delay(10);
    }
    Serial.println(">WARM:done");

    presionRef = bmp.readPressure();
    Serial.println(">OK:1");
}

void loop() {
    unsigned long ahora = millis();

    // ── BMP180 cada 200 ms ────────────────────────────────────────────────────
    if (ahora - ultimaBaro >= 200) {
        ultimaBaro = ahora;
        int32_t presion = bmp.readPressure();
        float   altitud = (float)(presionRef - presion) / 11.3f;
        float   tempC   = bmp.readTemperature();
        Serial.print(">Alt:"); Serial.println(altitud, 2);
        Serial.print(">Pre:"); Serial.println(presion);
        Serial.print(">Tmp:"); Serial.println(tempC, 1);
    }

    // ── MPU-6050 continuo (filtro complementario) ─────────────────────────────
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float dt = (ahora - ultimaImu) / 1000.0f;
    ultimaImu = ahora;

    float accRoll  = atan2f(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
    float accPitch = atan2f(-a.acceleration.x,
                            sqrtf(a.acceleration.y * a.acceleration.y +
                                  a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;
    roll  = ALPHA * (roll  + g.gyro.x * RAD_TO_DEG * dt) + (1.0f - ALPHA) * accRoll;
    pitch = ALPHA * (pitch + g.gyro.y * RAD_TO_DEG * dt) + (1.0f - ALPHA) * accPitch;
    yaw  += g.gyro.z * RAD_TO_DEG * dt;

    Serial.print(">R:");  Serial.println(roll,  1);
    Serial.print(">P:");  Serial.println(pitch, 1);
    Serial.print(">Y:");  Serial.println(yaw,   1);
}
