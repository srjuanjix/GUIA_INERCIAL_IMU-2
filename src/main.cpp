#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include <Adafruit_BMP085.h>

// Servo via LEDC — API v2.x (ledcSetup / ledcAttachPin / ledcWrite / ledcDetachPin)
// 50 Hz, 16 bits: periodo 20 ms → 0°=1 ms (3277 cnt), 180°=2 ms (6554 cnt)
class Servo {
    int     _pin     = -1;
    uint8_t _channel;
    static  uint8_t _nextChannel;
public:
    Servo() : _channel(_nextChannel++) {}
    void attach(int pin) {
        _pin = pin;
        ledcSetup(_channel, 50, 16);
        ledcAttachPin(pin, _channel);
    }
    void write(int angle) {
        if (_pin < 0) return;
        uint32_t duty = 3277u + (uint32_t)((long)angle * 3277 / 180);
        ledcWrite(_channel, duty);
    }
    void detach() {
        if (_pin >= 0) ledcDetachPin(_pin);
        _pin = -1;
    }
};
uint8_t Servo::_nextChannel = 0;

// Filtro de Kalman escalar 1D (proceso + medición)
struct Kalman1D {
    float Q, R, P = 1.0f, x = 0.0f;
    Kalman1D(float q, float r) : Q(q), R(r) {}
    float update(float z) {
        P += Q;
        float K = P / (P + R);
        x += K * (z - x);
        P *= (1.0f - K);
        return x;
    }
};

MPU9250 mpu;
Adafruit_BMP085 bmp;

// Instancias Kalman — Q: ruido de proceso | R: ruido de medición
// Ángulos: Q pequeño (confiamos en AHRS), R moderado (suaviza vibración)
// Baro: R más alto (barómetro es ruidoso a corto plazo)
Kalman1D kRoll (0.001f, 0.05f);
Kalman1D kPitch(0.001f, 0.05f);
Kalman1D kYaw  (0.001f, 0.10f);
Kalman1D kAlt  (0.010f, 0.50f);
Kalman1D kVelZ (0.010f, 0.30f);

// --- CONFIGURACIÓN DE SERVOS ---
Servo servoAleron;
Servo servoElevador;
Servo servoTimon;
Servo motorESC;

// Pines Arduino Nano ESP32 (D2=GPIO5, D3=GPIO6, D4=GPIO7, D5=GPIO8, D6=GPIO9)
const int PIN_ALERON   = 3;
const int PIN_ELEVADOR = 4;
const int PIN_TIMON    = 5;
const int PIN_MOTOR    = 6;
const int PIN_INT_IMU  = 2;

// --- VARIABLES DE ORIENTACIÓN Y NAVEGACIÓN ---
float roll, pitch, yaw;
float rumboMagnetico;

float targetRoll  = 0.0;
float targetPitch = 0.0;

float lastRollError  = 0.0;
float lastPitchError = 0.0;
float integralPitch  = 0.0;

const float Kp_roll  = 1.8, Kd_roll  = 0.5;
const float Kp_pitch = 2.0, Ki_pitch = 0.3, Kd_pitch = 0.6;
const float dt = 0.005;  // 200 Hz

// --- VARIABLES BAROMÉTRICAS (GY-68 / BMP180) ---
// La altitud relativa es 0 en el punto de arranque.
// Filtro complementario: IMU integra vel. vertical (alta frecuencia),
// barómetro corrige la deriva (baja frecuencia).
int32_t  presionRef  = 0;      // Presión de referencia al inicio (Pa)
float    altitudRel  = 0.0;    // Altitud relativa (m), 0 = posición inicial
float    velZ        = 0.0;    // Velocidad vertical filtrada (m/s)
float    velZ_imu    = 0.0;    // Vel. vertical integrada desde acelerómetro

unsigned long ultimaBaro = 0;
const uint16_t BARO_MS   = 50;  // Lectura barométrica cada 50 ms

// Ganancia del amortiguador de altitud sobre el elevador
const float Kp_velZ = 0.4;

// Offsets de referencia horizontal (capturados en arranque)
// Permiten que Roll=0°, Pitch=0°, Yaw=0° y Acc_Z=+1.0g con el sensor nivelado.
float rollOffset  = 0.0f;
float pitchOffset = 0.0f;
float yawOffset   = 0.0f;
float accXOffset  = 0.0f;
float accYOffset  = 0.0f;
float accZOffset  = 0.0f;

// Control de interrupción IMU
volatile bool mpuInterrupt = false;
void IRAM_ATTR dmpDataReady() { mpuInterrupt = true; }

// Normaliza un ángulo al rango ±180° para aplicar offsets sin discontinuidades
static inline float wrapAngle(float a) {
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static void fatalErr(const char* msg) {
    while (1) { Serial.println(msg); delay(500); }
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Wire.begin();

    if (!bmp.begin(BMP085_STANDARD)) fatalErr(">EB:1");

    // Restaurar 400 kHz después del Wire.begin() interno de bmp.begin()
    Wire.setClock(400000L);

    pinMode(PIN_INT_IMU, INPUT_PULLUP);

    // Inicializar MPU9250 / GY-9250 con bus I2C limpio y velocidad correcta
    if (!mpu.setup(0x68)) fatalErr(">EI:1");
    mpu.calibrateAccelGyro();
    mpu.calibrateMag();
    attachInterrupt(digitalPinToInterrupt(PIN_INT_IMU), dmpDataReady, RISING);

    // Promedio de 3 lecturas para una referencia de presión estable (altitud = 0 m)
    delay(100);
    int32_t p1 = bmp.readPressure(); delay(50);
    int32_t p2 = bmp.readPressure(); delay(50);
    int32_t p3 = bmp.readPressure();
    presionRef = (p1 + p2 + p3) / 3;

    // Calentar el filtro AHRS: el filtro NO corre durante las calibraciones,
    // arranca desde 0° en la primera llamada a mpu.update(). Se necesitan
    // ~9 s (1800 ciclos a 200 Hz) para que roll/pitch/yaw converjan.
    // La referencia se captura en los últimos 200 ciclos (valores ya estables).
    {
        float sR=0,sP=0,sY=0,sAX=0,sAY=0,sAZ=0;
        for (uint16_t i=0; i<2000; i++) {
            while (!mpuInterrupt) {}
            mpuInterrupt = false;
            mpu.update();
            if (i >= 1800) {
                sR+=mpu.getRoll();   sP+=mpu.getPitch();  sY+=mpu.getYaw();
                sAX+=mpu.getAccX(); sAY+=mpu.getAccY(); sAZ+=mpu.getAccZ();
            }
        }
        rollOffset=sR/200.0f; pitchOffset=sP/200.0f; yawOffset=sY/200.0f;
        accXOffset=sAX/200.0f; accYOffset=sAY/200.0f;
        accZOffset=sAZ/200.0f - 1.0f;
    }

    // Recapturar presionRef tras el calentamiento → altitud = 0 m al arrancar
    presionRef = bmp.readPressure();
    altitudRel = 0.0f;
    Serial.println(">OK:1");

    // Servos en posición neutra de seguridad
    servoAleron.attach(PIN_ALERON);
    servoElevador.attach(PIN_ELEVADOR);
    servoTimon.attach(PIN_TIMON);
    motorESC.attach(PIN_MOTOR);
    servoAleron.write(90);
    servoElevador.write(90);
    servoTimon.write(90);
    motorESC.write(0);
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    unsigned long ahora = millis();

    // ── Lectura barométrica (bloquea ~13 ms, se repite cada BARO_MS) ──────
    if (ahora - ultimaBaro >= BARO_MS) {
        float dtB  = (ahora - ultimaBaro) / 1000.0f;
        ultimaBaro = ahora;

        // Altitud relativa: ~11.3 Pa/m a nivel del mar (lineal, válido ±200 m)
        int32_t presion = bmp.readPressure();
        float nuevaAlt  = (float)(presionRef - presion) / 11.3f;

        // Velocidad vertical barométrica (derivada numérica)
        float baroVelZ = (nuevaAlt - altitudRel) / dtB;

        // Filtro complementario: 95 % IMU (respuesta rápida) + 5 % baro (anti-deriva)
        velZ     = 0.95f * velZ_imu + 0.05f * baroVelZ;
        velZ_imu = velZ;   // Reiniciar integración con valor filtrado (sin deriva)
        altitudRel = kAlt.update(nuevaAlt);
        velZ       = kVelZ.update(velZ);

        Serial.print(">BA:"); Serial.println(altitudRel, 1);
        Serial.print(">BP:"); Serial.println(presion);
        Serial.print(">VZ:"); Serial.println(velZ, 1);
    }

    // ── Loop IMU a 200 Hz (dirigido por interrupción) ─────────────────────
    if (!mpuInterrupt) return;
    mpuInterrupt = false;

    if (mpu.update()) {
        // Aplicar offsets de referencia horizontal (capturados en arranque)
        float rawRoll  = constrain(wrapAngle(mpu.getRoll()  - rollOffset),  -180.0f, 180.0f);
        float rawPitch = constrain(wrapAngle(mpu.getPitch() - pitchOffset), -180.0f, 180.0f);
        float rawYaw   = constrain(wrapAngle(mpu.getYaw()   - yawOffset),   -180.0f, 180.0f);

        // A pitch negativo el filtro Mahony acopla errores del magnetómetro al eje
        // de roll (tilt compensation asimétrica). Se incrementa R del Kalman de roll
        // proporcionalmente: a pitch=0 → R=0.05 (normal), a pitch≤-40° → R=0.25 (máx).
        kRoll.R = 0.05f + constrain(-rawPitch * 0.005f, 0.0f, 0.20f);

        roll  = kRoll .update(rawRoll);
        pitch = kPitch.update(rawPitch);
        yaw   = kYaw  .update(rawYaw);

        // Aceleraciones con offset: X=0g, Y=0g, Z=+1.0g en posición horizontal
        float accX = mpu.getAccX() - accXOffset;
        float accY = mpu.getAccY() - accYOffset;
        float accZ = mpu.getAccZ() - accZOffset;

        float mx = mpu.getMagX(), my = mpu.getMagY();
        rumboMagnetico = atan2(my, mx) * RAD_TO_DEG;
        if (rumboMagnetico < 0) rumboMagnetico += 360.0f;

        // Aceleración vertical lineal (marco mundo, sin gravedad)
        float cosR = cos(roll  * DEG_TO_RAD);
        float sinR = sin(roll  * DEG_TO_RAD);
        float cosP = cos(pitch * DEG_TO_RAD);
        float sinP = sin(pitch * DEG_TO_RAD);
        float az_lin = (-sinP * accX
                        + sinR*cosP * accY
                        + cosR*cosP * accZ - 1.0f) * 9.81f;  // m/s²

        velZ_imu += az_lin * dt;

        // ── Control de actitud (Roll / Alerones) ──────────────────────────
        float errorRoll = targetRoll - roll;
        float derivRoll = (errorRoll - lastRollError) / dt;
        lastRollError   = errorRoll;
        int servoAleronPos = constrain(
            90 + (int)(errorRoll*Kp_roll + derivRoll*Kd_roll), 45, 135);

        // ── Control de actitud (Pitch / Elevador) + amortiguación de altitud ──
        float errorPitch  = targetPitch - pitch;
        integralPitch    += errorPitch * dt;
        integralPitch     = constrain(integralPitch, -50.0f, 50.0f);  // anti-windup
        float derivPitch  = (errorPitch - lastPitchError) / dt;
        lastPitchError    = errorPitch;
        float corrElev = errorPitch*Kp_pitch + integralPitch*Ki_pitch + derivPitch*Kd_pitch
                       + constrain(-velZ * Kp_velZ, -15.0f, 15.0f);
        int servoElevadorPos = constrain(90 + (int)corrElev, 45, 135);

        servoAleron.write(servoAleronPos);
        servoElevador.write(servoElevadorPos);

        Serial.print(">R:");  Serial.println(roll,           1);
        Serial.print(">P:");  Serial.println(pitch,          1);
        Serial.print(">Y:");  Serial.println(yaw,            1);
        Serial.print(">M:");  Serial.println(rumboMagnetico, 1);
        Serial.print(">AX:"); Serial.println(accX,           1);
        Serial.print(">AY:"); Serial.println(accY,           1);
        Serial.print(">AZ:"); Serial.println(accZ,           1);
        Serial.print(">SA:"); Serial.println(servoAleronPos);
        Serial.print(">SE:"); Serial.println(servoElevadorPos);
    }
}
