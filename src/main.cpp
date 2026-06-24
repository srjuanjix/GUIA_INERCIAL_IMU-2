// ─────────────────────────────────────────────────────────────────────────────
// GUIA INERCIAL IMU-3
// POR JAB7B7 2026
//   PROGRAMA PARA ESP32-S3-NANO  
//   DE:
//   TELEMETRIA: DE SENSORES GIROSCOPICOS-MAGNETICOS Y BAROMETRICOS (EN LOCAL USB Y VIA WIFI)
//   CONTROL :   PILOTO AUTOMÁTICO DE ESTABILIZACIÓN CON CONTROL DE PITH, ROLL Y YAW MEDIANTE PID
//
//   SE REQUIERE INICIALIZACIÓN Y CALIBRACIÓN INICIAL.
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <MPU9250.h>
#include <Adafruit_BMP085.h>
#include <math.h>

// ── Pines MPU-9250 ────────────────────────────────────────────────────────────
#define INT_PIN   D4
#define FSYNC_PIN D5

// ── Punto de acceso WiFi propio ───────────────────────────────────────────────
#define AP_SSID  "IMU_Guia_Inercial"
#define AP_PASS  "imu12345"
#define UDP_PORT 4210

// ── Servo via LEDC ────────────────────────────────────────────────────────────
#define SERVO_PIN  D6
#define SERVO_CH   4
#define SERVO_FREQ 50
#define SERVO_BITS 16


// ── Definición del pin del LED y variable global de error ────────────────────── (GPIO 21 )

const int LED_PIN = 10;


// 0 = Sin error. (Led apagado)
// 1 = Error de Telemetría (Parpadeo lento de vida)
// 2 = Error de IMU / Sensor MPU9250-6500 (Parpadeo rápido)
// 3 = Error de sensor barometro GY-68 (Patrón de doble destello)
// 4 = Error Crítico de servos (LED fijo encendido)

volatile int codigoError = 0; 

// Variables para la gestión del tiempo no bloqueante

unsigned long tiempoAnteriorLED = 0;
int fasePatron = 0;                                     // Lleva la cuenta del paso actual en patrones complejos
bool estadoLED = false;

// ──────────────────────────────────────────────────────────────────────────────────

static int SERVO_US_MIN = 1000;
static int SERVO_US_MAX = 2000;

static uint32_t us2duty(int us) {
    return (uint32_t)((uint64_t)(long)us * 65535UL / 20000UL);
}
static void servoWrite(int deg) {
    int us = map(deg, 0, 180, SERVO_US_MIN, SERVO_US_MAX);
    ledcWrite(SERVO_CH, us2duty(constrain(us, SERVO_US_MIN, SERVO_US_MAX)));
}

// ── PID timón de profundidad ──────────────────────────────────────────────────
const float KP = 2.0f, KI = 0.3f, KD = 0.15f;
const float I_MAX = 20.0f, OUT_MAX = 45.0f;
float pidIntegral = 0.0f;

// ── EKF: tiempo de la última predicción ──────────────────────────────────────
static unsigned long tEkfPrev = 0;

// ── Sensores ──────────────────────────────────────────────────────────────────
MPU9250         mpu;
Adafruit_BMP085 bmp;
bool            bmpOk    = false;
float           altRef   = 0.0f;
float           altEma   = 0.0f;
float           altPrev  = 0.0f;
unsigned long   tBmpPrev = 0;

// ── WiFi / UDP ────────────────────────────────────────────────────────────────
WiFiUDP udp;
bool    wifiOk = false;

static char   udpBuf[320];
static size_t udpLen = 0;

static void udpAdd(const char* s) {
    size_t n = strlen(s);
    if (udpLen + n + 2 < sizeof(udpBuf)) {
        memcpy(udpBuf + udpLen, s, n);
        udpLen += n;
        udpBuf[udpLen++] = '\n';
    }
}
static void udpFlush() {
    if (!wifiOk || udpLen == 0) { udpLen = 0; return; }
    IPAddress apIP = WiFi.softAPIP();
    IPAddress bcast(apIP[0], apIP[1], apIP[2], 255);
    udp.beginPacket(bcast, UDP_PORT);
    udp.write((uint8_t*)udpBuf, udpLen);
    udp.endPacket();
    udpLen = 0;
}
static void send(const char* key, float val, int dec = 1) {
    Serial.print('>'); Serial.print(key); Serial.print(':'); Serial.println(val, dec);
    char tmp[40];
    snprintf(tmp, sizeof(tmp), ">%s:%.*f", key, dec, (double)val);
    udpAdd(tmp);
}
static void sendInt(const char* key, int val) {
    Serial.print('>'); Serial.print(key); Serial.print(':'); Serial.println(val);
    char tmp[32];
    snprintf(tmp, sizeof(tmp), ">%s:%d", key, val);
    udpAdd(tmp);
}

// ─────────────────────────────────────────────────────────────────────────────
// EKF 6-estado
//   x = [φ(rad), θ(rad), ψ(rad), b_p(rad/s), b_q(rad/s), b_r(rad/s)]
//
//   PREDICCIÓN: modelo cinemático de Euler integrado con el giroscopio.
//   MEDICIÓN:   salida del filtro Mahony (φ_m, θ_m, ψ_m) — ya tiene la
//               corrección de ejes del hardware y la fusión accel+gyro+mag.
//               Esto evita cualquier problema de convención de ejes y permite
//               que el EKF se centre en estimar el sesgo residual del giroscopio.
//
//   Ángulos ABSOLUTOS internamente. La salida resta los offsets de alineación
//   para que todo arranque en 0.
// ─────────────────────────────────────────────────────────────────────────────

static float x6[6]      = {};
static float P6[6][6]   = {};
static float rollOff_r  = 0.0f;
static float pitchOff_r = 0.0f;
static float yawOff_r   = 0.0f;
static float accBias_x  = 0.0f;   // bias eje X del chip en reposo (~0.25 g electrónico)
static float accBias_y  = 0.0f;   // bias eje Y del chip en reposo
static float accBias_z  = 0.0f;   // bias eje Z del chip en reposo (incluye ~1 g de gravedad)

// Ruidos de proceso
static const float Q_ANG  = 1e-5f;   // varianza proceso: ángulos    (rad²/s)
static const float Q_BIAS = 1e-6f;   // varianza proceso: sesgos     (rad²/s³)
// Ruidos de medición (Mahony)
static const float R_RP   = 1e-3f;   // roll/pitch  (rad²) — Mahony es preciso en estos
static const float R_Y    = 5e-3f;   // yaw         (rad²) — más ruido por el magnetómetro

// ── Inicialización ────────────────────────────────────────────────────────────
static void ekf_init(float r, float p, float y) {
    x6[0] = r; x6[1] = p; x6[2] = y;
    x6[3] = x6[4] = x6[5] = 0.0f;
    memset(P6, 0, sizeof(P6));
    P6[0][0] = P6[1][1] = P6[2][2] = 0.1f;
    P6[3][3] = P6[4][4] = P6[5][5] = 0.01f;
    Serial.println(">EKF:inicializado");
}

// ── Predicción (~200 Hz) ──────────────────────────────────────────────────────
static void ekf_predict(float gx_r, float gy_r, float gz_r, float dt) {
    const float phi   = x6[0];
    const float theta = constrain(x6[1], -1.5533f, 1.5533f);  // ±89°
    const float bp = x6[3], bq = x6[4], br = x6[5];

    const float pc = gx_r - bp;
    const float qc = gy_r - bq;
    const float rc = gz_r - br;

    const float sp = sinf(phi),   cp = cosf(phi);
    const float ct = cosf(theta), tt = tanf(theta);
    const float ct2 = ct * ct;

    x6[0] += (pc + (qc*sp + rc*cp) * tt) * dt;
    x6[1] += (qc*cp - rc*sp) * dt;
    x6[2] += ((qc*sp + rc*cp) / ct) * dt;
    while (x6[0] >  M_PI) x6[0] -= 2.0f * M_PI;
    while (x6[0] < -M_PI) x6[0] += 2.0f * M_PI;
    while (x6[2] >  M_PI) x6[2] -= 2.0f * M_PI;
    while (x6[2] < -M_PI) x6[2] += 2.0f * M_PI;

    // Jacobiano F = I + (∂f/∂x)·dt
    float F[6][6] = {};
    for (int i = 0; i < 6; i++) F[i][i] = 1.0f;

    F[0][0] += (qc*cp - rc*sp) * tt  * dt;
    F[0][1] += (qc*sp + rc*cp) / ct2 * dt;
    F[0][3]  = -dt;
    F[0][4]  = -sp * tt * dt;
    F[0][5]  = -cp * tt * dt;

    F[1][0] += (-qc*sp - rc*cp) * dt;
    F[1][4]  = -cp * dt;
    F[1][5]  =  sp * dt;

    F[2][0] += (qc*cp - rc*sp) / ct * dt;
    F[2][1] += (qc*sp + rc*cp) * sinf(theta) / ct2 * dt;
    F[2][4]  = -sp / ct * dt;
    F[2][5]  = -cp / ct * dt;

    // P = F·P·Fᵀ + Q
    float FP[6][6] = {};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 6; k++)
                FP[i][j] += F[i][k] * P6[k][j];

    float newP[6][6] = {};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 6; k++)
                newP[i][j] += FP[i][k] * F[j][k];

    for (int i = 0; i < 3; i++) newP[i][i] += Q_ANG  * dt;
    for (int i = 3; i < 6; i++) newP[i][i] += Q_BIAS * dt;

    for (int i = 0; i < 6; i++)
        for (int j = i + 1; j < 6; j++)
            newP[i][j] = newP[j][i] = 0.5f * (newP[i][j] + newP[j][i]);

    memcpy(P6, newP, sizeof(P6));
}

// ── Actualización con salida del filtro Mahony (3 mediciones simultáneas) ─────
// roll_m, pitch_m, yaw_m: ángulos Mahony en RADIANES (absolutos).
// H = [I₃ | 0₃]  →  S = P[0:3,0:3] + R  (3×3).
static void ekf_update_mahony(float roll_m, float pitch_m, float yaw_m) {
    // Innovación
    float inn[3];
    inn[0] = roll_m  - x6[0];
    inn[1] = pitch_m - x6[1];
    inn[2] = yaw_m   - x6[2];
    // Normalizar yaw a (−π, π]
    while (inn[2] >  M_PI) inn[2] -= 2.0f * M_PI;
    while (inn[2] < -M_PI) inn[2] += 2.0f * M_PI;

    // S = P[0:3,0:3] + diag(R_RP, R_RP, R_Y)
    float S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S[i][j] = P6[i][j];
    S[0][0] += R_RP;
    S[1][1] += R_RP;
    S[2][2] += R_Y;

    // Inversión analítica de S (3×3 simétrica)
    float d = S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1])
             -S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0])
             +S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);
    if (fabsf(d) < 1e-12f) return;
    float id = 1.0f / d;
    float Si[3][3];
    Si[0][0] = id*(S[1][1]*S[2][2] - S[1][2]*S[2][1]);
    Si[0][1] = id*(S[0][2]*S[2][1] - S[0][1]*S[2][2]);
    Si[0][2] = id*(S[0][1]*S[1][2] - S[0][2]*S[1][1]);
    Si[1][0] = id*(S[1][2]*S[2][0] - S[1][0]*S[2][2]);
    Si[1][1] = id*(S[0][0]*S[2][2] - S[0][2]*S[2][0]);
    Si[1][2] = id*(S[0][2]*S[1][0] - S[0][0]*S[1][2]);
    Si[2][0] = id*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);
    Si[2][1] = id*(S[0][1]*S[2][0] - S[0][0]*S[2][1]);
    Si[2][2] = id*(S[0][0]*S[1][1] - S[0][1]*S[1][0]);

    // K = P[:,0:3] · Si  (6×3)
    float K[6][3] = {};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                K[i][j] += P6[i][k] * Si[k][j];

    // x += K · inn
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++)
            x6[i] += K[i][j] * inn[j];

    // P = (I − K·H)·P
    // (K·H)[i][j] = K[i][j] si j < 3, 0 en otro caso
    // (I−K·H)·P fila i = P[i] − K[i][0]·P[0] − K[i][1]·P[1] − K[i][2]·P[2]
    float r0[6], r1[6], r2[6];
    memcpy(r0, P6[0], sizeof(r0));
    memcpy(r1, P6[1], sizeof(r1));
    memcpy(r2, P6[2], sizeof(r2));
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            P6[i][j] -= K[i][0]*r0[j] + K[i][1]*r1[j] + K[i][2]*r2[j];

    for (int i = 0; i < 6; i++) {
        for (int j = i + 1; j < 6; j++)
            P6[i][j] = P6[j][i] = 0.5f * (P6[i][j] + P6[j][i]);
        if (P6[i][i] < 1e-9f) P6[i][i] = 1e-9f;
    }
}
// 
// ── Función encargada de gestionar el LED mediante máquina de estados ────────────────
// 
// 0 = Sin error. (Led encendido 1 s y luego apagado)
// 1 = Error de Telemetría (Parpadeo lento de vida)
// 2 = Error de IMU / Sensor MPU9250-6500 (Parpadeo rápido)
// 3 = Error de sensor barometro GY-68 (Patrón de doble destello)
// 4 = Error Crítico de servos (LED fijo encendido)

void actualizarLedEstado() {
    unsigned long tiempoActual = millis();

    switch (codigoError) {
        
        case 1: // ---- Error de Telemetría (Parpadeo de pulso lento / "Heartbeat") ----
            if (tiempoActual - tiempoAnteriorLED >= 1000) { // Cada 1000ms cambia de estado
                tiempoAnteriorLED = tiempoActual;
                estadoLED = !estadoLED;
                digitalWrite(LED_PIN, estadoLED);
            }
            break;

        case 2: // ---- Error de IMU / Sensor MPU9250-6500 (Parpadeo rápido de advertencia) ----
            if (tiempoActual - tiempoAnteriorLED >= 150) { // Cada 150ms
                tiempoAnteriorLED = tiempoActual;
                estadoLED = !estadoLED;
                digitalWrite(LED_PIN, estadoLED);
            }
            break;

        case 3: { // ----  Error de sensor barometro GY-68 (Doble destello sofisticado) ----
            // Secuencia: ON(100ms) → OFF(100ms) → ON(100ms) → OFF(700ms) → repetir
            static const unsigned long T[4] = {100, 100, 100, 700};
            if (!estadoLED && fasePatron == 0) {
                // Primera entrada: encender LED inmediatamente y arrancar temporizador
                estadoLED = true;
                tiempoAnteriorLED = tiempoActual;
                digitalWrite(LED_PIN, HIGH);
            } else if (tiempoActual - tiempoAnteriorLED >= T[fasePatron]) {
                tiempoAnteriorLED = tiempoActual;
                fasePatron = (fasePatron + 1) % 4;
                estadoLED = (fasePatron == 0 || fasePatron == 2);
                digitalWrite(LED_PIN, estadoLED ? HIGH : LOW);
            }
            break;
        }

        case 4: // ---- Error Crítico de servos (LED Fijo) ----
            if (!estadoLED) {
                estadoLED = true;
                digitalWrite(LED_PIN, HIGH);
            }
            break;

        default:
            digitalWrite(LED_PIN, LOW);
            break;
    }
}
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    // ..............................
    pinMode(LED_PIN, OUTPUT);                           // Led de mensajes de error
    digitalWrite(LED_PIN, 1);                           // Enciende 2 S
    delay(2000);
    digitalWrite(LED_PIN, 0);                           // Apaga led de errores
    // ..............................
    Serial.begin(115200);
    unsigned long t = millis();
    while (!Serial && millis() - t < 3000) yield();

    pinMode(INT_PIN,   INPUT);
    pinMode(FSYNC_PIN, OUTPUT);
    digitalWrite(FSYNC_PIN, LOW);

    Wire.begin();
    Wire.setClock(400000L);

    // ── MPU-9250 @ 0x68 ───────────────────────────────────────────────────────
    MPU9250Setting cfg;
    cfg.accel_fs_sel     = ACCEL_FS_SEL::A4G;
    cfg.gyro_fs_sel      = GYRO_FS_SEL::G500DPS;
    cfg.mag_output_bits  = MAG_OUTPUT_BITS::M16BITS;
    cfg.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    cfg.gyro_fchoice     = 0x03;
    cfg.gyro_dlpf_cfg    = GYRO_DLPF_CFG::DLPF_41HZ;
    cfg.accel_fchoice    = 0x01;
    cfg.accel_dlpf_cfg   = ACCEL_DLPF_CFG::DLPF_45HZ;

    if (!mpu.setup(0x68, cfg)) {
        codigoError = 2;
        Serial.println(">ERR:MPU - IMU no encontrada");
        while (1) { actualizarLedEstado(); delay(50); }
    }
    Serial.println(">OK:MPU9250@0x68");

    // ── Calibración acelerómetro + giroscopio ────────────────────────────────
    Serial.println(">CAL:accel+gyro - mantén quieto...");
    mpu.calibrateAccelGyro();
    Serial.println(">CAL:accel+gyro done");

    // ── Calibración magnetómetro opcional ────────────────────────────────────
    Serial.println(">CAL:pulsa 'C' (o envia 'M') en 5s para calibrar mag (figura-8)");
    unsigned long tWait = millis();
    while (millis() - tWait < 5000) {
        mpu.update();   // mantener el filtro corriendo durante la espera
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 'C' || c == 'M') {
                Serial.println(">CAL:mag - mueve en figura-8 ~15s...");
                mpu.calibrateMag();
                Serial.println(">CAL:mag done");
                break;
            }
        }
        delay(50);
    }

    // ── Convergencia Madgwick 10 s + promedio 2 s → referencia estable en 0° ───
    Serial.println(">ALIGN:esperando 12s convergencia Mahony...");
    unsigned long tAlign = millis();
    while (millis() - tAlign < 10000) {  // 10 s de convergencia
        mpu.update();
        delay(10);
    }
    // Promedio del último segundo: elimina ruido puntual y captura bias del acelerómetro
    float sumRoll = 0.0f, sumPitch = 0.0f, sumYsin = 0.0f, sumYcos = 0.0f;
    float sumAX   = 0.0f, sumAY   = 0.0f, sumAZ   = 0.0f;
    int   nSamples = 0;
    unsigned long tAvg = millis();
    while (millis() - tAvg < 2000) {
        mpu.update();
        sumRoll  += mpu.getRoll();
        sumPitch += mpu.getPitch();
        float yr  = mpu.getYaw() * DEG_TO_RAD;
        sumYsin  += sinf(yr);
        sumYcos  += cosf(yr);
        sumAX    += mpu.getAccX();
        sumAY    += mpu.getAccY();
        sumAZ    += mpu.getAccZ();
        nSamples++;
        delay(10);
    }
    rollOff_r  = (sumRoll  / nSamples) * DEG_TO_RAD;
    pitchOff_r = (sumPitch / nSamples) * DEG_TO_RAD;
    yawOff_r   = atan2f(sumYsin / nSamples, sumYcos / nSamples);
    accBias_x  = sumAX / nSamples;           // ~0.25 g (solo bias electrónico, eje horizontal)
    accBias_y  = sumAY / nSamples;           // ~0.25 g (solo bias electrónico, eje horizontal)
    accBias_z  = sumAZ / nSamples + 1.0f;    // solo bias electrónico: se resta la gravedad esperada (-1 g)

    Serial.print(">ALIGN:ref roll=");  Serial.print(rollOff_r  * RAD_TO_DEG, 2);
    Serial.print(" pitch=");           Serial.print(pitchOff_r * RAD_TO_DEG, 2);
    Serial.print(" yaw=");             Serial.println(yawOff_r * RAD_TO_DEG, 2);
    Serial.print(">ALIGN:accBias X="); Serial.print(accBias_x, 4);
    Serial.print(" Y=");               Serial.print(accBias_y, 4);
    Serial.print(" Z=");               Serial.println(accBias_z, 4);

    // EKF arranca en la orientación absoluta real → salida = diferencia = 0
    ekf_init(rollOff_r, pitchOff_r, yawOff_r);

    // ── BMP180 @ 0x77 ─────────────────────────────────────────────────────────
    Serial.println(">BMP:iniciando sensor barometrico...");
    if (!bmp.begin()) {
        Serial.println(">ERR:BMP180 no encontrado - verifica SDA/SCL y alimentacion");
        codigoError = 3;
    } else {
        bmpOk    = true;
        float t0 = bmp.readTemperature();
        float p0 = (float)bmp.readPressure();
        altRef   = 44330.0f * (1.0f - powf(p0 / 101325.0f, 0.1903f));
        altEma   = 0.0f;
        altPrev  = 0.0f;
        tBmpPrev = millis();
        Serial.print(">OK:BMP180@0x77 T="); Serial.print(t0, 1);
        Serial.print("C P="); Serial.print((int)p0);
        Serial.print("Pa AltRef="); Serial.print(altRef, 1); Serial.println("m");
    }

    // ── Servo ─────────────────────────────────────────────────────────────────
    ledcSetup(SERVO_CH, SERVO_FREQ, SERVO_BITS);
    ledcAttachPin(SERVO_PIN, SERVO_CH);
    servoWrite(90);

    // ── WiFi AP ───────────────────────────────────────────────────────────────
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    udp.begin(UDP_PORT);
    wifiOk = true;
    IPAddress apIP = WiFi.softAPIP();
    Serial.print(">AP:"); Serial.print(AP_SSID);
    Serial.print(" IP:"); Serial.println(apIP);

    tEkfPrev = millis();   // evita dt artificialmente grande en la primera iteración
    Serial.println(">OK:1");
}

// ─────────────────────────────────────────────────────────────────────────────
// ..............................................................................
// _______________ CICLO PRINCIPAL DEL PROGRAMA _________________________________
// ..............................................................................
void loop() {
    actualizarLedEstado();           // Siempre se ejecuta, independiente del estado del sensor
    if (!mpu.update()) return;

    unsigned long ahora = millis();
    float dt = constrain((ahora - tEkfPrev) / 1000.0f, 0.001f, 0.05f);
    tEkfPrev = ahora;

    // ── Giroscopio en rad/s (para la predicción del EKF) ─────────────────────
    float gx_r = mpu.getGyroX() * DEG_TO_RAD;
    float gy_r = mpu.getGyroY() * DEG_TO_RAD;
    float gz_r = mpu.getGyroZ() * DEG_TO_RAD;

    // ── Aceleraciones corregidas de bias ─────────────────────────────────────
    // En reposo plano: AX=0.0, AY=0.0, AZ=-1.0 (gravedad en -Z del chip)
    float acc_x = mpu.getAccX() - accBias_x;
    float acc_y = mpu.getAccY() - accBias_y;
    float acc_z = mpu.getAccZ() - accBias_z;

    // ── EKF: predecir con giroscopio, corregir con la fusión del MPU ──────────
    // La medición (φ,θ,ψ) es la salida del filtro Madgwick de la librería, que
    // fusiona ACELERÓMETRO + GIROSCOPIO + MAGNETÓMETRO y resuelve la geometría de
    // ejes del hardware. Por tanto el yaw que entra al Kalman ya está alimentado
    // por el magnetómetro (referencia de norte magnético).
    // El EKF estima el sesgo residual del giróscopo y suaviza la salida.
    ekf_predict(gx_r, gy_r, gz_r, dt);
    ekf_update_mahony(mpu.getRoll()  * DEG_TO_RAD,
                      mpu.getPitch() * DEG_TO_RAD,
                      mpu.getYaw()   * DEG_TO_RAD);

    // ── Ángulos relativos al arranque (todos = 0 al inicio) ───────────────────
    float roll  = (x6[0] - rollOff_r)  * RAD_TO_DEG;
    float pitch = (x6[1] - pitchOff_r) * RAD_TO_DEG;
    float yaw   = (x6[2] - yawOff_r)   * RAD_TO_DEG;
    while (yaw >  180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;

    // ── PID timón de profundidad (setpoint pitch = 0°) ────────────────────────
    float error  = -pitch;
    float p_term = KP * error;
    pidIntegral  = constrain(pidIntegral + error * dt, -I_MAX / KI, I_MAX / KI);
    float i_term = KI * pidIntegral;
    // D-term: tasa de cabeceo del giróscopo con sesgo estimado por EKF
    float d_term = KD * (-(gy_r - x6[4]) * RAD_TO_DEG);
    float output = constrain(p_term + i_term + d_term, -OUT_MAX, OUT_MAX);
    int   angulo = (int)constrain(90.0f + output, 45.0f, 135.0f);
    servoWrite(angulo);

    // ── Envío serial + UDP ────────────────────────────────────────────────────
    send("R",   roll,   1);
    send("P",   pitch,  1);
    send("Y",   yaw,    1);
    send("AX",  acc_x,  3);
    send("AY",  acc_y,  3);
    send("AZ",  acc_z,  3);
    sendInt("SRV", angulo);

    // ── BMP180: lectura cada 150 ms con filtro EMA ────────────────────────────
    static unsigned long tBaro   = 0;
    static bool          altInit = false;
    if (bmpOk && ahora - tBaro >= 150) {
        tBaro = ahora;
        float press  = (float)bmp.readPressure();
        float temp   = bmp.readTemperature();
        float altRaw = 44330.0f * (1.0f - powf(press / 101325.0f, 0.1903f)) - altRef;

        // EMA α=0.2 → constante de tiempo ≈ 0.6 s; primera muestra sin transitorio
        if (!altInit) { altEma = altRaw; altInit = true; }
        else           altEma += 0.2f * (altRaw - altEma);

        float dt_b  = (ahora - tBmpPrev) / 1000.0f;
        float vel_z = (dt_b > 0.01f) ? (altEma - altPrev) / dt_b : 0.0f;
        altPrev  = altEma;
        tBmpPrev = ahora;

        send("Alt", altEma, 2);
        send("Pre", press,  0);
        send("Tmp", temp,   1);
        send("VZ",  vel_z,  3);
    }
    udpFlush();
}
// ..............................................................................
// _______________ FIN CICLO PRINCIPAL DEL PROGRAMA _____________________________
// ..............................................................................