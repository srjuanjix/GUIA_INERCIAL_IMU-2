#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

// Dirección I2C del MPU6050
const int MPU_ADDR = 0x68;

// Variables para el PID
double Consigna = 0.0;  // El ángulo objetivo (0 grados = avión nivelado)
double Entrada = 0.0;   // El ángulo actual medido por el MPU6050
double Salida = 0.0;    // La corrección que calculada por el PID

// --- SINTONIZACIÓN DEL PID (Ajusta estos valores en tus pruebas) ---
double Kp = 2.0;   // Ganancia Proporcional (fuerza inmediata de respuesta)
double Ki = 0.5;   // Ganancia Integral (corrige errores acumulados en el tiempo)
double Kd = 1.0;   // Ganancia Derivativa (amortigua el balanceo, evita sobreoscilaciones)

// Instancia del PID
// DIRECT significa que si la entrada sube, la salida también sube.
// Si tu timón se mueve al revés, cámbialo a REVERSE.
PID miPID(&Entrada, &Salida, &Consigna, Kp, Ki, Kd, DIRECT);

Servo servoTimon;
int16_t AcX, AcY, AcZ;

void setup() {
  Serial.begin(9600);
  servoTimon.attach(9);

  // Inicializar MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Configurar el PID
  miPID.SetMode(AUTOMATIC);

  // Limitar la salida del PID. El servo se moverá en su rango seguro alrededor del centro (90º)
  // Permite corregir hasta 45 grados hacia arriba o hacia abajo.
  miPID.SetOutputLimits(-45, 45);
}

void loop() {
  // --- LECTURA DEL SENSOR ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.write(0);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  // Calcular el ángulo de inclinación (Pitch) usando el acelerómetro.
  // Nota: Aunque mencionas el eje Z, en aviación y en el MPU6050, el cabeceo (pitch)
  // se suele calcular combinando los ejes X e Y mediante trigonometría.
  Entrada = atan2(AcY, sqrt(pow(AcX, 2) + pow(AcZ, 2))) * 180 / M_PI;

  // --- CÁLCULO DEL PID ---
  miPID.Compute(); // Calcula la 'Salida' necesaria basándose en la 'Entrada'

  // --- CONTROL DEL SERVO ---
  // El centro del servo es 90 grados (timón neutro).
  // Sumamos o restamos la salida del PID para corregir la posición.
  int posicionServo = 90 + Salida;

  servoTimon.write(posicionServo);

  // Monitoreo
  Serial.print("Ángulo Actual: "); Serial.print(Entrada);
  Serial.print(" | Ajuste PID: "); Serial.print(Salida);
  Serial.print(" | Servo Pos: "); Serial.println(posicionServo);

  delay(10); // El lazo corre rápido para que el PID reaccione a tiempo
}
