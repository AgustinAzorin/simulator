#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu; // Dirección por defecto 0x68 (AD0 a GND). Si usás 0x69: MPU6050 mpu(0x69);

// Escalas según rangos elegidos (ver setup)
const float ACC_SCALE = 16384.0f; // LSB por g (±2g)
const float GYR_SCALE = 131.0f;   // LSB por (°/s) (±250 dps)

// Bias (se calculan en una calibración rápida al inicio)
float ax_bias=0, ay_bias=0, az_bias=0;
float gx_bias=0, gy_bias=0, gz_bias=0;

// Yaw integrado (sin magnetómetro deriva con el tiempo)
float yawDeg = 0.0f;
unsigned long prevMicros = 0;

void quickCalibrate(uint16_t N = 200) {
  // Promedia N lecturas asumiendo el sensor quieto y horizontal.
  // Resta ~1g del eje Z para que ACC quede centrado alrededor de 0.
  long sax=0, say=0, saz=0, sgx=0, sgy=0, sgz=0;
  int16_t ax, ay, az, gx, gy, gz;
  for (uint16_t i=0; i<N; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sax += ax; 
    say += ay; 
    saz += (az - 16384); // 16384 LSB ≈ 1g en ±2g
    sgx += gx; sgy += gy; sgz += gz;
    delay(2);
  }
  ax_bias = (float)sax / N;
  ay_bias = (float)say / N;
  az_bias = (float)saz / N;
  gx_bias = (float)sgx / N;
  gy_bias = (float)sgy / N;
  gz_bias = (float)sgz / N;
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);     // I2C rápido (400 kHz)
  Serial.begin(115200);

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("ERROR: MPU6050 no responde (I2C). Verifica cableado y dirección.");
    // Continuamos igual para que se vea algo por Serial en Wokwi
  }

  // Configurar rangos (coinciden con ACC_SCALE y GYR_SCALE)
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);   // ±2g
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);   // ±250 °/s

  // Filtro y rate sugeridos para estabilidad
  mpu.setDLPFMode(3);  // ~44 Hz ACC / ~42 Hz GYRO
  mpu.setRate(4);      // 1kHz/(1+4) = 200 Hz

  // Calibración rápida (sensor quieto)
  quickCalibrate(200);

  prevMicros = micros();
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Quitar bias
  float ax_c = ax - ax_bias;
  float ay_c = ay - ay_bias;
  float az_c = az - az_bias;
  float gx_c = gx - gx_bias;
  float gy_c = gy - gy_bias;
  float gz_c = gz - gz_bias;

  // A unidades físicas
  float ax_g = ax_c / ACC_SCALE;
  float ay_g = ay_c / ACC_SCALE;
  float az_g = az_c / ACC_SCALE;

  float gx_dps = gx_c / GYR_SCALE;
  float gy_dps = gy_c / GYR_SCALE;
  float gz_dps = gz_c / GYR_SCALE;

  // Roll/Pitch desde acelerómetro (grados)
  float rollDeg  = atan2(ay_c, az_c) * 180.0f / PI;
  float pitchDeg = atan2(-ax_c, sqrt((double)ay_c*ay_c + (double)az_c*az_c)) * 180.0f / PI;

  // Yaw integrado desde giroscopio Z (grados) — deriva con el tiempo
  unsigned long now = micros();
  float dt = (now - prevMicros) / 1e6f;
  prevMicros = now;
  yawDeg += gz_dps * dt;

  // Salida por consola
  Serial.print("ACC[g]: ");
  Serial.print(ax_g, 3); Serial.print('\t');
  Serial.print(ay_g, 3); Serial.print('\t');
  Serial.print(az_g, 3); Serial.print(" | ");

  Serial.print("GYRO[dps]: ");
  Serial.print(gx_dps, 4); Serial.print('\t');
  Serial.print(gy_dps, 4); Serial.print('\t');
  Serial.print(gz_dps, 4); Serial.print(" | ");

  Serial.print("RPY[deg]: ");
  Serial.print(rollDeg, 4);  Serial.print('\t');
  Serial.print(pitchDeg, 4); Serial.print('\t');
  Serial.println(yawDeg, 4);

  delay(50); // ~20 Hz de impresión (ajustá a gusto)
}

