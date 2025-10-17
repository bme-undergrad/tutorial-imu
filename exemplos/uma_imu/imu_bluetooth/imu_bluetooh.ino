#include "FastIMU.h"
#include <Wire.h>
#include "BluetoothSerial.h"

// ===== Checagem de suporte BT =====
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth não está habilitado no core do ESP32. Habilite 'Bluetooth' em Tools -> PSRAM/BT settings."
#endif

// ===== IMU =====
MPU6500 imu;               // Troque se usar outro sensor suportado
calData   calib = { 0 };
AccelData accelData;
GyroData  gyroData;
MagData   magData;

// ===== Variáveis de estado / Kalman =====
float accelX1=0, accelY1=0, accelZ1=0;
float gyroX1=0,  gyroY1=0,  gyroZ1=0;

float anglePitch1 = 0, angleRoll1 = 0, angleYaw1 = 0; // yaw adicionado
float biasPitch1  = 0, biasRoll1  = 0;

float P_pitch1[2][2] = {{1,0},{0,1}};
float P_roll1 [2][2] = {{1,0},{0,1}};

const float Q_angle   = 0.001f;
const float Q_bias    = 0.003f;
const float R_measure = 0.03f;

unsigned long lastTime = 0;
float temposec = 0;

// ===== Bluetooth Clássico (SPP) =====
BluetoothSerial SerialBT;
static const char* BT_NAME = "ESP32_IMU_SPP";
// Opcional: defina um PIN de pareamento (legacy). Comente se não quiser PIN.
// Obs.: chame setPin() ANTES de begin().
// static const char* BT_PIN  = "1234";  // 4 a 16 dígitos

// ===== Kalman =====
float kalmanFilter(float angle, float rate, float accel, float *bias, float P[2][2]) {
  // Previsão
  angle += temposec * (rate - *bias);
  P[0][0] += temposec * (temposec * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= temposec * P[1][1];
  P[1][0] -= temposec * P[1][1];
  P[1][1] += Q_bias * temposec;

  // Atualização
  float S  = P[0][0] + R_measure;
  float K0 = P[0][0] / S;
  float K1 = P[1][0] / S;

  float y = accel - angle;
  angle  += K0 * y;
  *bias  += K1 * y;

  float P00 = P[0][0];
  float P01 = P[0][1];

  P[0][0] -= K0 * P00;
  P[0][1] -= K0 * P01;
  P[1][0] -= K1 * P00;
  P[1][1] -= K1 * P01;

  return angle;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // ===== IMU =====
  int err = imu.init(calib);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) { delay(10); }
  }
  Serial.println("MPU6500 inicializada com sucesso.");

  // ===== Bluetooth Clássico (SPP) =====
  // Se quiser PIN (pareamento legado), descomente a linha abaixo:
  SerialBT.setPin(BT_PIN);         // comente esta linha para pareamento sem PIN
  // Habilita SSP (Secure Simple Pairing) — opcional; pode manter descomentado.
  SerialBT.enableSSP();

  // Inicia como servidor SPP com o nome desejado
  if (!SerialBT.begin(BT_NAME)) {
    Serial.println("Falha ao iniciar Bluetooth SPP");
    while (true) { delay(10); }
  }
  Serial.println("Bluetooth SPP iniciado. Pareie e conecte-se ao dispositivo para receber dados.");

  lastTime = millis();
}

void loop() {
  // Envie somente quando houver cliente conectado
  if (!SerialBT.hasClient()) {
    delay(50);
    return;
  }

  // ===== Leitura IMU =====
  imu.update();
  imu.getAccel(&accelData);
  imu.getGyro(&gyroData);

  unsigned long now = millis();
  temposec = (now - lastTime) / 1000.0f;
  if (temposec <= 0) temposec = 0.001f;
  lastTime = now;

  unsigned long tempo = now;

  // Mapeamentos conforme seu código original
  accelX1 = accelData.accelY;
  accelY1 = accelData.accelX;
  accelZ1 = accelData.accelZ;
  gyroX1  = gyroData.gyroX;
  gyroY1  = gyroData.gyroY;
  gyroZ1  = gyroData.gyroZ;

  float pitchAccel1 = atan2(accelY1, sqrt(accelX1*accelX1 + accelZ1*accelZ1)) * 180.0f / PI;
  float rollAccel1  = atan2(-accelX1, accelZ1) * 180.0f / PI;

  anglePitch1 = kalmanFilter(anglePitch1, gyroX1, pitchAccel1, &biasPitch1, P_pitch1);
  angleRoll1  = kalmanFilter(angleRoll1,  gyroY1, rollAccel1,  &biasRoll1,  P_roll1);
  angleYaw1  += gyroZ1 * temposec * 180.0f / PI;

  // CSV: pitch,roll,yaw,ax,ay,az,gx,gy,gz,tempo_ms
  String dadosimu = String(anglePitch1, 2) + "," + String(angleRoll1, 2) + "," + String(angleYaw1, 2) + "," +
                    String(accelX1, 6) + "," + String(accelY1, 6) + "," + String(accelZ1, 6) + "," +
                    String(gyroX1, 6)  + "," + String(gyroY1, 6)  + "," + String(gyroZ1, 6)  + "," +
                    String(tempo) + "\n";

  // Envia pela porta serial Bluetooth
  SerialBT.print(dadosimu);

  // Intervalo pequeno para não saturar o link
  delay(5);
}
