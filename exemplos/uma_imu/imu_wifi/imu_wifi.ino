#include "FastIMU.h"
#include <Wire.h>
#include <WiFi.h>

MPU6500 imu;               //Change to the name of any supported IMU! 

// Currently supported IMUS: MPU9255 MPU9250 MPU6886 MPU6500 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL QMI8658

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;

const char* ssid = "NOME_DO_WIFI"; // nome da rede Wifi que a ESP32 vai conectar. Por exemplo, vc pode usar o hotspot de um celular.
const char* password =  "SENHA_DO_WIFI"; // senha do Wifi que a ESP32 vai conectar. Tanto o nome quanto a senha devem estar corretos. Ela conecta automaticamente.
WiFiServer server(8080); 

// Variáveis para dados dos sensores
float accelX1 = 0.0, accelY1 = 0.0, accelZ1 = 0.0;
float gyroX1 = 0.0, gyroY1 = 0.0, gyroZ1 = 0.0;

// Variáveis do Filtro de Kalman (para pitch e roll de ambos os sensores)
float anglePitch1 = 0, angleRoll1 = 0; // Ângulos calculados para IMU 1
float biasPitch1 = 0, biasRoll1 = 0;   // Bias do giroscópio para IMU 1

float P_pitch1[2][2] = {{1, 0}, {0, 1}}; // Covariância para pitch (IMU 1)
float P_roll1[2][2] = {{1, 0}, {0, 1}};  // Covariância para roll (IMU 1)

const float Q_angle = 0.001; // Variância do ruído de processo (ângulo)
const float Q_bias = 0.003;  // Variância do ruído de processo (bias)
const float R_measure = 0.03; // Variância da medição

unsigned long lastTime = 0;
float temposec = 0;

// Função do Filtro de Kalman para processar dados de pitch/roll
float kalmanFilter(float angle, float rate, float accel, float *bias, float P[2][2]) {
    // Previsão
    angle += temposec * (rate - *bias);
    P[0][0] += temposec * (temposec * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= temposec * P[1][1];
    P[1][0] -= temposec * P[1][1];
    P[1][1] += Q_bias * temposec;

    // Atualização
    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = accel - angle;
    angle += K[0] * y;
    *bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}

void setup() {
 Serial.begin(115200);
 WiFi.begin(ssid, password);
 
 while (WiFi.status() != WL_CONNECTED) {
  delay(500);
  Serial.println("Connecting to WiFi..");
 }
 Serial.println("Connected to the WiFi network");
 Serial.println(WiFi.localIP());  // Exibe o IP atribuído à ESP32

 server.begin(); 

 Wire.begin(); // Inicializa a comunicação I2C
  
  // Inicializa a MPU6500
  int err = imu.init(calib);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  Serial.println("MPU6500 inicializada com sucesso.");
}

void loop() {
 WiFiClient client = server.available(); // Verifica se há cliente conectado

 if (client) { // Somente executa se houver cliente conectado
    Serial.println("Cliente conectado.");
    while (client.connected()) {
      imu.update();
      imu.getAccel(&accelData);
      imu.getGyro(&gyroData);

      unsigned long now = millis();
      temposec = (now - lastTime) / 1000.0;
      lastTime = now;

      unsigned long tempo = now;

      // Coleta dados
      accelX1 = accelData.accelY;
      accelY1 = accelData.accelX;
      accelZ1 = accelData.accelZ;
      gyroX1 = gyroData.gyroX;
      gyroY1 = gyroData.gyroY;
      gyroZ1 = gyroData.gyroZ;

      float pitchAccel1 = atan2(accelY1, sqrt(accelX1 * accelX1 + accelZ1 * accelZ1)) * 180 / PI;
      float rollAccel1 = atan2(-accelX1, accelZ1) * 180 / PI;

      anglePitch1 = kalmanFilter(anglePitch1, gyroX1, pitchAccel1, &biasPitch1, P_pitch1);
      angleRoll1 = kalmanFilter(angleRoll1, gyroY1, rollAccel1, &biasRoll1, P_roll1);
      angleYaw1 += gyroZ1 * temposec * 180.0 / PI;

      String dadosimu = String(anglePitch1, 2) + "," + String(angleRoll1, 2) + "," + String(angleYaw1, 2) + "," + String(accelX1) + "," 
                        + String(accelY1) + "," + String(accelZ1) + "," + String(gyroX1) + "," 
                        + String(gyroY1) + "," + String(gyroZ1) + "," + String(tempo) + "\n";
      client.print(dadosimu); // Envia os dados formatados
      //delay(5);
    }
    client.stop(); // Finaliza a conexão com o cliente
    Serial.println("Cliente desconectado.");
  }
}
