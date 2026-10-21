#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>

// Pinos I2C usados para o MPU9250 neste ESP32-S3.
constexpr uint8_t I2C_SDA = 14;
constexpr uint8_t I2C_SCL = 13;
constexpr uint8_t MPU_ADDRESS = 0x68;  // use 0x69 se o sensor não responder
constexpr uint32_t SERIAL_BAUD = 115200;

MPU9250 mpu;
uint32_t ultimoDiagnosticoMs = 0;

void setup() {
  // A USB CDC do ESP32-S3 aparece como COM3 no computador (conforme ini).
  Serial.begin(SERIAL_BAUD);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  if (!mpu.setup(MPU_ADDRESS)) {
    // A mensagem começa com # para que o programa Python possa ignorá-la.
    Serial.println("# ERRO: MPU9250 nao encontrado");
    while (true) delay(1000);
  }

  // O Madgwick desta biblioteca interrompe a atualização quando a amostra do
  // magnetômetro não está pronta. Mahony mantém roll/pitch com acc+gyro;
  // o yaw continuará respondendo ao giro, embora possa derivar com o tempo.
  mpu.selectFilter(QuatFilterSel::MAHONY);
  Serial.println("# MPU9250 pronto");
}

void loop() {
  if (mpu.update()) {
    // Protocolo simples para o Python: IMU,roll,pitch,yaw (em graus).
    // Não envie mensagens extras sem começar por #, pois o Python interpreta
    // as linhas que começam com IMU como leituras do sensor.
    // Uma única linha por amostra, compatível com ler_imu() no Python.
    Serial.printf("IMU,%.3f,%.3f,%.3f\n", mpu.getRoll(), mpu.getPitch(),
                  mpu.getYaw());

    // Diagnóstico a 1 Hz: valores físicos antes do filtro de orientação.
    // Linhas iniciadas por # são ignoradas pelo Python/Teleplot.
    if (millis() - ultimoDiagnosticoMs >= 1000) {
      Serial.printf("# RAW acc=%.3f,%.3f,%.3f g | gyro=%.3f,%.3f,%.3f dps | mag=%.1f,%.1f,%.1f mG\n",
                    mpu.getAcc(0), mpu.getAcc(1), mpu.getAcc(2),
                    mpu.getGyro(0), mpu.getGyro(1), mpu.getGyro(2),
                    mpu.getMag(0), mpu.getMag(1), mpu.getMag(2));
      ultimoDiagnosticoMs = millis();
    }
  }

  delay(20);  // amostragem aproximada de 50 Hz
}
