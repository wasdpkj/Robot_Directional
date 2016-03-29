#include "userDef.h"
#include "motor.h"

#include <Microduino_Protocol_HardSer.h>
Protocol ProtocolB(&Serial1, TYPE_NUM);

uint32_t safe_ms = 0;

double motor[3];
uint16_t channal_data[8]; //8通道数据

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_EN, OUTPUT);   // ENABLE MOTORS
  digitalWrite(MOTOR_EN, HIGH);  // Disbale motors
  for (uint8_t a = 0; a < 3; a++) {
    pinMode(MOTOR_DIR[a], OUTPUT);  // DIR MOTOR
    pinMode(MOTOR_STEP[a], OUTPUT);  // STEP MOTOR
  }
  timerIsr(); //初始化步进电机定时器
  Serial.println("Initializing Stepper motors...");
  digitalWrite(MOTOR_EN, LOW);   // 使步进驱动器
  // 小车转动，表明机器人已准备就绪
  setMotorSpeed(0, 512);
  setMotorSpeed(1, 512);
  setMotorSpeed(2, 512);
  delay(2000);
  setMotorSpeed(0, 0);
  setMotorSpeed(1, 0);
  setMotorSpeed(2, 0);
  delay(500);
  setMotorSpeed(0, -512);
  setMotorSpeed(1, -512);
  setMotorSpeed(2, -512);
  delay(2000);
  setMotorSpeed(0, 0);
  setMotorSpeed(1, 0);
  setMotorSpeed(2, 0);

  ProtocolB.begin(BLE_SPEED);  //9600/19200/38400
}

// 主循环
void loop() {
  int sta = ProtocolB.parse(channal_data, MODE_LOOP);
  if (sta != P_NONE) {
    switch (sta) {
      case P_FINE: {
          Serial.println(" \t DATA OK");
          safe_ms = millis();

          int16_t x = map(channal_data[CHANNEL_THROTTLE - 1], 1000, 2000, -MAX_THROTTLE, MAX_THROTTLE);
          int16_t y = map(channal_data[CHANNEL_STEERING - 1], 2000, 1000, -MAX_THROTTLE, MAX_THROTTLE);
          int16_t r = map(channal_data[CHANNEL_ROTATE - 1], 2000, 1000, -MAX_STEERING, MAX_STEERING);

          if (x != 0 && y != 0) {
            motor[0] = V(x, y) * COS(x, y);
            motor[1] = -V(x, y) * (0.5 * sqrt(3) * SIN(x, y) + 0.5 * COS(x, y));
            motor[2] = V(x, y) * (0.5 * sqrt(3) * SIN(x, y) - 0.5 * COS(x, y));
          }
          else {
            motor[0] = 0;
            motor[1] = 0;
            motor[2] = 0;
          }

          for (uint8_t a = 0; a < 3; a++) {
            motor[a] += r;
            setMotorSpeed(a, motor[a]);
          }

#ifdef _DEBUG
          Serial.print(x);
          Serial.print(",");
          Serial.print(y);
          Serial.print("\t");
          Serial.print(motor[0]);
          Serial.print(",");
          Serial.print(motor[1]);
          Serial.print(",");
          Serial.println(motor[2]);
#endif
        }
        break;
      case P_ERROR:
        Serial.println(" \t DATA ERROR");
        break;
      case P_TIMEOUT:
        Serial.println(" \t DATA TIMEOUT");
        break;
    }
  }

  if (safe_ms > millis()) safe_ms = millis();
  if (millis() - safe_ms > SAFE_TIME_OUT) {
    for (uint8_t a = 0; a < 3; a++) {
      motor[a] = 0;
      setMotorSpeed(a, motor[a]);
    }
  }
}
