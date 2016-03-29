#define ZERO_SPEED 65535 //零速

uint16_t counter_m[4];        // 计数器周期
uint16_t period_m[4][8];      // 八子时期
uint8_t period_m_index[4];    //索引子时期

uint8_t dir_m[4];             // 步进电机的实际方向
int16_t speed_m[4];           // 电机的实际转速

void fastDigitalWrite(uint8_t pin, bool value) {
  if (value) {
    *portOutputRegister(digitalPinToPort(pin)) |= digitalPinToBitMask(pin);
  }
  else {
    *portOutputRegister(digitalPinToPort(pin)) &= ~digitalPinToBitMask(pin);
  }
}

//在16MHz的200ns=>4条指令
void delay_200ns() {
  __asm__ __volatile__ (
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop");
}

void timerIsr() {
  //我们将覆盖定时器使用步进电机
  //步进电机初始化
  //定时器CTC模式
  // STEPPER MOTORS INITIALIZATION
  // TIMER1 CTC MODE
  TCCR1B &= ~(1 << WGM13);
  TCCR1B |=  (1 << WGM12);
  TCCR1A &= ~(1 << WGM11);
  TCCR1A &= ~(1 << WGM10);
  // output mode = 00 (disconnected)
  TCCR1A &= ~(3 << COM1A0);
  TCCR1A &= ~(3 << COM1B0);
  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
  TCCR1B = (TCCR1B & ~(0x07 << CS10)) | (2 << CS10);

  //OCR1A = 125;  // 16Khz
  //OCR1A = 100;  // 20Khz
  OCR1A = 160;   // 12.5Khz
  TCNT1 = 0;

  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 interrupt
}

ISR(TIMER1_COMPA_vect) {
  for (int a = 0; a < 4; a++) {
    counter_m[a]++;
    if (counter_m[a] >= period_m[a][period_m_index[a]]) {
      counter_m[a] = 0;
      if (period_m[a][0] == ZERO_SPEED)
        return;
      if (dir_m[a])
        fastDigitalWrite(MOTOR_DIR[a], LOW);
      else
        fastDigitalWrite(MOTOR_DIR[a], HIGH);
      //delay_200ns();
      period_m_index[a] = (period_m_index[a] + 1) & 0x07;
      fastDigitalWrite(MOTOR_STEP[a], HIGH);
      delayMicroseconds(1);
      fastDigitalWrite(MOTOR_STEP[a], LOW);
    }
  }
}

void calculateSubperiods(uint8_t motor) {
  int subperiod;
  int absSpeed;
  uint8_t j;

  if (speed_m[motor] == 0) {
    for (j = 0; j < 8; j++) {
      period_m[motor][j] = ZERO_SPEED;
    }
    return;
  }
  if (speed_m[motor] > 0 ) {  // 正速度
    dir_m[motor] = 1;
    absSpeed = speed_m[motor];
  }
  else {                      // 负速度
    dir_m[motor] = 0;
    absSpeed = -speed_m[motor];
  }

  for (j = 0; j < 8; j++) {
    period_m[motor][j] = 1000 / absSpeed;
    // 计算亚期。如果模块<0.25=>子期间= 0，如果模块<0.5=>子周期= 1。如果模块<0.75子期间=2其他子期间=3
  }
  subperiod = ((1000 % absSpeed) * 8) / absSpeed; // 优化代码来计算子期间（整数运算）
  if (subperiod > 0) {
    period_m[motor][1]++;
  }
  if (subperiod > 1) {
    period_m[motor][5]++;
  }
  if (subperiod > 2) {
    period_m[motor][3]++;
  }
  if (subperiod > 3) {
    period_m[motor][7]++;
  }
  if (subperiod > 4) {
    period_m[motor][0]++;
  }
  if (subperiod > 5) {
    period_m[motor][4]++;
  }
  if (subperiod > 6) {
    period_m[motor][2]++;
  }
}

void setMotorSpeed(uint8_t motor, int16_t tspeed) {
  // 我们限制最大加速度
  if ((speed_m[motor] - tspeed) > MAX_ACCEL) {
    speed_m[motor] -= MAX_ACCEL;
  }
  else if ((speed_m[motor] - tspeed) < -MAX_ACCEL) {
    speed_m[motor] += MAX_ACCEL;
  }
  else {
    speed_m[motor] = tspeed;
  }
  
  calculateSubperiods(motor);  //我们采用四个子周期来提高分辨率
  // 为了节省能量，当它没有运行...
  if ((speed_m[0] == 0) && (speed_m[1] == 0) && (speed_m[2] == 0) && (speed_m[3] == 0)) {
    fastDigitalWrite(MOTOR_EN, HIGH);  //禁用电机
  }
  else {
    fastDigitalWrite(MOTOR_EN, LOW);  // 使电机
  }
}
