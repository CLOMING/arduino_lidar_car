// ESP32 사용 코드입니다.
// 2MB APP / 2MB SPIFFS
// 모터드라이버 실드 세팅
#define SHIFT_DATA 14
#define SHIFT_LATCH 25
#define SHIFT_CLOCK 32
#define ENA1 33
#define ENA2 27
#define ENA3 26
#define ENA4 13
#define BAT_READ_PIN 36
#define R1 2200.0 // 분압 저항 값
#define R2 1000.0 // 분압 저항 값
#define VOLT_LIMIT 7000 // 저전압 경보 값
#define BUZZER_PIN 5
uint8_t motor_bit = 0b00000000;


void m1(int value)
{
  if ( value > 0 )
  {
    motor_bit = motor_bit & 0b00111111;
    motor_bit = motor_bit | 0b10000000;
    ledcWrite(0, value);
  }
  else
  {
    motor_bit = motor_bit & 0b00111111;
    motor_bit = motor_bit | 0b01000000;
    ledcWrite(0, -value);
  }
  digitalWrite(SHIFT_LATCH, LOW);
  shiftOut(SHIFT_DATA, SHIFT_CLOCK, LSBFIRST, motor_bit);
  digitalWrite(SHIFT_LATCH, HIGH);
}

void m2(int value)
{
  if ( value > 0 )
  {
    motor_bit = motor_bit & 0b11110011;
    motor_bit = motor_bit | 0b00001000;
    ledcWrite(1, value);
  }
  else
  {
    motor_bit = motor_bit & 0b11110011;
    motor_bit = motor_bit | 0b00000100;
    ledcWrite(1, -value);
  }
  digitalWrite(SHIFT_LATCH, LOW);
  shiftOut(SHIFT_DATA, SHIFT_CLOCK, LSBFIRST, motor_bit);
  digitalWrite(SHIFT_LATCH, HIGH);
}

void m3(int value)
{
  if ( value > 0 )
  {
    motor_bit = motor_bit & 0b11001111;
    motor_bit = motor_bit | 0b00100000;
    ledcWrite(2, value);
  }
  else
  {
    motor_bit = motor_bit & 0b11001111;
    motor_bit = motor_bit | 0b00010000;
    ledcWrite(2, -value);
  }
  digitalWrite(SHIFT_LATCH, LOW);
  shiftOut(SHIFT_DATA, SHIFT_CLOCK, LSBFIRST, motor_bit);
  digitalWrite(SHIFT_LATCH, HIGH);
}

void m4(int value)
{
  if ( value > 0 )
  {
    motor_bit = motor_bit & 0b11111100;
    motor_bit = motor_bit | 0b00000010;
    ledcWrite(3, value);
  }
  else
  {
    motor_bit = motor_bit & 0b11111100;
    motor_bit = motor_bit | 0b00000001;
    ledcWrite(3, -value);
  }
  digitalWrite(SHIFT_LATCH, LOW);
  shiftOut(SHIFT_DATA, SHIFT_CLOCK, LSBFIRST, motor_bit);
  digitalWrite(SHIFT_LATCH, HIGH);
}

void stop()
{
  m1(0);
  m2(0);
  m3(0);
  m4(0);
}

void front_left()
{
  m1(MOTOR_POWER/2);
  m2(MOTOR_POWER);
  m3(MOTOR_POWER/2);
  m4(MOTOR_POWER);
}

void front_right()
{
  m1(MOTOR_POWER);
  m2(MOTOR_POWER/2);
  m3(MOTOR_POWER);
  m4(MOTOR_POWER/2);
}

void front()
{
  m1(MOTOR_POWER);
  m2(MOTOR_POWER);
  m3(MOTOR_POWER);
  m4(MOTOR_POWER);
}

void back()
{
  m1(-MOTOR_POWER);
  m2(-MOTOR_POWER);
  m3(-MOTOR_POWER);
  m4(-MOTOR_POWER);
}

void left()
{
  m1(-MOTOR_POWER);
  m2(MOTOR_POWER);
  m3(-MOTOR_POWER);
  m4(MOTOR_POWER);
}

void right()
{
  m1(MOTOR_POWER);
  m2(-MOTOR_POWER);
  m3(MOTOR_POWER);
  m4(-MOTOR_POWER);
}
