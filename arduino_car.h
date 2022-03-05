// 2MB APP / 2MB SPIFFS
#define SHIFT_DATA 14
#define SHIFT_LATCH 25
#define SHIFT_CLOCK 3
// 시프트 레지스터 사용을 위한 data 핀, latch 핀, clock핀
#define ENA1 33
#define ENA2 27
#define ENA3 26
#define ENA4 13
// ENA는 4개의 모터에 할당되어있는 핀
#define BAT_READ_PIN 36 // 배터리 전압 체크 핀
#define R1 2200.0       // 분압 저항 값
#define R2 1000.0       // 분압 저항 값
#define VOLT_LIMIT 7000 // 저전압 경보 값
#define BUZZER_PIN 5
uint8_t motor_bit = 0b00000000;

void m1(int value) // m1은 좌측 상단의 모터 하나를 제어하기 위한 함수, value는 모터 속도
{
  if (value > 0)
  {
    motor_bit = motor_bit & 0b00111111;
    motor_bit = motor_bit | 0b10000000;
    ledcWrite(0, value); // 모터 하나 당 디지털 핀 2개와 아날로그 핀 1개가 존재하며 현재 코드는 아날로그 핀에 대한 설정
  }
  else
  {
    motor_bit = motor_bit & 0b00111111;
    motor_bit = motor_bit | 0b01000000;
    ledcWrite(0, -value);
  }
  // motor_bit의 8자리는 2자리씩 끊어서 모터 4개의 디지털핀에 할당 되어있으며 비트 연산은 구동되고 있는 모터들 중 m1 모터만 제어하기 위함
  digitalWrite(SHIFT_LATCH, LOW);
  shiftOut(SHIFT_DATA, SHIFT_CLOCK, LSBFIRST, motor_bit); // 시프트레지스터 사용을 쉽게하기 위해 shiftOut 함수를 사용
                                                          // 이 함수를 사용하므로 코드에서 motor_bit 값만큼 모터를 설정한 내용이 데이터핀에 전달되고 클락핀에서 전달여부가 확인됨
  digitalWrite(SHIFT_LATCH, HIGH);                        // 이후 latch 핀에 출력을 주면 설정된 내용만큼 모터 작동
}

void m2(int value)
{
  if (value > 0)
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
  if (value > 0)
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
  if (value > 0)
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

void stop() // 이후 정의한 m1~m4 모터 제어 함수를 이용하여 방향 제어 함수를 설정
{
  m1(0);
  m2(0);
  m3(0);
  m4(0);
}

void front_left()
{
  m1(MOTOR_POWER / 2);
  m2(MOTOR_POWER);
  m3(MOTOR_POWER / 2);
  m4(MOTOR_POWER);
}

void front_right()
{
  m1(MOTOR_POWER);
  m2(MOTOR_POWER / 2);
  m3(MOTOR_POWER);
  m4(MOTOR_POWER / 2);
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
