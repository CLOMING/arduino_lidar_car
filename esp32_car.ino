#define CHECK_DISTANCE 500      // 장애물 감지 거리 mm
bool auto_driving_mode = false; // 시작시 수동 제어모드로 시작

int obstacle_distance = 10000;
// 장애물이 있는 경우 좌우측으로 회전하지만 모서리의 경우 좌측 우측을 반복하는 현상이 발생
// 따라서 장애물 감지후 좌회전을 한적이 있는지 우회전을 한적이 있는지 기억을 해 두었다가
// 좌회전 우회전 모두 한적이 있으면 우회전을 하는 것으로 함.
int left_right_flag = 0; // 방향을 어디로 틀었는지에 대한 정보를 저장하는 변수 선언

int distance_list[36]; // 36개의 측정된 거리 데이터를 가지는 리스트 선언

#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

#define MOTOR_POWER 200 // 모터 속도
#define BUZZER_PIN 5    // 주동부저와 연결된 핀
#include "arduino_car.h"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// 일반적으로 LCD I2C의 주소는 0x27 혹은 0x3F
// LCD 출력이 되지 않는 경우 아래의 주소를 0x27과 0x3F로 변경
// 변경후에도 출력이 되지 않을 경우 i2c Scanner로 주소를 확인
LiquidCrystal_I2C lcd(0x27, 16, 2);
// LiquidCrystal_I2C lcd(0x3F,16,2);

#define BT_NAME "kProject_Car"

void setup()
{
  // LCD 출력
  Wire.begin(18, 17);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("BT NAME =");
  lcd.setCursor(0, 1);
  lcd.print(BT_NAME);

  // 블루투스 시리얼
  SerialBT.begin(BT_NAME);
  Serial.begin(115200);
  Serial.println("Booting");
  Serial1.begin(115200, SERIAL_8N1, 23, 2);

  pinMode(19, OUTPUT); // 19번 핀을 모터로 활성화(청소 기능)
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SHIFT_DATA, OUTPUT);
  pinMode(SHIFT_LATCH, OUTPUT);
  pinMode(SHIFT_CLOCK, OUTPUT);
  // 시프트 레지스터 사용을 위한 data, latch, clock 핀을 출력모드로 설정

  digitalWrite(BUZZER_PIN, LOW);

  ledcAttachPin(ENA1, 0);
  ledcAttachPin(ENA2, 1);
  ledcAttachPin(ENA3, 2);
  ledcAttachPin(ENA4, 3);
  // ENA1~4는 모터에 연결된 핀이며 esp32에서 pwm을 위해 ledcWrite을 이용
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcSetup(2, 5000, 8);
  ledcSetup(3, 5000, 8);
  stop(); // 모터는 정지상태로 시작
}

void loop()
{
  // 전압 체크 및 경보
  int val = analogRead(36);
  int volt = map(val, 0, 4095, 0, 3300);
  int real_volt = volt * ((R1 + R2) / R2);

  // 배터리 전압이 설정값 이하이면 알람 울림
  if (real_volt < VOLT_LIMIT)
  {
    digitalWrite(BUZZER_PIN, HIGH);
  }
  else
  {
    digitalWrite(BUZZER_PIN, LOW);
  }

  if (Serial1.available() >= 74) // 시리얼 버퍼에 저장된 데이터가 74바이트일 때
  {
    int id = Serial1.read(); // 데이터 수신
    if (id == 20)
    {
      int cmd = Serial1.read();
      if (cmd == 6) // 라이다 값을 가져온 경우
      {
        uint8_t value[72];
        for (int i = 0; i < 36; i++)
        {
          value[i * 2] = Serial1.read();
          value[i * 2 + 1] = Serial1.read();
        }
        // 라이다에서 측정된 36개의 거리데이터는 2바이트이며 시리얼통신 전송은 1바이트만 가능하므로 남은 1바이트를 마저 수신하기 위해 두번에 나눔
        if (Serial1.read() == 21)
        {
          SerialBT.write(255);
          SerialBT.write(20);
          for (int i = 0; i < 36; i++)
          {
            SerialBT.write(value[i * 2]);
            SerialBT.write(value[i * 2 + 1]);
            distance_list[i] = (value[i * 2 + 1] << 8) | value[i * 2];
          }
          // value 리스트에 저장된 데이터를 역시 두번에 걸쳐서 블루투스 전송하고 빅엔디안 표준 통신에 의해 거꾸로 조합된 2바이트 데이터를 다시 보정하여 distance_list에 넣음
          obstacle_distance = 10000;
          obstacle_distance = min(obstacle_distance, int(distance_list[18]));
          obstacle_distance = min(obstacle_distance, int(distance_list[19]));
          obstacle_distance = min(obstacle_distance, int(distance_list[20]));
          obstacle_distance = min(obstacle_distance, int(distance_list[17]));
          obstacle_distance = min(obstacle_distance, int(distance_list[16]));

          int obstacle_left = 10000;
          int obstacle_right = 10000;

          obstacle_right = min(obstacle_right, int(distance_list[19]));
          obstacle_right = min(obstacle_right, int(distance_list[20]));
          obstacle_right = min(obstacle_right, int(distance_list[21]));

          obstacle_left = min(obstacle_left, int(distance_list[17]));
          obstacle_left = min(obstacle_left, int(distance_list[16]));
          obstacle_left = min(obstacle_left, int(distance_list[15]));
          // obstacle_distance는 전방 거리 계산에 사용될 데이터(라이다 기준 160도~200도에서 측정된 데이타 중 최소값)
          // obstacle_left 는 좌측 거리 계산에 사용될 데이터
          // obstacle_rigt 는 우측 거리 계산에 사용될 데이터

          if (auto_driving_mode == true) // 자율주행 모드일 때
          {
            if (obstacle_distance < CHECK_DISTANCE) // 전방 측정 거리가 기준 값 미만일 때
            {
              if (left_right_flag == 0b00000011) // 장애물 감지 후 좌회전 우회전 모두 했는데도 여전히 기준거리 값 이하일 때(모서리에 빠진 경우)
              {
                right(); // 모서리를 탈출하여 다시 기준거리 이상이 될 때까지 우측으로 회전
              }
              else
              {
                if (obstacle_right < obstacle_left) // 좌측 거리가 우측 거리보다 클 때
                {
                  left_right_flag = left_right_flag | 0b00000001; // 좌회전 한적이 있음을 설정
                  left();                                         // 좌회전
                }
                else
                {
                  left_right_flag = left_right_flag | 0b00000010; // 우회전 한적 있음을 설정
                  right();                                        // 우회전
                }
              }
            }
            else // 전방 측정 거리가 기준 값 이상일 때
            {
              left_right_flag = 0; // 초기화(좌회전 우회전 한적 없음으로 설정)
              front();             //직진
            }
          }
        }
      }
    }
  }

  if (SerialBT.available()) // 블루투스 수신 시 수동제어모드로 설정하고 앱에서 보낸 숫자에 따라 진행방향 및 청소기능 설정
  {
    int cmd = SerialBT.read();
    if (cmd == '2')
    {
      Serial.println("FRONT");
      auto_driving_mode = false;
      front();
    }
    if (cmd == '4')
    {
      Serial.println("LEFT");
      auto_driving_mode = false;
      left();
    }
    if (cmd == '5')
    {
      Serial.println("STOP");
      auto_driving_mode = false;
      stop();
    }
    if (cmd == '6')
    {
      Serial.println("RIGHT");
      auto_driving_mode = false;
      right();
    }
    if (cmd == '8')
    {
      Serial.println("BACK");
      auto_driving_mode = false;
      back();
    }
    if (cmd == '9')
    {
      Serial.println("FRONT");
      auto_driving_mode = true;
      front();
    }
    if (cmd == '0')
    {
      Serial.println("STOP");
      auto_driving_mode = false;
      stop();
    }
    if (cmd == '3')
    {
      Serial.println("clean");
      digitalWrite(19, HIGH);
    }
    if (cmd == '1')
    {
      Serial.println("clean STOP");
      digitalWrite(19, LOW);
    }
  }
}
