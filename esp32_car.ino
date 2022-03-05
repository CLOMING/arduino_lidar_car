#define CHECK_DISTANCE 500 // 장애물 감지 거리 mm
bool auto_driving_mode = false; // 시작시 수동 제어모드로 시작

int obstacle_distance = 10000;
// 장애물이 있는 경우 좌우측으로 회전하지만 모서리의 경우 좌측 우측을 반복하는 현상이 발생
// 따라서 장애물 감지후 좌회전을 한적이 있는지 우회전을 한적이 있는지 기억을 해 두었다가
// 좌회전 우회전 모두 한적이 있으면 우회전을 하는 것으로 함.
int left_right_flag = 0;

int distance_list[36];

#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

#define MOTOR_POWER 200
#define BUZZER_PIN 5
#include "arduino_car.h"

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// 일반적으로 LCD I2C의 주소는 0x27 혹은 0x3F입니다.
// LCD 출력이 되지 않는 경우 아래의 주소를 0x27과 0x3F로 변경해 보시기 바랍니다.
// 변경후에도 출력이 되지 않을 경우 i2c Scanner로 주소를 확인해 주시기 바랍니다.
LiquidCrystal_I2C lcd(0x27,16,2);
//LiquidCrystal_I2C lcd(0x3F,16,2);

#define BT_NAME "kProject_Car"

void setup()
{
  // LCD 출력
  Wire.begin(18,17);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("BT NAME =");
  lcd.setCursor(0,1);
  lcd.print(BT_NAME);
  
  SerialBT.begin(BT_NAME);
  Serial.begin(115200);
  Serial.println("Booting");
  Serial1.begin(115200, SERIAL_8N1, 23, 2);
  
  pinMode(19, OUTPUT); // 19번 핀을 모터로 활성화
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SHIFT_DATA, OUTPUT);
  pinMode(SHIFT_LATCH, OUTPUT);
  pinMode(SHIFT_CLOCK, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);

  ledcAttachPin(ENA1, 0);
  ledcAttachPin(ENA2, 1);
  ledcAttachPin(ENA3, 2);
  ledcAttachPin(ENA4, 3);
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcSetup(2, 5000, 8);
  ledcSetup(3, 5000, 8);
  stop();
}

void loop()
{
  // 전압 체크 및 경보
  int val = analogRead(36);
  int volt = map(val, 0, 4095, 0, 3300);
  int real_volt = volt * ( ( R1 + R2 ) / R2 );
  //  Serial.println(volt);
  // Serial1.print(real_volt);
  // Serial1.println(" V");

  // 배터리 전압이 설정값 이하이면 알람 울림
  if ( real_volt < VOLT_LIMIT )
  {
    digitalWrite(BUZZER_PIN, HIGH);
  }
  else
  {
    digitalWrite(BUZZER_PIN, LOW);
  }

  if ( Serial1.available() >= 74 )
  {
    int id = Serial1.read();
    if ( id == 20 )
    {
      int cmd = Serial1.read();
      if ( cmd == 6 ) // 라이다 값을 가져온 경우
      {
        uint8_t value[72];
        for ( int i = 0; i < 36; i++)
        {
          value[i*2] = Serial1.read();
          value[i*2+1] = Serial1.read();
        }

        if ( Serial1.read() == 21 )
        {
          SerialBT.write(255);
          SerialBT.write(20);
          for ( int i = 0; i < 36; i++)
          {
            SerialBT.write(value[i*2]);
            SerialBT.write(value[i*2+1]);
            distance_list[i] = ( value[i*2+1] << 8 ) | value[i*2];
          }

          obstacle_distance = 10000;
          obstacle_distance = min(obstacle_distance, int(distance_list[18]));
          obstacle_distance = min(obstacle_distance, int(distance_list[19]));
          obstacle_distance = min(obstacle_distance, int(distance_list[20]));
          //      obstacle_distance = min(obstacle_distance, int(distance_list[21]));
          obstacle_distance = min(obstacle_distance, int(distance_list[17]));
          obstacle_distance = min(obstacle_distance, int(distance_list[16]));
          //      obstacle_distance = min(obstacle_distance, int(distance_list[15]));

          int obstacle_left = 10000;
          int obstacle_right = 10000;

          obstacle_right = min(obstacle_right , int(distance_list[19]));
          obstacle_right = min(obstacle_right , int(distance_list[20]));
          obstacle_right = min(obstacle_right , int(distance_list[21]));

          obstacle_left = min(obstacle_left , int(distance_list[17]));
          obstacle_left = min(obstacle_left , int(distance_list[16]));
          obstacle_left = min(obstacle_left , int(distance_list[15]));

          if ( auto_driving_mode == true )
          {
            if ( obstacle_distance < CHECK_DISTANCE )
            {
              if ( left_right_flag == 0b00000011 ) // 장애물 감지후 좌회전 우회전 모두 해본 경우 우회전으로 결정
              {
                right();
              }
              else
              {
                if ( obstacle_right < obstacle_left )
                {
                  left_right_flag = left_right_flag | 0b00000001; // 좌회전 한적이 있음을 설정
                  left();
                }
                else
                {
                  left_right_flag = left_right_flag | 0b00000010; // 우회전 한적 있음을 설정
                  right();
                }
              }
            }
            else
            {
              left_right_flag = 0; // 좌회전 우회전 한적 없음으로 설정
              front();
            }
          }
        }
      }
    }
  }

  if ( SerialBT.available() )
  {
    int cmd = SerialBT.read();
    if ( cmd == '2' )
    {
      Serial.println("FRONT");
      auto_driving_mode = false;
      front();
    }
    if ( cmd == '4' )
    {
      Serial.println("LEFT");
      auto_driving_mode = false;
      left();
    }
    if ( cmd == '5' )
    {
      Serial.println("STOP");
      auto_driving_mode = false;
      stop();
    }
    if ( cmd == '6' )
    {
      Serial.println("RIGHT");
      auto_driving_mode = false;
      right();
    }
    if ( cmd == '8' )
    {
      Serial.println("BACK");
      auto_driving_mode = false;
      back();
    }
    if ( cmd == '9' )
    {
      Serial.println("FRONT");
      auto_driving_mode = true;
      front();
    }
    if ( cmd == '0' )
    {
      Serial.println("STOP");
      auto_driving_mode = false;
      stop();
    }
    if ( cmd == '3' )
    {
      Serial.println("clean");
      digitalWrite(19,HIGH);
    }
    if ( cmd == '1' )
    {
      Serial.println("clean STOP");
      digitalWrite(19,LOW);
    }
  }
}
