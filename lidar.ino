// LIDAR PIN
#define M_EN 14
#define DEV_EN 2
#define M_SCTR 13

#define CHECK_PERIOD 100
#define SEND_PERIOD 200
// 거리 데이터 측정주기와 거리 데이터 전송주기 설정

#include "YDLidar.h"

float angle_distance[360]; // 360도의 거리 데이터가 저장될 리스트 선언
float distance_list[36];   // 360도의 데이터를 10도 단위로 축약하여 36개의 거리데이터를 저장할 리스트 선언

YDLidar lidar; // 라이다 제조회사에서 제공하는 라이다 클래스를 선언
bool isScanning = false;
unsigned long prev_millis;      // 거리데이터를 측정할 주기를 정의할 변수 선언
unsigned long prev_send_millis; // 거리데이터를 전송할 주기를 정의할 변수 선언

void setup()
{
  pinMode(M_EN, OUTPUT);
  digitalWrite(M_EN, HIGH);
  pinMode(DEV_EN, OUTPUT);
  digitalWrite(DEV_EN, HIGH);

  ledcAttachPin(M_SCTR, 1);
  ledcSetup(1, 12000, 8);
  ledcWrite(1, 150); // 0~255 값이 작을 수록 수록 라이다가 빨리 회전함
                     // 라이다 구동 설정

  Serial.begin(115200); // 시리얼 모니터와의 통신
  Serial.println("LIDAR START");
  Serial2.begin(128000, SERIAL_8N1, 16, 15);
  Serial2.setRxBufferSize(2048);
  lidar.begin(Serial2); // 라이다와는 Serial2 와 통신
  Serial.println("#####START#####");
  isScanning = false;
  prev_millis = millis();
}

void loop()
{
  if (isScanning) // iscanning이 true 일 때
  {
    if ((millis() - prev_millis) > CHECK_PERIOD) // 방금 전에 millis로 맞춰놨던 prev_millis 가 어느새 시간이 흘러 millis 값과 1초이상 차이가 나면 거리데이터를 새로 측정
    {
      int ct = 0;
      for (int i = 0; i < 36; i++)
      {
        int min_value = 30000;
        for (int j = 0; j < 10; j++)
        {
          if (angle_distance[i * 10 + j] == 0)
            continue;
          if (min_value > angle_distance[i * 10 + j])
          {
            min_value = angle_distance[i * 10 + j]; // i에 대해 10도 단위 마다 가장 최솟값을 가지는 angle_distance 값을 min_value로 정의
          }
        }
        distance_list[i] = min_value;
      }
      prev_millis = millis(); // 시간 초기화
    }

    if ((millis() - prev_send_millis) > SEND_PERIOD) // 방금 전에 millis로 맞춰놨던 prev_millis 가 어느새 시간이 흘러 millis 값과 2초이상 차이가 나면 바로 위에서 측정했던 거리데이터를 전송
    {
      Serial.write(20);
      Serial.write(6);
      for (int i = 0; i < 36; i++)
      {
        int value = distance_list[i]; // 0~360도의 데이터를 10도 단위로 변환 시킨 36개의 데이터 값을 순서대로 value 값으로 정의
        Serial.write(value);
        Serial.write(value >> 8); // 2바이트 거리데이터를 한번 보낼 때 1바이트 밖에 못보내므로 두번에 나눠서 전송
                                  // 거꾸로 보내는 이유는 빅엔디안 표준 통신법 때문이며 나중에 esp32_car 코드에서 다시 거꾸로 조합
      }
      Serial.write(21);
      prev_send_millis = millis(); // 시간 초기화
    }
    while (Serial2.available())
    {
      if (lidar.waitScanDot() == RESULT_OK) // 결과 읽기가 성공한 경우
      {
        float distance = lidar.getCurrentScanPoint().distance; // distance value in mm unit
        float angle = lidar.getCurrentScanPoint().angle;       // anglue value in degree
        byte quality = lidar.getCurrentScanPoint().quality;    // quality of the current measurement
        bool startBit = lidar.getCurrentScanPoint().startBit;
        int angle_index = angle;
        angle_distance[angle_index] = distance; // 최종적으로 360도의 거리데이터를 저장하는 angle_distance 리스트 생성
      }
      else
      {
      }
    }
  }
  else // iscanning이 false 일 때
  {
    restartScan(); // 다시 스캔
  }
}

void restartScan()
{
  device_info deviceinfo;
  if (lidar.getDeviceInfo(deviceinfo, 100) == RESULT_OK)
  {
    int _samp_rate = 4;
    String model;
    float freq = 7.0f;
    switch (deviceinfo.model)
    {
    case 1:
      model = "F4";
      _samp_rate = 4;
      freq = 7.0;
      break;
    case 4:
      model = "S4";
      _samp_rate = 4;
      freq = 7.0;
      break;
    case 5:
      model = "G4";
      _samp_rate = 9;
      freq = 7.0;
      break;
    case 6:
      model = "X4";
      _samp_rate = 5;
      freq = 7.0;
      break;
    default:
      model = "Unknown";
    }

    uint16_t maxv = (uint16_t)(deviceinfo.firmware_version >> 8);
    uint16_t midv = (uint16_t)(deviceinfo.firmware_version & 0xff) / 10;
    uint16_t minv = (uint16_t)(deviceinfo.firmware_version & 0xff) % 10;
    if (midv == 0)
    {
      midv = minv;
      minv = 0;
    }

    Serial.print("Firmware version:");
    Serial.print(maxv, DEC);
    Serial.print(".");
    Serial.print(midv, DEC);
    Serial.print(".");
    Serial.println(minv, DEC);

    Serial.print("Hardware version:");
    Serial.println((uint16_t)deviceinfo.hardware_version, DEC);

    Serial.print("Model:");
    Serial.println(model);

    Serial.print("Serial:");
    for (int i = 0; i < 16; i++)
    {
      Serial.print(deviceinfo.serialnum[i] & 0xff, DEC);
    }
    Serial.println("");

    Serial.print("[YDLIDAR INFO] Current Sampling Rate:");
    Serial.print(_samp_rate, DEC);
    Serial.println("K");

    Serial.print("[YDLIDAR INFO] Current Scan Frequency:");
    Serial.print(freq, DEC);
    Serial.println("Hz");
    delay(100);
    device_health healthinfo;
    if (lidar.getHealth(healthinfo, 100) == RESULT_OK)
    {
      // detected...
      Serial.print("[YDLIDAR INFO] YDLIDAR running correctly! The health status:");
      Serial.println(healthinfo.status == 0 ? "well" : "bad");
      if (lidar.startScan() == RESULT_OK)
      {
        isScanning = true;
        Serial.println("Now YDLIDAR is scanning ......");
      }
      else
      {
        Serial.println("start YDLIDAR is failed!  Continue........");
      }
    }
    else
    {
      Serial.println("cannot retrieve YDLIDAR health");
    }
  }
  else
  {
    Serial.println("YDLIDAR get DeviceInfo Error!!!");
  }
}
