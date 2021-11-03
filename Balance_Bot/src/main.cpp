#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <PS2_Controll.h>
#include "servoControl.h"
#include "PID.h"
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include <MPU.h>
#include <WiFi.h>
#include "Wire.h"
#define CHANNEL 1
#define MMin -0.1
#define MMax 0.1
MPU6050 mpu1(MPU6050_ADDRESS_AD0_HIGH);
MPU6050 mpu2(MPU6050_ADDRESS_AD0_LOW);

double Setpoint, Input, Output;
double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
PID PID_Y(&Input, &Output, &Setpoint, 0, 0, 0, 10, AUTOMATIC);
PID PID_Y1(&Input1, &Output1, &Setpoint1, 9, 0, .5, 10, AUTOMATIC);
PID PID_Y2(&Input2, &Output2, &Setpoint2, 9, 0, .5, 10, AUTOMATIC);

int pid_y1 = 0;
int pid_y2 = 0;
int pid_z = 0;

int Z_DRIFT = 0;
int Y_DRIFT = 0;

bool interruptCounter = 0;
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
uint8_t frame[10];

esp_now_peer_info_t peer;
uint8_t mac[] = {0xCC, 0x50, 0xE3, 0x9C, 0x47, 0x40};
uint8_t peerMacAddress[] = {0xA4, 0xCF, 0x12, 0x75, 0x51, 0x6C};

bool data1 = false;
bool data2 = false;

float tmpy1 = 0;
float tmpy2 = 0;

typedef struct data
{
  int battery = 0;
  float kp = 0;
  float ki = 0;
  float kd = 0;
} data;
data DataSend;

typedef struct test
{
  char m1 = 0;
  char m2 = 0;
} test;
test test1;

volatile typedef struct
{
  float X = 0;
  float Y = 0;
  float Z = 0;
} Angle;
Angle angle;

volatile typedef struct
{
  int motor1 = 0;
  int motor2 = 0;
} MOTOR;
MOTOR motor;

void Get_Angles();
void Calc_PID();
void Setup_MPU();
void Set_Motor(uint8_t Speed1, uint8_t Speed2);
void Motores();
void addPeer(uint8_t *peerMacAddress);
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

void addPeer(uint8_t *peerMacAddress)
{
  peer.channel = CHANNEL;
  peer.encrypt = 0;
  memcpy(peer.peer_addr, peerMacAddress, 6);
  esp_now_add_peer(&peer);
}

bool InitESPNow()
{
  if (esp_now_init() == ESP_OK)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&myData, incomingData, sizeof(myData));
  
  controller.Y = map(myData.Y, 0, 255, 5, -5);
  controller.Z = map(myData.Z, 0, 255, 5, -5);
  
  controller.SAFE = myData.SAFE;
}

volatile bool mpuInterrupt1 = false;
void dmpDataReady1()
{
  mpuInterrupt1 = true;
}

volatile bool mpuInterrupt2 = false;
void dmpDataReady2()
{
  mpuInterrupt2 = true;
}

void Get_Angles()
{
  if (mpu1.dmpGetCurrentFIFOPacket(fifoBuffer1))
  {

    float ypr[3];
    mpu1.dmpGetQuaternion(&q1, fifoBuffer1);
    mpu1.dmpGetGravity(&gravity1, &q1);
    mpu1.dmpGetYawPitchRoll(ypr, &q1, &gravity1);
    ypr[2] = (ypr[2] * 180 / PI);
    tmpy1 = ypr[2];
    data1 = true;
  }

  if (mpu2.dmpGetCurrentFIFOPacket(fifoBuffer2))
  {

    float ypr[3];
    mpu2.dmpGetQuaternion(&q2, fifoBuffer2);
    mpu2.dmpGetGravity(&gravity2, &q2);
    mpu2.dmpGetYawPitchRoll(ypr, &q2, &gravity2);
    ypr[2] = (ypr[2] * 180 / PI);
    tmpy2 = ypr[2];
    data2 = true;
  }

  if (data1 && data2)
  {
    data1 = false;
    data2 = false;
    angle.Y = (tmpy1 + tmpy2) / 2;
    Calc_PID();
    Motores();
  }
}

void Setup_MPU()
{

  mpu1.initialize();
  mpu2.initialize();

  devStatus1 = mpu1.dmpInitialize();
  devStatus2 = mpu2.dmpInitialize();

  if (devStatus1 == 0)
  {

    mpu1.setXGyroOffset(46);
    mpu1.setYGyroOffset(316);
    mpu1.setZGyroOffset(631);
    mpu1.setXAccelOffset(-6978);
    mpu1.setYAccelOffset(8866);
    mpu1.setZAccelOffset(11168);
    Serial.println(F("Enabling DMP..."));
    mpu1.setDMPEnabled(true);
    mpuIntStatus1 = mpu1.getIntStatus();
    dmpReady1 = true;
    //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN1), dmpDataReady1, RISING);
    packetSize1 = mpu1.dmpGetFIFOPacketSize();
  }
  if (devStatus2 == 0)
  {

    mpu2.setXGyroOffset(-39);
    mpu2.setYGyroOffset(-36);
    mpu2.setZGyroOffset(-74);
    mpu2.setXAccelOffset(-5174);
    mpu2.setYAccelOffset(5316);
    mpu2.setZAccelOffset(9498);
    Serial.println(F("Enabling DMP..."));
    mpu2.setDMPEnabled(true);
    mpuIntStatus2 = mpu2.getIntStatus();
    dmpReady2 = true;
    //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN2), dmpDataReady2, RISING);
    packetSize2 = mpu2.dmpGetFIFOPacketSize();
  }
}

void Calc_PID()
{
  timePrev = time1;             // the previous time is stored before the actual time read
  time1 = esp_timer_get_time(); // actual time read
  elapsedTime = (esp_timer_get_time() - timePrev);
  Input  = -angle.Y;
  Input1 = controller.Z;
  Input2 = controller.Z;
  Setpoint  = controller.Y;
  Setpoint1 = 0;
  Setpoint2 = 0;
  Serial.print(Output);
  Serial.print(",");
  PID_Y.Compute();
  PID_Y1.Compute();
  PID_Y2.Compute();
  Serial.print(Output1);
  Serial.print(",");
  Serial.println(Output2);
  pid_y1 = Output - Output1;
  pid_y2 = Output + Output2;
}

void Set_Motor(int Speed1, int Speed2)
{
  test1.m1 = Speed1;
  test1.m2 = Speed2;
  Wire1.beginTransmission(4);
  Wire1.write((uint8_t *)&test1, sizeof(test1));
  Wire1.endTransmission();
}

void Motores()
{

  if ((angle.Y < 30 && angle.Y > -30))
    Set_Motor(pid_y1, pid_y2);
  else
    Set_Motor(0, 0);
}

void Second_Core(void *o)
{
  while (1)
  {
    DataSend.kp = PID_Y1.GetKp();
    DataSend.ki = PID_Y1.GetKi();
    DataSend.kd = PID_Y1.GetKd();

    DataSend.battery = 89;

    esp_now_send(peerMacAddress, (uint8_t *)&DataSend, sizeof(DataSend));
    vTaskDelay(500);
  }
}

void setup()
{

  Serial.begin(115200);
  disableCore0WDT();
  Wire.begin(SDA_H, SCL_H);
  Wire1.begin(SDA_P, SCL_P);
  Wire.setClock(400000);
  Wire1.setClock(400000);
  Serial.println("LED");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Setup_MPU();

  Serial.println("MPU OK");
  digitalWrite(LED_BUILTIN, HIGH);
  PID_Y.SetTunings(pid.KP_XY, pid.KI_XY, pid.KD_XY);
  PID_Y.SetMode(AUTOMATIC);
  PID_Y.SetOutputLimits(-100, 100);
  PID_Y.SetSampleTime(5);

  PID_Y1.SetTunings(pid.KP_Z, pid.KI_Z, pid.KD_Z);
  PID_Y1.SetMode(AUTOMATIC);
  PID_Y1.SetOutputLimits(-100, 100);
  PID_Y1.SetSampleTime(5);

  PID_Y2.SetTunings(pid.KP_Z, pid.KI_Z, pid.KD_Z);
  PID_Y2.SetMode(AUTOMATIC);
  PID_Y2.SetOutputLimits(-100, 100);
  PID_Y2.SetSampleTime(5);
  Serial.println("Pronto");

  WiFi.mode(WIFI_STA);
  vTaskDelay(2000);
  if (!InitESPNow())
    ESP.restart();
  addPeer(peerMacAddress);
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  xTaskCreatePinnedToCore(
      &Second_Core,
      "Second_Core",
      10000,
      NULL,
      1,
      &second_core,
      0);
}

void loop()
{
  Get_Angles();
}
