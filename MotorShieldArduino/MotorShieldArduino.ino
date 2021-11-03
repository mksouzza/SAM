#include <Wire.h>

#define BRAKEVCC 0
#define CW 1
#define CCW 2
#define BRAKEGND 3
#define CS_THRESHOLD 1024
#define SHIFT 500

long long Watch_Dog = 0;
long long setTime = 0;
int inApin[2] = {7, 4}; // INA: Sentido Horário Motor0 e Motor1 (Consulte:"1.2) Hardware Monster Motor Shield").
int inBpin[2] = {8, 9}; // INB: Sentido Anti-Horário Motor0 e Motor1 (Consulte: "1.2) Hardware Monster Motor Shield").
int pwmpin[2] = {5, 6};            // Entrada do PWM
int cspin[2] = {2, 3};              // Entrada do Sensor de Corrente

typedef struct test
{
  char m1 = 0;
  char m2 = 0;
} test;
test test1;

int statpin = 13;
int i = 0;;
uint8_t frame[10];

void setup()
{
  Serial.begin(9600);
  Wire.begin(4);
  Wire.setClock(400000);
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  Wire.onReceive(receiveEvent);
  pinMode(statpin, OUTPUT);
  for (int i = 0; i < 2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
}

void loop()
{
  Watch_Dog = millis();
  if (Watch_Dog / setTime >= 1)
  {
    setTime = Watch_Dog + SHIFT;
    motorOff(0);
  }
  if ((analogRead(cspin[0]) > CS_THRESHOLD) || (analogRead(cspin[1]) > CS_THRESHOLD)) {
    motorOff(0);
    motorOff(1);
  }
  delay(1);
}

void motorOff(int motor)     //Função para desligar o motor se o mesmo travar
{

  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
  i = 0;
  digitalWrite(13, HIGH);
  delay(1000);
}

void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <= 4)
    {
      if (direct <= 1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      if ((direct == 0) || (direct == 2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}

void receiveEvent(int howMany)
{
  if (Wire.available()) {
    unsigned char buffer = 0;
    unsigned char overflow = 0;
    while (Wire.available())
    {
      frame[buffer] = Wire.read();
      buffer++;
    }
    memcpy(&test1, frame,buffer);
    if(test1.m1>=1)
    { 
      uint8_t motor1 = test1.m1;  
      motor1 = map(test1.m1, 0, 100, 0, 255);
      motorGo(0, CW, motor1);
    }else if(test1.m1<=-1)
    {
      uint8_t motor1 = 0;  
      motor1 = map(test1.m1, -1, -100, 0, 255);
      Serial.print(motor1);
      Serial.print(",");
      motorGo(0, CCW, motor1);  
    }
    else
      motorOff(0);
    if(test1.m2>=1)
    { 
      uint8_t motor2 = 0;  
      motor2 = map(test1.m2, 0, 100, 0, 255);
      motorGo(1, CCW, motor2);
    }else if(test1.m2<=-1)
    {
      uint8_t motor2 = 0;  
      motor2 = map(test1.m2, -1, -100, 0, 255);
      Serial.print(motor2);
      motorGo(1, CW, motor2);  
    }
    else
      motorOff(1);
    Serial.println();
    setTime = millis() + SHIFT;
  }
}
