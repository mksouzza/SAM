#include <Arduino.h>
#include <Wire.h>
#define SCL_H GPIO_NUM_23
#define SDA_H GPIO_NUM_22

#define SCL_P GPIO_NUM_16
#define SDA_P GPIO_NUM_17

#define INTERRUPT_PIN1 39
#define INTERRUPT_PIN2 5


bool dmpReady1 = false;
uint8_t mpuIntStatus1;
uint8_t devStatus1;
uint16_t packetSize1;
uint16_t fifoCount1;
uint8_t fifoBuffer1[64];
Quaternion q1;
VectorFloat gravity1;

bool dmpReady2 = false;
uint8_t mpuIntStatus2;
uint8_t devStatus2;
uint16_t packetSize2;
uint16_t fifoCount2;
uint8_t fifoBuffer2[64];
Quaternion q2;
VectorFloat gravity2;

uint32_t timer1 = 0;
int state = 0;
