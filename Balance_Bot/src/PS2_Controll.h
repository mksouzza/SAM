#include <Arduino.h>
#define MAX_TILT 5

TaskHandle_t second_core;

typedef struct Controller
{
  float Y = 0;
  float Z = 0;
  
  bool SAFE = false;
} Controller;
Controller controller;


typedef struct struct_message
{
  int Y = 0;
  int Z = 0;

  bool SAFE = false;
} struct_message;
struct_message myData;
