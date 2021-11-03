#include <Arduino.h>
#include <Wire.h> 
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_GFX.h> 
#include <Adafruit_SSD1306.h> 
#include <PS2X_lib.h>

#define PS2_DAT 21 //MISO  19
#define PS2_CMD 19 //MOSI  23
#define PS2_SEL 18 //SS     5
#define PS2_CLK 5 //SLK   18
#define pressures false
#define rumble false
#define CHANNEL 1
#define ALTURA 64
#define LARGURA 128


PS2X ps2x;

int error = -1;
byte type = 0;
byte vibrate = 0;
int tryNum = 1;
uint64_t leituraAtual = 1;

typedef struct data
{
  int battery = 0;
  float kp = 0;
  float ki = 0;
  float kd = 0;
} data;
data myDataR;

typedef struct struct_message
{
  int Y = 0;
  int Z = 0;
  bool SAFE = false;
} struct_message;
struct_message myDataS;

Adafruit_SSD1306 display = Adafruit_SSD1306();
esp_now_peer_info_t peer;
uint8_t mac[] = {0xA4, 0xCF, 0x12, 0x75, 0x51, 0x6C};
uint8_t peerMacAddress[] = {0xCC, 0x50, 0xE3, 0x9C, 0x47, 0x40};

void addPeer(uint8_t *peerMacAddress) //adiciona o endereço do receptor
{
  peer.channel = CHANNEL;
  peer.encrypt = 0;
  memcpy(peer.peer_addr, peerMacAddress, 6);
  esp_now_add_peer(&peer);
}

bool InitESPNow() //inicia o esp now
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

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)// call back func de envio
{
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  memcpy(&myDataR, incomingData, sizeof(myDataR));
  display.clearDisplay();
  display.setCursor(0,0); 
  display.printf("%0.02f, %0.02f, %0.02f\n",myDataR.kp,myDataR.ki,myDataR.kd); 
  display.printf("bat: %d\n",myDataR.battery); 
  display.display();

  
}

void setup() {
  Wire.begin(23, 22);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
  display.setTextColor(WHITE); 
  display.setTextSize(0); 
  display.clearDisplay(); 
  display.setCursor(0,0); 
  display.print("Iniciando...\n"); 
  display.print("Controle: "); 
  display.display();
  while (error != 0)
  {
    delay(1000); // 1 second wait
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    tryNum++;
  }
  if (ps2x.readType() == 1)
    display.print("OK\n");
  else  
    display.print("ERORR\n");  
  display.display();
  
  display.print("WiFi: "); 
  WiFi.mode(WIFI_STA);
  vTaskDelay(2000);
  if (InitESPNow())
    display.print("OK\n");
  else{
    display.print("ERORR\n");
    display.print("Reboot in 5s\n");
    display.display();
    vTaskDelay(5000);
    ESP.restart();
  }  
  display.display();
  addPeer(peerMacAddress);
  display.print("Iniciando...");
  display.display();
  vTaskDelay(1000);
  display.clearDisplay(); 
  display.display();
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  ps2x.read_gamepad(false, vibrate);//atualiza os valores e mantem o controle ligado
  myDataS.Y = ps2x.Analog(PSS_RY);//frente e trás
  myDataS.Z = ps2x.Analog(PSS_RX);//esquerda e direita
  if (ps2x.ButtonPressed(PSB_CROSS))//fail safe 2 (ativa/desativa os motores)
  {
    vTaskDelay(10);
    myDataS.SAFE = !myDataS.SAFE;
  }

  esp_now_send(peerMacAddress, (uint8_t *)&myDataS, sizeof(myDataS));
  vTaskDelay(50);
}