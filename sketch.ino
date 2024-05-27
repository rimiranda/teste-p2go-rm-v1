
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

#define TOPIC_SUBSCRIBE_LED       "topico_tcl_tsl_led"
#define TOPIC_PUBLISH_TEMPERATURE "topico_sensor_temperature"
#define TOPIC_PUBLISH_HUMIDITY    "topico_sensor_humidity"
#define TOPIC_PUBLISH_LED_COLOR    "topico_color_led"
#define TOPIC_PUBLISH_LED_STATE    "topico_state_led"

// WiFi Credential
const char *ssid = "Wokwi-GUEST";
const char *password = "";

// MQTT Credential
const char *mqtt_broker = "broker.hivemq.com";
const char *mqtt_username = "";
const char *mqtt_password = "";
const int mqtt_port = 1883;

// MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Led RGB
#define COMMON_ANODE;
#define COLOR_OFF 0
#define COLOR_R 1
#define COLOR_G 2
#define COLOR_B 3
#define COLOR_Y 4

const byte BUTTON_PIN = 1;  // Pino do Botao
const int rgbLedPinR = 13;  // Pino do LED RGB (Vermelho)
const int rgbLedPinG = 12;  // Pino do LED RGB (Verde)
const int rgbLedPinB = 11;  // Pino do LED RGB (Azul)

const char* StateLed[4] = {"Desligado", "Ligado", "Led Piscando 1 seg", "Led Piscando 0,3 seg"};
const char* CorLed[5] = {"Apagado", "Vermelho", "Verde", "Azul", "Amarelo"};

// Variaveis globais de controle
byte valorBotao;
int valorContBotao = 0; 
int valorContBotaoTemp = 0; 
byte flagBotao=0;
byte flagCorLed=1;
byte flagStateLed=0;
byte flagBloqR=0;

DHT dht(15,DHT22);
//QueueHandle_t xtemp, xhumi;
QueueHandle_t xtemp, xhumi, xcorled, xstateled;
TaskHandle_t task1, task2, task3, task4;
SemaphoreHandle_t xsemaforo;
SemaphoreHandle_t xsemaforo1;

void setup() {
  Serial.begin(115200);
  Serial.println("TESTE-LED RGB!");

  pinMode(rgbLedPinR, OUTPUT);
  pinMode(rgbLedPinG, OUTPUT);
  pinMode(rgbLedPinB, OUTPUT);
  pinMode (BUTTON_PIN, INPUT_PULLUP);

// Define a interrupção como detecção na borda de descida
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button1f_isr_handler, FALLING);

// Inicializa a conexao Wi-Fi
  wifiSetup();

  dht.begin(); 
  xtemp = xQueueCreate(1,sizeof(float));
  xhumi = xQueueCreate(1,sizeof(float));
  xsemaforo = xSemaphoreCreateMutex();
  xcorled = xQueueCreate(1,sizeof(byte));
  xstateled = xQueueCreate(1,sizeof(byte));
  xsemaforo = xSemaphoreCreateMutex();
  xsemaforo1 = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(core1_led,"Task Led",4000,NULL,3,&task1,1);
  xTaskCreatePinnedToCore(core1_sensor,"Task Sensor",4000,NULL,3,&task2,1);
  xTaskCreatePinnedToCore(core0_botao,"Task Botao",4000,NULL,4,&task3,0);  
  xTaskCreatePinnedToCore(core0_relatorio,"Task Relatorio",4000,NULL,3,&task4,0);  
}

void button1f_isr_handler(void){
  flagBotao = 1;
}

void core1_led(void *pvParameters) {

  byte corLed;
  byte stateLed;

  while (true) {
    xSemaphoreTake(xsemaforo1,portMAX_DELAY);
    xSemaphoreGive(xsemaforo1); 

    if(xQueueReceive(xcorled,&corLed,portMAX_DELAY) == pdTRUE) {}
    if(xQueueReceive(xstateled,&stateLed,portMAX_DELAY) == pdTRUE) {}

    switch(stateLed){
      case 0: // Led Desligado
              setLed(COLOR_OFF);
              vTaskDelay(100); 
      break;
      case 1: // Led Ligado - Inicia na cor vermelha
              setLed(corLed);
              vTaskDelay(100); 
      break;
      case 2: // Led Piscando 1 seg
              setLed(corLed);
             vTaskDelay(500); 
             setLed(COLOR_OFF);
             vTaskDelay(500); 
      break;
      case 3: // Led Piscando 0,3 seg 
             setLed(corLed);
             vTaskDelay(150); 
             setLed(COLOR_OFF);
             vTaskDelay(150); 
      break;
    }

    xSemaphoreGive(xsemaforo1); 
    vTaskDelay(5); 
  }
}

void core1_sensor(void *pvParameters) {
  while (true) {
    xSemaphoreTake(xsemaforo,portMAX_DELAY);
    float temp = dht.readTemperature();
    float humi = dht.readHumidity();
    xQueueSend(xtemp,&temp,0);
    xQueueSend(xhumi,&humi,0); 
    xSemaphoreGive(xsemaforo); 
    vTaskDelay(100); 
  }
}

void core0_botao(void *pvParameters) {
while (true) {

    if (flagBotao == 1){
      valorBotao = digitalRead(BUTTON_PIN);
      if(digitalRead(BUTTON_PIN) == LOW){
        vTaskDelay(20);
        valorContBotao++;
        valorContBotaoTemp = valorContBotao; 
      }
      else{
        valorContBotaoTemp = valorContBotao; 
        flagBotao = 0;
        flagBloqR = 1;
      }
    }
    if (flagBloqR){      
      if (valorContBotaoTemp < 7){
        vTaskDelay(20);
        flagBotao = 0;
        valorContBotao = 0;
        valorContBotaoTemp = 0;
        // Muda o estado do led
        flagStateLed++;
        if (flagStateLed > 3){flagStateLed = 0;}
        flagBloqR = 0;
      }
      if ((valorContBotaoTemp >= 7) && (valorContBotaoTemp <= 21)){
        vTaskDelay(20);
        valorContBotao = 0;
        valorContBotaoTemp = 0;
        flagBloqR = 0;
      }
      if (valorContBotaoTemp > 21){
        vTaskDelay(20);
        flagBotao = 0;
        valorContBotao = 0;
        valorContBotaoTemp = 0;
        // Muda a cor do led
        flagCorLed++;
        if (flagCorLed > 4){flagCorLed = 1;}
        flagBloqR = 0;
      }
    }

    xSemaphoreTake(xsemaforo1,portMAX_DELAY);

    xQueueSend(xcorled,&flagCorLed,0); 
    xQueueSend(xstateled,&flagStateLed,0); 

    xSemaphoreGive(xsemaforo1);

    vTaskDelay(5);
  }
}

void core0_relatorio(void *pvParameters) {

  float temp, hum;
  while (true) {    

    vTaskDelay(3000); 

    xSemaphoreTake(xsemaforo,portMAX_DELAY);
    Serial.println("Report ******");

    if(xQueueReceive(xtemp,&temp,portMAX_DELAY) == pdTRUE) {
      Serial.println("Temp: " + String(temp) + "C");
    }
    if(xQueueReceive(xhumi,&hum,portMAX_DELAY) == pdTRUE) {
      Serial.println("Humid: " + String(hum) + "%");
    }
    Serial.println("Estado do Led: " + String(StateLed[flagStateLed]));
    Serial.println("Cor do Led: " + String(CorLed[flagCorLed]));

    Serial.println("****** ******");

    // Envio MQTT
    client.publish(TOPIC_PUBLISH_TEMPERATURE, String(temp).c_str());
    client.publish(TOPIC_PUBLISH_HUMIDITY, String(hum).c_str());
    client.publish(TOPIC_PUBLISH_LED_COLOR, String(CorLed[flagCorLed]).c_str());
    client.publish(TOPIC_PUBLISH_LED_STATE, String(StateLed[flagStateLed]).c_str());
    client.loop();

    xSemaphoreGive(xsemaforo);
    vTaskDelay(20);
  }
}

void loop() {

}

void setLed(int color){

  switch(color){
    case 0: setColor(0, 0, 0);
    break;
    case 1: setColor(255, 0, 0);
    break;
    case 2: setColor(0, 255, 0);
    break;
    case 3: setColor(0, 0, 255);
    break;
    case 4: setColor(255, 255, 0);
    break;
  }
}

void setColor(int vermelho, int verde, int azul){

  #ifdef COMMON_ANODE
  vermelho = 255 - vermelho;
  verde = 255 - verde;
  azul = 255 - azul;
  #endif
  analogWrite(rgbLedPinR, vermelho);
  analogWrite(rgbLedPinG, verde);
  analogWrite(rgbLedPinB, azul);
}