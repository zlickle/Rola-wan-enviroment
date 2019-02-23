#include<Arduino.h>
#include <SoftwareSerial.h>
#include <Adafruit_AM2315.h>
#include "E32_TTL_100.h"
#define TRANSMITT_ADDR_H 0x05
#define TRANSMITT_ADDR_L 0x02

#define RECV_ADDR_H 0x05
#define RECV_ADDR_L 0x01
unsigned int pm1 = 0;
unsigned int pm2_5 = 0;
unsigned int pm10 = 0;
Adafruit_AM2315 am2315;
float uvmap();
float temp();
float humidity();
int soilhumid();
int rain();
int wind();
/*
 need series a 4.7k Ohm resistor between .
 UNO/NANO(5V mode)                E32-TTL-100
    *--------*                      *------*
    | D7     | <------------------> | M0   |
    | D8     | <------------------> | M1   |
    | A0     | <------------------> | AUX  |
    | D10(Rx)| <---> 4.7k Ohm <---> | Tx   |
    | D11(Tx)| <---> 4.7k Ohm <---> | Rx   |
    *--------*                      *------*
*/


String data_send = "";
SoftwareSerial softSerial(10,11);  // RX, TX
E32_TTL_100 lora;


void setup()
{
  am2315.begin();
  Serial.begin(9600);
  Serial.println("Transmitter Init........");
  lora.E32_TTL_100_init(&softSerial,9600);
  RET_STATUS STATUS = RET_SUCCESS;
  struct CFGstruct CFG;
  struct MVerstruct MVer;
  pinMode(LED_BUILTIN, OUTPUT);
  STATUS = lora.SleepModeCmd(R_CFG, (void* )&CFG);
  STATUS = lora.SettingModule(&CFG,RECV_ADDR_H,RECV_ADDR_L);

  STATUS = lora.SleepModeCmd(R_MODULE_VERSION, (void* )&MVer);
  // Mode 0 | normal operation
  lora.SwitchMode(MODE_0_NORMAL);
  lora.WaitAUX_H();
  delay(10);
  
  if(STATUS == RET_SUCCESS)
    Serial.println("Setup init OK!!");
  //noInterrupts();
}

unsigned long previousMillis = 0;
void loop()
{
  int w=wind(); 
  float t=temp();
  float h=humidity();
  float uv=uvmap();
  int r=rain();
  int s=soilhumid();
  Serial.println("Temp: "+String(t)+" C");
  Serial.println("Humid "+String(h)+" %");
  Serial.println("UV "+String(uv)+"mW/cm");
  Serial.println("Rain Status: "+String(r));
  Serial.println("SoilHumid "+String(s)+" %");
  uint8_t data_buf[100], data_len;
  String recvStr = lora.ReceiveMsg(data_buf, &data_len);
  if(recvStr.length()>0){
    Serial.print("TEST INCOMING=");
    Serial.println(recvStr);
    data_send = String(pm1)+"/"+String(pm2_5)+"/"+String(pm10)+"/"+String(t)+"/"+String(h)+"/"+String(uv)+"/"+String(r)+"/"+String(s)+"/"+String(w);///////////
    if(recvStr == "GET"){
      if(lora.SendMsg(TRANSMITT_ADDR_H,TRANSMITT_ADDR_L,(char)data_send.length()+data_send)==RET_SUCCESS){
        Serial.println("SEND ACK");
      }else{
        Serial.println("ERROR FROM SEND BACK");
      }
    }
  }else{
    Serial.println("ERROR from loop");
  }
  delay(random(400, 600));
}

void serialEvent(){
  int index = 0;
  char value;
  char previousValue;
  while (Serial.available()) {
    value = Serial.read();
    if ((index == 0 && value != 0x42) || (index == 1 && value != 0x4d)){
      Serial.println("Cannot find the data header.");
      break;
    }

    if (index == 4 || index == 6 || index == 8 || index == 10 || index == 12 || index == 14) {
      previousValue = value;
    }
    else if (index == 5) {
      pm1 = 256 * previousValue + value;
      Serial.print("{ ");
      Serial.print("\"pm1\": ");
      Serial.print(pm1);
      Serial.print(", ");
    }
    else if (index == 7) {
      pm2_5 = 256 * previousValue + value;
      Serial.print("\"pm2_5\": ");
      Serial.print(pm2_5);
      Serial.print(", ");
    }
    else if (index == 9) {
      pm10 = 256 * previousValue + value;
      Serial.print("\"pm10\": ");
      Serial.print(pm10);
    } else if (index > 15) {
      break;
    }
    index++;
  }
  while(Serial.available()) Serial.read();
  Serial.println(" }");
  //delay(5000);
}
int soilhumid(){
  int avg=0;
  for(int i=0;i<10;i++){
    int humid = analogRead(A1);////
    avg += humid;
    delay(3);
  }
  avg = avg/10;
  int value = map(avg,580,310,0,100);
  if(value<0){
    return 0;
  }
  else if (value>100){
    return 100;
  }else{
    return value;
  }
  
  
}
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
int averageAnalogRead(int pinToRead){
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}
float uvmap(){
  int uvLevel = averageAnalogRead(A2);
  int refLevel = 690;
  float outputVoltage = 3.3 / refLevel * uvLevel;
  float uvIntensity = mapfloat(outputVoltage, 0.97, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level
  return uvIntensity;
}
int rain(){
  pinMode(2,INPUT);///digital 2
  int rainstatus = digitalRead(2);
  return rainstatus;
}
float temp(){
  // Connect RED of the AM2315 sensor to 5.0V
  // Connect BLACK to Ground
  // Connect WHITE to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
  // Connect YELLOW to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
  float tempvalue = am2315.readTemperature();
  return tempvalue;
}
float humidity(){
  float humidvalue = am2315.readHumidity();
  return humidvalue;
}
int wind(){
  int sensorValue = analogRead(A3);
  float outvoltage = sensorValue * (5.0 / 1023.0);
  int Level = 6*outvoltage;//The level of wind speed is proportional to the output voltage.
  return Level;
}





