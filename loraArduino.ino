#include <SoftwareSerial.h>
#define RX 6
#define TX 7
SoftwareSerial loraSerial(RX, TX); // RX, TX
//Sensor
#include "DHT.h"
#define DHTPIN 10
#define DHTTYPE DHT22 
DHT dht(DHTPIN, DHTTYPE);
//TIMER
long clkTime = 0;
//RS485
#include <ModbusMaster.h>   //Modbus Lib
#define DE  3 // DE -> 3
#define RE  2 // RE -> 2
ModbusMaster node;   

//RELAY
int relayDV1 = 8;
int relayDV2 = 9;

//FUNCTION MODBUS
void preTransmission(){
  digitalWrite(RE, 1);         
  digitalWrite(DE, 1);
}
void postTransmission(){
  digitalWrite(RE, 0);
  digitalWrite(DE, 0);
}

void setup() {
  Serial.begin(9600);
  loraSerial.begin(9600);
  pinMode(relayDV1, OUTPUT);
  pinMode(relayDV2, OUTPUT);

  //OFF ALL
  digitalWrite(relayDV1, HIGH);
  digitalWrite(relayDV2, HIGH);

  //SENSOR
  dht.begin();
  
  //RS485 SETUP
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
  digitalWrite(RE, 0);
  digitalWrite(DE, 0);

  node.begin(1, Serial);  //ID SLAVE: 1
  node.preTransmission(preTransmission);  
  node.postTransmission(postTransmission);

  
}
void loop() {
  if(loraSerial.available()>0){
    String dataFromSender = loraSerial.readString();
    
    //SO SANH DU LIEU TU SENDER THUC HIEN LENH
    if(dataFromSender.indexOf("DV1On") > 0){
      loraSerial.print("SttDV1ON");
      Serial.println("DEVICE 1 IS ON!");
      //RELAYDV1 CONTROL ON
      digitalWrite(relayDV1, LOW);
    }
    else if (dataFromSender.indexOf("DV1Off") > 0){
      loraSerial.print("SttDV1OFF");
      Serial.println("DEVICE 1 IS OFF!");
      //RELAYDV1 CONTROL OFF
      digitalWrite(relayDV1, HIGH);
    }
    else if (dataFromSender.indexOf("DV2On") > 0){
      loraSerial.print("SttDV2ON");
      Serial.println("DEVICE 2 IS ON!");
      //RELAYDV2 CONTROL ON
      digitalWrite(relayDV2, LOW);
    }
    else if (dataFromSender.indexOf("DV2Off") > 0){
      loraSerial.print("SttDV2OFF");
      Serial.println("DEVICE 2 IS OFF!");
      //RELAYDV2 CONTROL OFF
      digitalWrite(relayDV2, HIGH);
    }
  }

  //SENSOR
  if(millis() - clkTime > 10000){
    clkTime = millis();
    
    int h = dht.readHumidity();
    node.writeSingleRegister(0x40000,h); //SAVE HUMIDITY TO REGISTER 0x40000H
    int t = dht.readTemperature();
    node.writeSingleRegister(0x40001,t); //SAVE TEMPERATURE TO REGISTER 0x40001H

    loraSerial.print(h*100 + t);
    
    if (isnan(h) || isnan(t)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }
    
    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.print(" *C ");
    Serial.println();
  }
  
}
