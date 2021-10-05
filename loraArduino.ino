#include <SoftwareSerial.h>
#define RX 6
#define TX 7
SoftwareSerial loraSerial(RX, TX); // RX, TX
//Sensor Temp & Air Humi
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
//Sensor Soil Moisture
//#define SensorSoilMois A0

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
  if(millis() - clkTime > 30000){
    clkTime = millis();

    //Sensor Temp & Air Humidity
    int h = dht.readHumidity();
    node.writeSingleRegister(0x40000,h); //SAVE HUMIDITY TO REGISTER 0x40000H
    loraSerial.print(" Humi" + (String)h);
    delay(500);
    int t = dht.readTemperature();
    node.writeSingleRegister(0x40001,t); //SAVE TEMPERATURE TO REGISTER 0x40001H
    loraSerial.print(" Temp" + (String)t);
    delay(500);
    //Sensor Soil Moisture
    //int soilMois = analogRead(SensorSoilMois);
    //int percentSoilMois = map(soilMois, 0, 1023, 100, 0);
    //node.writeSingleRegister(0x40002,percentSoilMois); //SAVE SOIL MOISTURE TO REGISTER 0x40002H
    //loraSerial.print(" SoilMois" + (String)percentSoilMois);
    //delay(500);
    
    if (isnan(h) || isnan(t)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }
    
    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.println(" %");
    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.println(" *C");
    //Serial.print("Soil Moisture: ");
    //Serial.print(percentSoilMois);
    //Serial.println(" %"); 
  }
  
}
