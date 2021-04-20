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
#define SensorSoilMois A0
//AS Sensor
#define rxSensor 12
#define txSensor 11
SoftwareSerial sensorSerial(rxSensor, txSensor);
int Pin_x = 4;
int Pin_y = 5;
char computerdata[20];
char sensordata[30];
byte computer_bytes_received=0;
byte sensor_bytes_received=0;
char *cmd;
int counterSensorAS;
float dataToFloat;
int pH;
int ORP;

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

//FUNCTION AS SENSOR
void serialEvent(){                                                 
  sensor_bytes_received=sensorSerial.readBytesUntil(13,sensordata,30); 
  sensordata[sensor_bytes_received]=0;
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
  
  //AS SENSOR SERIAL SETUP
  pinMode(Pin_x, OUTPUT);
  pinMode(Pin_y, OUTPUT);
  sensorSerial.begin(9600); 
  
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
    int soilMois = analogRead(SensorSoilMois);
    int percentSoilMois = map(soilMois, 0, 1023, 100, 0);
    node.writeSingleRegister(0x40002,percentSoilMois); //SAVE SOIL MOISTURE TO REGISTER 0x40002H
    loraSerial.print(" SoilMois" + (String)percentSoilMois);
    delay(500);
    //AS Sensor
    node.writeSingleRegister(0x40003,ORP);
    loraSerial.print(" ORP" + (String)ORP);
    delay(500);
    node.writeSingleRegister(0x40004,pH);
    loraSerial.print(" pH" + (String)pH);
    delay(500);
    
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
    Serial.print("Soil Moisture: ");
    Serial.print(percentSoilMois);
    Serial.println(" %"); 
  }

  //AS SENSOR
    if(sensor_bytes_received!=0){
    //channel=strtok(computerdata, ":");
    cmd=strtok(NULL, ":");
    if(counterSensorAS  == 0){
      open_channel (1);
      counterSensorAS = 1;
      }
    else{
      open_channel (2);
      counterSensorAS = 0;
      }
    if(cmd!=0){
      sensorSerial.print(cmd);
      sensorSerial.print("\r");
      }
      computer_bytes_received=0;
    }
  
  if(sensorSerial.available() > 0){
    sensor_bytes_received=sensorSerial.readBytesUntil(13,sensordata,30);
    sensordata[sensor_bytes_received]=0;
    dataToFloat = atof(sensordata);
    if (dataToFloat >= 0 && dataToFloat <= 14){
      pH = dataToFloat;
      Serial.print("pH: ");
      Serial.println(pH);
      }
    else {
      ORP = dataToFloat;
      Serial.print("ORP: ");
      Serial.println(ORP);
      }
  } delay(1000);
  
}

//FUNCTION AS SENSOR
void open_channel(int channel_function){                          
     switch (channel_function) {                               
       case 1:                                       
         digitalWrite(Pin_x, LOW);                   
         digitalWrite(Pin_y, LOW);                  
       break;                                
        
       case 2:
         digitalWrite(Pin_x, LOW);
         digitalWrite(Pin_y, HIGH);
       break;

       case 3:
         digitalWrite(Pin_x, HIGH);
         digitalWrite(Pin_y, LOW);
       break;

       case 4:
         digitalWrite(Pin_x, HIGH);
         digitalWrite(Pin_y, HIGH);
       break;
      }
}    
