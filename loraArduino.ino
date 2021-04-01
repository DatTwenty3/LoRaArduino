#include <SoftwareSerial.h>
#define RX 6
#define TX 7
SoftwareSerial loraSerial(RX, TX); // RX, TX

int i = 0;
int relayDV1 = 8;
int relayDV2 = 9;

void setup() {
  Serial.begin(9600);
  loraSerial.begin(9600);
  pinMode(relayDV1, OUTPUT);
  pinMode(relayDV2, OUTPUT);
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
}
