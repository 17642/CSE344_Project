#include<string.h>
#include<Arduino.h>

#define VIRTUAL_GND D0
#define LED_1 D1
#define LED_2 D2
#define LED_3 D3

char buffer[21] = {0,};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  
  //while(!Serial){
  //  delay(5);
  //}

  pinMode(VIRTUAL_GND, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);

  digitalWrite(VIRTUAL_GND, LOW);
  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, LOW);
  digitalWrite(LED_3, LOW);

  Serial.println("READY");
  
}

void loop() {
  if(Serial.available()){
    int len = Serial.readBytes(buffer, sizeof(buffer));
    buffer[len] = '\0';

    for(int i=0;i<len;i++){
      if(buffer[i] == '\n' || buffer[i] == '\r'){
        buffer[i] = '\0';
        break;
      }
    }

    

    if(strcmp(buffer, "1 ON")==0){
      digitalWrite(LED_1, HIGH);
    }
    else if(strcmp(buffer, "2 ON")==0){
      digitalWrite(LED_2, HIGH);
    }
    else if(strcmp(buffer, "3 ON")==0){
      digitalWrite(LED_3, HIGH);
    }
    else if(strcmp(buffer, "RSET")==0){
      digitalWrite(LED_1, LOW);
      digitalWrite(LED_2, LOW);
      digitalWrite(LED_3, LOW);
    }

    delay(5);
  }

}
