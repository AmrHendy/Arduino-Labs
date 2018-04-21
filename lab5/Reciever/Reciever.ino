#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
int msg[1];
RF24 radio(9,53);
const uint64_t pipe = 0xE8E8F0F0E1LL;
int led_pin = 3;
int led_state = LOW;

void setup(void){
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(1,pipe);
  radio.startListening();
  pinMode(led_pin, OUTPUT);
}

void loop(void){
  if (radio.available()){
    bool done = false;
    while (!done){
      done = radio.read(msg, 1);
      Serial.println(msg[0]);
      if (msg[0] == 111){
        delay(10);
        if(led_state == HIGH){
          digitalWrite(led_pin, LOW);
          led_state = LOW;
        }
        else{
          digitalWrite(led_pin, HIGH);
          led_state = HIGH;
        }
      }
      else {
        // do not change the led state
      }
      delay(10);
    }
  }
  else{
    Serial.println("No radio available");
  }
}
