#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
int msg[1];
RF24 radio(9,10);
const uint64_t pipe = 0xE8E8F0F0E1LL;
int led_pin = 3;

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
        digitalWrite(led_pin, HIGH);
      }
      else {
        digitalWrite(led_pin, LOW);
      }
      delay(10);
    }
  }
  else{
    Serial.println("No radio available");
  }
}
