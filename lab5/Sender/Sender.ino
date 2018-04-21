#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

int msg[1];
RF24 radio(9,53);
const uint64_t pipe = 0xE8E8F0F0E1LL;
int push_button_pin = 7;

void setup(void){
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(pipe);
}

void loop(void){
  if (digitalRead(push_button_pin) == HIGH){
    msg[0] = 111;
    radio.write(msg, 1);
  }
}
