#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define button 7
RF24 radio(9, 53); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};
boolean buttonState = 0;
void setup() {
  Serial.begin(9600);
  pinMode(button, INPUT);
  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00002
  radio.setPALevel(RF24_PA_MIN);
}
void loop() {
  delay(5);
  buttonState = digitalRead(button);
  radio.write(&buttonState, sizeof(buttonState));
  Serial.println(buttonState);
}
  

