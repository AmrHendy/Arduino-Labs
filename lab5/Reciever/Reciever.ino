#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define led 3
RF24 radio(9, 53); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};
boolean buttonState = 0;
int prev_button_state = LOW;
int prev_led_state = LOW;

void setup() {
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  radio.begin();
  radio.openReadingPipe(1, addresses[0]);
  radio.setPALevel(RF24_PA_MIN);
}
void loop() {
  delay(5);
  radio.startListening();
  while (!radio.available());
  radio.read(&buttonState, sizeof(buttonState));
  Serial.println(buttonState);
  if (buttonState == HIGH && prev_button_state == LOW) {
    if(prev_led_state == HIGH){
      prev_led_state = LOW;
      digitalWrite(led, LOW);
    }
    else{
      prev_led_state = HIGH;
      digitalWrite(led, HIGH);
    }
  }
  prev_button_state = buttonState;
}
