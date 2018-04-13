class Flasher
{
  int ledPin;          
  long blankingTime;
  
  // A variable should be declared volatile whenever its value can be changed by something
  // beyond the control of the code section in which it appears, such as a concurrently executing thread.
  // In the Arduino, the only place is in sections of code associated with interrupts, called an interrupt service routine.
  
  // These maintain the current state
  volatile int ledState;                    // ledState used to set the LED (LOW or HIGH)
  volatile unsigned long previousMillis;    // will store last time LED was updated
 
  
  public:
  Flasher(int pin, long blankTime)
  {
    ledPin = pin;
    pinMode(ledPin, OUTPUT);     
    
    blankingTime = blankTime;
    ledState = LOW; 
    previousMillis = 0;
  }
 
  void Update(unsigned long currentMillis)
  {
    if((ledState == HIGH) && (currentMillis - previousMillis >= blankingTime))
    {
      ledState = LOW;  // Turn it off
      previousMillis = currentMillis;  // Remember the time
      digitalWrite(ledPin, ledState);  // Update the actual LED
    }
    else if ((ledState == LOW) && (currentMillis - previousMillis >= blankingTime))
    {
      ledState = HIGH;  // turn it on
      previousMillis = currentMillis;   // Remember the time
      digitalWrite(ledPin, ledState);   // Update the actual LED
    }
  }

  void decreaseRate()
  {
    blankingTime = max(blankingTime - 100, 0);
  }
};


// pins for the led are 11, 12 (digital pins)
Flasher led1(11, 1000);
Flasher led2(12, 1000);

// pins for the push buttons are 2, 3 (digital pins)
int interruptPin1 = 2;
int interruptPin2 = 3;

void setup() 
{   
  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function below
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  // in UNO interrupt pins are 2, 3 (digital pins)
  // in Mega interrupt pins are 2, 3, 18, 19, 20, 21 (digital pins)
  // use FAILLING not CHANGE in interrupt type to detect bush putton press (go from high to low)
  
  attachInterrupt(digitalPinToInterrupt(interruptPin1), decrease1, FALLING);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), decrease2, FALLING);
} 


void decrease1()
{
  led1.decreaseRate();  
}


void decrease2()
{
  led2.decreaseRate();  
}


// Interrupt is called once a millisecond, 
SIGNAL(TIMER0_COMPA_vect) 
{
  unsigned long currentMillis = millis();
  led1.Update(currentMillis);
  led2.Update(currentMillis);
} 
 
void loop()
{

}
