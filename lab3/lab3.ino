#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <Wire.h>
#include <LSM303.h>


#define ALEX_QIBLA 135.8     
#define CHAR_HEIGHT 8              
#define CHAR_WIDTH 5              


LSM303 compass; //lsm303 sensor
Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 4, 3); //nokia display

int r = 24;  // radius of compass rose
int x= 58;  // x-origin
int y = 24; // y-origin
float angle;  


void setup() {
  /*setup serial communication*/
  Serial.begin(9600);

  /*initiallize compass library*/
  Wire.begin();
  compass.init();
  compass.enableDefault();

  /*initiallize nokia lcd library*/
  display.begin();
  display.setContrast(30);
  display.clearDisplay();
}

void loop() {
  
 //calculate compass heading reading 
 calculateCompass();  
 //draw the compass and the line which points to North.
 drawCompass(); 
 delay(200);  
}

void calculateCompass(){
  //read compass values to be used later. 
  compass.read(); 
  //calculate heading value = Angle between North and -ve axis on the LSM chip.
  angle = compass.heading();
}

void drawCompass(){
  display.clearDisplay();
  
  /*diplay the angle on the screen. */
  display.setTextSize(2);
  display.setTextColor(BLACK);
  display.setCursor(0,0);
  display.println((int)angle);
  
  drawDirections();

  /* draw the line which points to the North. */
  display.drawLine(x, y, x + r * sin(angle / 180.0 * PI), y - r * cos(angle / 180.0 * PI), BLACK);
  display.display();
}

void drawDirections(){
  /*draw the compass circle*/
  display.drawCircle(x, y, r, BLACK);

  /*prepare settings for drawing*/
  display.setTextSize(1);
  display.setTextColor(BLACK);

  /*draw directions*/
  display.setCursor(x - CHAR_WIDTH / 2, 0);
  display.println("N");

  display.setCursor(x + r - CHAR_WIDTH, y - CHAR_HEIGHT / 2);
  display.println("E");

  display.setCursor(x - CHAR_WIDTH / 2, y * 2 - CHAR_HEIGHT);
  display.println("S");

  display.setCursor(x - r, y - CHAR_HEIGHT / 2);
  display.println("W");

  /*draw the line. */
  display.setCursor(x + r * sin(ALEX_QIBLA / 180.0 * PI) - CHAR_WIDTH / 2, y - r * cos(ALEX_QIBLA / 180.0 * PI) - CHAR_HEIGHT / 2);
  display.println("Q");
}
