#include <Wire.h>
#include <LiquidCrystal.h>

#define window_size 20
#define check_after_samples 50

// the address of MPU6050 which is known by I2C to make connection
// we have two options 0b1101000 (common used) if we make AD0 = 0 or 0b1101001 if we make AD0 = 1
const int MPU_addr = 0x68;  // I2C address of the MPU-6050

long AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

long total_XYZ[3][check_after_samples] = {0};
long acc_arr[3][window_size] = {{0}};
long counter;
int counter_big_array;

int lcd_delay = 600;
//calibration to remove the error value at first 
long x_, y_, z_;
bool first_time = true;

// counter for steps
long steps = 0;


// initialize the library by providing the nuber of pins to it
LiquidCrystal lcd(8,9,4,5,6,7);

// the starting time of program
unsigned long startTime;

bool flag = false;


float vel = 0;
float last_time = 0;
float last_distance = 0;


float last_v = 0;
float average_acc = 0;

void setup() {
  Wire.begin();
  setupMPU();
  counter = 0;
  counter_big_array = 0;
  Serial.begin(9600);

  //initialize the lcd
  lcd.begin(16,2);
  lcd.clear();

  startTime = millis();

}


void setupMPU() {
  Wire.beginTransmission(MPU_addr);

  ///we select the register number 6B as it is responsible of controlling sleep and clock of MPU device
  //MPU at first will be is sleep mode = not active = not sampling so we should wake him by setting sleep bits = 0
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)

  Wire.endTransmission(true);

}


void loop() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers

  readAcceleration();
  readTempreture();
  readGyro();

  if(first_time){
    x_ = AcX;
    y_ = AcY;
    z_ = AcZ;
    first_time = false;
  }
  
  if (flag) {
    calc_values();
  }
  
  acc_arr[0][counter] = AcX - x_;
  acc_arr[1][counter] = AcY - y_;
  acc_arr[2][counter] = AcZ - z_;
  
  counter++;
  if(counter == window_size)flag = true;
  
  if(counter == window_size){
    counter = 0;
  }

  if(millis() % 1000 == 0){
     float temp1 = millis();
     float temp2 = getDistance();
     vel = (temp2 - last_distance) / ((temp1 - last_time) / 1000.0);
     last_distance = temp2;
     last_time = temp1;
  }
  
  printOnLcd();

/*
  Serial.print("Distance = "); Serial.println(getDistance());
  Serial.print("Velo = "); Serial.println(getVelocity());
  Serial.print("Calroies = "); Serial.println(getCalories());
  delay(500);
*/

}


void calc_values() {
  for (int i = 0 ; i < 3 ; i++) {
    total_XYZ[i][counter_big_array] = 0;
    for (int j = 0 ; j < window_size ; j++) {
      total_XYZ[i][counter_big_array] += acc_arr[i][j];
    }
    total_XYZ[i][counter_big_array] /= window_size;
  }
  
  counter_big_array++;
  if(counter_big_array == check_after_samples){
    counter_big_array=0;
    long best_diff = 0, threshold;
    int dir;
    for(int i=0;i<2;i++){
      long min_val = calculate_min(i);
      long max_val = calculate_max(i);
      if(max_val - min_val > best_diff){
        dir = i;
        best_diff = max_val - min_val;
        threshold = best_diff / 2 + min_val;
        average_acc = threshold / 10;
      }
    } 
    int samples = check_threshold(dir, threshold); 

    if(samples >= 1 && best_diff > 1800){
      steps++;
    }  
  }
}


long calculate_min(int dir){
  // max value of long
  long min_value = 2147483647;
  // get the min value of the 50 samples
  for(int i=0;i<check_after_samples;i++){
    min_value = min(total_XYZ[dir][i], min_value);
  }
  return min_value;
}

long calculate_max(int dir){
  // min value of long
  long max_value = -2147483648;
  // get the max value of the 50 samples
  for(int i=0;i<check_after_samples;i++){
    max_value = max(total_XYZ[dir][i], max_value);
  }
  return max_value;
}

// return number of samples which have value greater than the threshold
int check_threshold(int dir, long threshold){
  int samples = 0;
  for(int i=0;i<check_after_samples;i++){
    if(total_XYZ[dir][i] > threshold){
      if((i == 0 || total_XYZ[dir][i] > total_XYZ[dir][i - 1]) &&  (i == 49 || total_XYZ[dir][i] > total_XYZ[dir][i + 1])){
        samples++;
      }
    }
  }
  return samples;
}


void readAcceleration() {

  while (Wire.available() < 6);

  // read the acceleration values , note that the value is divided into two adjacent register
  // each one has the first half bits so we should concatenate them by shifting and oring
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  /*
  // Plot
  Serial.print(AcX);Serial.print(",");
  Serial.print(AcY);Serial.print(",");
  Serial.println(AcZ);
  */
}


void readTempreture() {
  while (Wire.available() < 1);
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
}

void readGyro() {

  while (Wire.available() < 6);

  // read the gyro values , note that the value is divided into two adjacent register
  // each one has the first half bits so we should concatenate them by shifting and oring
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

int index = 0;
void printOnLcd(){
  lcd.clear();  
  
  lcd.setCursor(0,0);
  lcd.print("Step=");
  lcd.print(max(steps, 0));
  
  lcd.print(",D=");
  lcd.print(getDistance());
  
  
  lcd.setCursor(0,1);
  lcd.print("V=");
  lcd.print(getVelocity());
  //lcd.print("m/s");
  
  lcd.print(",C=");
  lcd.print(getCalories());
}


// distance by meter
float getDistance(){
  return (max(steps, 0) * 74.0) / 100.0;
}

//velocity by m/s
float getVelocity(){
  //float v = last_v + ((millis() - startTime) / 1000.0) * average_acc;
  //Serial.println(v);
  //last_v = v;
  //startTime = millis();
  //return v;
  
  //return (getDistance() / ((millis() - startTime) / 1000.0));

  return vel;
}

//calaroies by (C/kg/h)
float getCalories(){
  return 4.5 * getVelocity();
}

