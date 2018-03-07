#include <Wire.h>
#include <Servo.h>


// 6 for horizontal rotation , 7 for the vertical rotation.
const int servo_pin1 = 6, servo_pin2 = 7;

// the address of MPU6050 which is known by I2C to make connection
// we have two options 0b1101000 (common used) if we make AD0 = 0 or 0b1101001 if we make AD0 = 1
const int MPU_addr = 0x68;  // I2C address of the MPU-6050

//reading values.
long AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;


//calibration to remove the error value at first 
long x_, y_, z_;
bool first_time = true;


//angles values for the servo motor.
int angles[2] = {0,0};
Servo servos[2];  // create servo object to control a servo
//Servo servo2;  // create servo object to control a servo

void setup() {
  Wire.begin();
  setupMPU();
  Serial.begin(9600);

 servos[0].attach(servo_pin1);  // attaches the servo on pin 9 to the servo object
 servos[1].attach(servo_pin2);
 servos[0].write(angles[0]);
 servos[1].write(angles[1]);
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

  read_values();

  if(first_time){
    x_ = AcX;
    y_ = AcY;
    z_ = AcZ;
    first_time = false;
  }
  
  rotate_servos(2000,30,0);
}


void rotate_servos(long velo, int val, int dir){
  servos[1 - dir].write(angles[1 - dir]);
  long time1;
  time1 = ((val * 1.0) / velo) * 1000;
  if(val == 0){
    servos[dir].write(angles[dir]);
    return;
  }
  
  int delayy = time1/val;
  for(int i=0;i<val;i++){
    servos[dir].write((angles[dir] + 1) % 181);
    angles[dir] = (angles[dir] + 1) % 181;
    delay(delayy);
  }
}

void calc_values() {
  
}


void read_values(){
  readAcceleration();
  readTempreture();
  readGyro();
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

  
  // Plot
  Serial.print(GyX);Serial.print(",");
  Serial.print(GyY);Serial.print(",");
  Serial.println(GyZ);
  Serial.println("===================================");  
}
