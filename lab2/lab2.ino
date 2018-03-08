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
long xg_, yg_, zg_, xa_, ya_, za_;
bool first_time = true;


//angles values for the servo motor.
int angles[2] = {0,0};
Servo servos[2];  // create servo object to control a servo
//Servo servo2;  // create servo object to control a servo


float gyro_sensitivity = 0.681318;
float acc_sensitivity = 306.9;


float gyro_rate[3], acc_rate[3]; 



long prev_time = 0, curr_time;


float alpha = 0.98;

void setup() {
  Wire.begin();
  setupMPU();
  Serial.begin(9600);

 servos[0].attach(servo_pin1);  // attaches the servo on pin 9 to the servo object
 servos[1].attach(servo_pin2);
 servos[0].write(angles[0]);
 servos[1].write(angles[1]);

  read_values();
  
  xa_ = AcX;
  ya_ = AcY;
  za_ = AcZ;
  xg_ = GyX;
  yg_ = GyY;
  zg_ = GyZ;
/*
  Serial.println(xg_);
  Serial.println(yg_);
  Serial.println(zg_);
  */
  //delay(1000000);
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

  read_values();
  curr_time = millis();
  gyro_rate[0] = (GyX -  xg_) / gyro_sensitivity;
  gyro_rate[1] = (GyY -  yg_) / gyro_sensitivity;
  gyro_rate[2] = (GyZ -  zg_) / gyro_sensitivity;

  Serial.print("rate\n");
  Serial.println(gyro_rate[0]);
  Serial.println(gyro_rate[1]);
  Serial.println(gyro_rate[2]);
  Serial.println("================");

  acc_rate[0] = (AcX -  xa_) / acc_sensitivity;
  acc_rate[1] = (AcY -  ya_) / acc_sensitivity;
  acc_rate[2] = (AcZ -  za_) / acc_sensitivity;
  
  int dir;
  if(gyro_rate[0] > gyro_rate[1] && abs(GyX-xg_) > 100 )dir = 0;
  else if(abs(GyY-yg_)  > 100) dir = 1;
  else dir = -1;

  Serial.print("dir = ");
  Serial.println(dir);

  long dtt = curr_time - prev_time;

  
  Serial.print("dtt = ");
  Serial.println(dtt);
    
  float diff_angle_gyro = gyro_rate[dir] * dtt / 1000.0;

  
  Serial.print("diff_angle_gyro = ");
  Serial.println(diff_angle_gyro);

  //delay(1000);
  
  float diff_angle_acc = 0;
  float pitch = (atan2(AcY, AcZ) + PI) * RAD_TO_DEG;
  float roll = (atan2(AcX, AcZ) + PI) * RAD_TO_DEG;
  // pitch ub and down
  if(dir == 0){
    diff_angle_acc = pitch;
  }
  else{
    diff_angle_gyro = roll;
  }
  
  rotate_servos(diff_angle_gyro, diff_angle_acc, gyro_rate[dir],dir);
  prev_time = curr_time;
}


void rotate_servos(float diff_angle_gyro, float diff_angle_acc, int velo, int dir){
  if(dir == -1 ){
    return;
  }
  Serial.print("Velo");
  Serial.println(velo);
  
  servos[1 - dir].write(angles[1 - dir]);
  
  long val = alpha * (angles[dir] + diff_angle_gyro) + (1 - alpha) * diff_angle_acc - angles[dir];
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
    delay(10);
  }
}

void calc_values() {
  
}


void read_values(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers

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
  //Serial.println("===================================");  
}
