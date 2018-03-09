#include <Wire.h>
#include <Servo.h>

#define acc_sensitivity 8192.0
#define gyro_sensitivity 65.536
// 10 ms sample rate. */
#define dt 0.01                 


/* MPU values. */
int gyro[3];
int acc[3];
int temp;

/* gyro calibration values. */
long gyro_cal[3];

/*pin 7 for (pitch values) dir = 0 , pin 7 for (roll values) dir = 1. */
const int servo_pins[2] = {6, 7};
Servo servos[2];  

/* initial angles for the servo motor. */
int angles[2] = {90,90};

/* variables to calculate pitch & roll values. */
float angle = 0;
int dir;
float pitch_angle, roll_angle, prev_pitch, prev_roll;
float tolerance = 0.1;
bool first_time = false;

void setup() {
  Wire.begin();
  Serial.begin(9600);                                                        
  
  setupMPU();                                           
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  
    read_values();
    gyro_cal[0] += gyro[0];                                              
    gyro_cal[1] += gyro[1];                                              
    gyro_cal[2] += gyro[2];                                              
    delay(3);                                                          
  }

  /* divide by 1000 to get avarage offset. */
  gyro_cal[0] /= 1000;                                                 
  gyro_cal[1] /= 1000;                                                 
  gyro_cal[2] /= 1000;                                                 

  /* attaches the servo on pin 9 to the servo object. */
  servos[0].attach(servo_pins[0]);  
  servos[1].attach(servo_pins[1]);
  servos[0].write(angles[0]);
  servos[1].write(angles[1]);
  
}

void loop(){
  read_values();   
  gyro[0] -= gyro_cal[0];
  gyro[1] -= gyro_cal[1];
  gyro[2] -= gyro_cal[2];

  ComplementaryFilter(acc, gyro, &pitch_angle, &roll_angle);
}


 

void ComplementaryFilter(int accData[3], int gyrData[3], float *pitch, float *roll)
{
    float pitchAcc, rollAcc;               
 
    /* Integrate the gyroscope data which is angular velocity to get the angle. */
    // Angle around the X-axis
    *pitch += ((float)gyrData[0] / gyro_sensitivity) * dt; 
    // Angle around the Y-axis
    *roll -= ((float)gyrData[1] / gyro_sensitivity) * dt;   
 
    /* Compensate for drift with accelerometer data. */
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    
    int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
    
    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {
        // Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
        *pitch = *pitch * 0.98 + pitchAcc * 0.02;
 
        // Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
        *roll = *roll * 0.98 + rollAcc * 0.02;
    }
    

    /* plot the data to understance the pattern. */
    /*
    Serial.println(*pitch);Serial.print(",");
    Serial.println(*roll);Serial.print(",");
    */

    /* first time to set pitch & roll values to be used later in feedback. */
    if(first_time == false){
      prev_pitch = *pitch;
      prev_roll = *roll;
      first_time = true;
      return;
    }
    

    /* determine which direction we will go and by which angle. */
    /* I choose scale to each angle degree , I know that from the plotting pattern. */
    if(abs(*pitch - prev_pitch) > abs(*roll - prev_roll) && abs(*pitch - prev_pitch) > tolerance){
      dir = 0; 
      angle = (*pitch - prev_pitch) * 2;
    }
    else if(abs(*roll - prev_roll) > tolerance){
      dir = 1;
      angle = (*roll - prev_roll) * 2;
    }
    else{
      /* means no change at any servo (stationary). */
      dir = -1;
    }

    prev_pitch = *pitch;
    prev_roll = *roll;
    
    rotate(); 
} 


void rotate(){
  
  /* DEBUG. */
  /* 
  Serial.print("rotate by angle = ");
  Serial.println(angle);
  Serial.print("in direction");
  Serial.println(dir);
  Serial.println("===========================");
  */
  
  
  if(dir == -1)return;
  
  servos[1 - dir].write(angles[1 - dir]);

  int sign = 1;
  if(angle < 0) sign = -1;
  for(int i=0;i<abs(angle);i++){
    
    servos[dir].write((angles[dir] + sign * 1 + 181) % 181);
    angles[dir] = (angles[dir] + sign * 1 + 181) % 181;
    delay(20);
  }
  
}

void setupMPU(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                             
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                             
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                             
}


void read_values(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true); // request a total of 14 registers

  readAcceleration();
  readTempreture();
  readGyro();  
}

void readAcceleration() {
  while (Wire.available() < 6);
  // read the acceleration values , note that the value is divided into two adjacent register
  // each one has the first half bits so we should concatenate them by shifting and oring
  acc[0] = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  acc[1] = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acc[2] = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L);
}


void readTempreture() {
  while (Wire.available() < 1);
  temp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
}

void readGyro() {
  while (Wire.available() < 6);
  // read the gyro values , note that the value is divided into two adjacent register
  // each one has the first half bits so we should concatenate them by shifting and oring
  gyro[0] = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyro[1] = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyro[2] = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  
  // Plot
  /*
  Serial.print(gyro[0]);Serial.print(",");
  Serial.print(gyro[1]);Serial.print(",");
  Serial.println(gyro[2]);
  */ 
}
