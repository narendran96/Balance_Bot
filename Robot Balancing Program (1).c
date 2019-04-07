// Robot Balancing Program for Arduino
// MPU 6050 set up with SDA A4 and SCL A5

#include<Wire.h>
#include<math.h>

// PID Control Variables

#define Kp 60.0
#define Ki 1.0
#define Kd 10.0

// Motor H-bridge Contrxsw2ol Pins

#define en1 6
#define in1 7
#define in2 8
#define en2 11
#define in3 9
#define in4 10

// I2C Address
#define I2C_address 0b1101000

// Complimentary Filter
#define gyroFil 0.98
#define accFil 0.02

// Initializing global variables

float accX;
float accY;
float accZ;
float accAng;

float gyroX;
float gyroY;
float gyroZ;
float gyroAng;

double angSum, prevAng, currAng, targAng, diffAng;
double prevTime, currTime, diffTime;
int motorSpeed;

void setup(){
  // Set Output Pins
  pinMode(en1, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  targAng = 0;
  Wire.begin();
  IMU_setup();
  Serial.begin(9600);
}

void loop(){
  currTime = millis();
  diffTime = currTime - prevTime;
  prevTime = currTime;
  // Complimentary filter
  currAng =  (double)  gyroFil*gyroAngle() + accFil*accAngle();
  Serial.println(currAng);
  PID();
  motorControl();
}

void IMU_setup(){
  Wire.beginTransmission(I2C_address); // connects MPU
  Wire.write(0x6B); // PU management 
  Wire.write(0b00000000); // Taking MPU off sleep mode
  Wire.endTransmission();
  // Set up the Gyro data 
  Wire.beginTransmission(I2C_address);
  Wire.write(0x1B);
  Wire.write(0x00000000);
  Wire.endTransmission();
  // Accelerometer setup
  Wire.beginTransmission(I2C_address);
  Wire.write(0x1C);
  Wire.write(0x00000000);
  Wire.endTransmission();
}

int gyroAngle(){
  Wire.beginTransmission(I2C_address);
  Wire.write(0x43);
  Wire.endTransmission();
  // Request 6 registers starting from 0x43 from MPU
  Wire.requestFrom(I2C_address, 6); 
  while(Wire.available() < 6); 
  gyroX = Wire.read()<<8|Wire.read();
  gyroY = Wire.read()<<8|Wire.read();
  gyroZ = Wire.read()<<8|Wire.read();
  // Need to scale down gyro 16 bit 2's compliment value to 250 deg/s range
  // Only interested in x direction angle 
  gyroX /= 131.0; 
  gyroX = map(gyroX, -32768, 32767, -250, 250);
  gyroAng = (float) prevAng + gyroX*diffTime/1000;
  //Serial.println(gyroX);
  return gyroAng;

}

int accAngle(){
  Wire.beginTransmission(I2C_address);
  Wire.write(0x3B);
  Wire.endTransmission();
  // Request 6 registers starting from 0x3B from MPU
  Wire.requestFrom(I2C_address, 6);
  while(Wire.available() < 6); 
  accX = Wire.read()<<8|Wire.read();
  accY = Wire.read()<<8|Wire.read();
  accZ = Wire.read()<<8|Wire.read();
  // Calculates acceleration angle and converts to degrees
  accAng = atan2(accY, accZ) * 57.2958; 
  //Serial.println(accAng);
  return accAng;
}

void PID(){
  diffAng = currAng - targAng;
  // angSum used for Ki integration component
  angSum += diffAng;
  // need to constrain angle to certain range 
  angSum = constrain(angSum, -250, 250);
  // constrain motorspeed to range of PWM signal
  //motorSpeed = constrain(motorSpeed, -250, 250);
  motorSpeed = Kp*(diffAng) + Ki*(angSum)*diffTime + Kd*(currAng-prevAng)/diffTime;
  prevAng = currAng;
  Serial.println(motorSpeed);
} 

void motorControl(){
  // Need to set motor values based on motor speed
  if(currAng > 0){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(en1, motorSpeed);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(en2, motorSpeed);
  }
  else{
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(en1, motorSpeed);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(en2, motorSpeed);
  }
}

