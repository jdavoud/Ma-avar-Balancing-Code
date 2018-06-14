// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "Arduino.h"
#include "I2Cdev.h"
//#include "MPU6050.h"
#include <SoftwareSerial.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>
#include <Sabertooth.h>
#include <Filters.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t acc_x, acc_y, acc_z;
int16_t gyro_x, gyro_y, gyro_z;

float acc_x_offset = 0, acc_y_offset = 0, acc_z_offset = 0;
float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0; //gyro offsets

float acc_angle = 0;
float gyro_angle = 0;
double cur_angle = 0;
double desired_angle = 0;
double balance = 0; 
float error = 0, last_error = 0;

float motor1percent, motor2percent;

int potPin = 2;

#define POT_MAX 276 //the physical limits of the Potentiometer
#define POT_MIN 153

float  steer = 0;
#define STEER_MAX 10

//double kP = 19, kI = 10, kD = 0.01825; //kP = 28 and kD = 0.014 was good before we saw the lateral shaking
double kP = 65, kI = 15, kD = 0.03; //kD = 0.01675; //30 and 0.018 was good //a good version was 39 and 0.018 with 1.75 hz, 45 and 0.018 and 1.6 was decent
//39 and 0.024 with 1.75 is best weve seen
//39 and 0.024 with 1.4 was good now
float pTerm = 0, iTerm = 0, dTerm = 0;


boolean buttonState = false;
boolean prevButtonState = false;
int buttonStateSize = 15;
boolean buttonStates[15];
int buttonPressLength = 0;
int buttonPin = 0; //Was 2, now 0, the label is incorrect

int greenLED = 3;
int redLED = 4;

int cal_amt = 50;

float ypr[3];
Quaternion q;
VectorFloat gravity;

bool runMotors = true;

//Note: Set Sabertooth dip switches on the board for simplified serial and 9600 Baudrate. 
#define SABER_TX_PIN  13 //Digital pin 13 is serial transmit pin to sabertooth
#define SABER_RX_PIN  12 //Not used but still initialised, Digital pin 12 is serial receive from Sabertooth
#define SABER_BAUDRATE  9600 //set baudrate to match sabertooth dip settings

//simplifierd serial limits for each motor
#define MOTOR1_FULL_FORWARD 1
#define MOTOR1_FULL_REVERSE 127
#define MOTOR2_FULL_FORWARD 128
#define MOTOR2_FULL_REVERSE 255

#define ANGLE_DEAD_ZONE 0.2 //TEST and update
#define ANGLE_MAX 15
#define ANGLE_MIN -12

#define FORWARD_REST_ANGLE = -20;

#define LED_PIN 13

bool blinkState = false;

int runningState = 0; //0 = Not calibrated, 1 = calibrated but off, 2 = calibrated and running

FilterTwoPole filter;
RunningStatistics inputStats, outputStats;

float frequency = 1.2;


PID SegwayPID (&cur_angle, &balance, &desired_angle, kP, kI, kD, DIRECT);

SoftwareSerial SaberSerial = SoftwareSerial (SABER_RX_PIN, SABER_TX_PIN );

Sabertooth ST(128, SaberSerial);

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    SaberSerial.begin(9600);

    delay(1000);
    
    Serial.begin(9600);

    ST.motor(1,0);
    ST.motor(2,0);
    
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    
    SegwayPID.SetOutputLimits(-1000,1000);
    SegwayPID.SetMode(AUTOMATIC);
    SegwayPID.SetSampleTime(4);
    
    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
    pinMode(greenLED, OUTPUT);
    pinMode(redLED, OUTPUT);
    pinMode(buttonPin, INPUT);

    
    Serial.println("Waiting for calibrate");
    
    filter.setAsFilter(LOWPASS_BUTTERWORTH, frequency);
    inputStats.setWindowSecs(10);
    outputStats.setWindowSecs(10);

    waitForCalibrate();
    calibrate();
    
}

void loop() {
   
   long loop_timer = micros();
   while(runningState == 0){
    Serial.println("Calibrate here ");
    waitForCalibrate();
    calibrate();
    loop_timer = micros();
   }
   checkState();
   acc_angle = 0;
   float acc_angle_y = 0;
   gyro_angle = 0;
   for(int i = 0; i < 10; i++){
    accelgyro.getMotion6(&acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z); //Gather raw data, acc_x and gyro_y are necessary
    acc_x -= acc_x_offset; //Calibrate relevant angle values
    acc_y -= acc_y_offset;
    gyro_y -= gyro_y_offset;
   
    float acc_x_g = ((float) (acc_x)) / 16384; //Convert to in terms of g
    float acc_y_g = ((float) (acc_y)) / 16384;
    if (acc_x_g > 1) { //Cap to between [1,-1] for inverse trig
      acc_x_g = 1;
    }
    if (acc_x_g < -1) {
      acc_x_g = -1;
    }
    if (acc_y_g > 1) { //Cap to between [1,-1] for inverse trig
      acc_y_g = 1;
    }
    if (acc_y_g < -1) {
      acc_y_g = -1;
    }
   //Gyro gives an angular rate value
    float gyro_y_deg = ((float) (gyro_y)) / 131; //Convert to in terms of degrees/sec
   
    acc_angle += asin(acc_x_g)* 57.296; //Find acceleration angle in terms of deg, 57.296 = 180/PI
    acc_angle_y += asin(acc_y_g)* 57.296;
    gyro_angle += gyro_y_deg * 0.004;
  }
  
  acc_angle /= 10;
  acc_angle_y /= 10;
  gyro_angle /= 10;
   //gyro_angle += gyro_y_deg * 0.004;
   
   //Serial.print("AccY: "); Serial.print(acc_angle_y);Serial.print("\t");
   //Serial.print("Gyro: ");Serial.print(gyro_angle+cur_angle);Serial.print("\t");
   float gyro_constant = 0.9;
   float prev_angle = cur_angle;
   cur_angle = (gyro_angle+cur_angle) * gyro_constant + acc_angle * (1-gyro_constant); //MPU6050 complementary filter, adjust drift from gyro with accel
   //cur_angle = (gyro_angle) * 0.98 + acc_angle * 0.02; //MPU6050 complementary filter, adjust drift from gyro with accel
   

   Serial.print("rawAng: ");Serial.print(cur_angle);Serial.print("\t");
   //Serial.print("Angle_dt: "); Serial.print((cur_angle-prev_angle)*0.04);Serial.print("\t");
   
   cur_angle = filter.input(cur_angle);

   Serial.print("filtAng: ");Serial.print(cur_angle);Serial.print("\t");
   
   SegwayPID.Compute(); 
   
   //Serial.print("PID: ");Serial.print(balance);Serial.print("\t");
   calculateTurn();
   //Serial.print("Str2: ");Serial.print(steer);Serial.print("\t");
   //steer = 0;

   setMotors();
   Serial.println();
   blinkState = !blinkState; // blink LED to indicate activity
   digitalWrite(LED_PIN, blinkState);
   while (loop_timer - micros() < 4000){}
}
void waitForCalibrate(){
  digitalWrite(greenLED, LOW);
  boolean redState = true;
  int redStateCounter = 0; 
  while(runningState == 0){
    buttonStates[0] = digitalRead(buttonPin);
    
    redStateCounter += 20;
    if(redStateCounter % 1000 == 0){ //blink the red led every 1 second
      redState = !redState;
    }
    digitalWrite(redLED, redState);
    if(buttonStates[0] == false && buttonStates[1] == true){ //when the button is pressed, move onto calibrate
    //if(buttonState){
      while (!digitalRead(buttonPin)){} //wait until unpressed
      digitalWrite(redLED, HIGH);
      runningState = 1;
    }
    delay(10);
    for (int i = 1; i < 5; i++){
      buttonStates[5-i] = buttonStates[5-i-1];
    }
  }
}
void calibrate(){
  Serial.println("Calibrating");
  for (int cal_int = 0; cal_int < cal_amt ; cal_int ++){      //Read the raw acc and gyro data from the MPU-6050 cal_amt times
    //Serial.println(cal_int);
    accelgyro.getMotion6(&acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);                                           
    gyro_x_offset += gyro_x;                                              //Add the gyro x offset to the x offset variable
    gyro_y_offset += gyro_y;                                              //Add the gyro y offset to the y offset variable
    gyro_z_offset += gyro_z;                                              //Add the gyro z offset to the z offset variable
    acc_x_offset += acc_x;                                              //Add the acc x offset to the x offset variable
    acc_y_offset += acc_y;                                              //Add the acc y offset to the y offset variable
    acc_z_offset += acc_z;                                              //Add the acc z offset to the z offset variable
    delay(3);                                                          //Delay 3us to have 250Hz for-loop
  }
  // divide by cal_amt to get average offset
  gyro_x_offset /= cal_amt;                                                 
  gyro_y_offset /= cal_amt;                                                 
  gyro_z_offset /= cal_amt;                                                 
  acc_x_offset /= cal_amt;                                                 
  acc_y_offset /= cal_amt;                                                 
  acc_z_offset /= cal_amt;  
  float acc_x_offset_g = acc_x_offset / 16384;
  Serial.print("X Offset: ");Serial.println(acc_x_offset_g);

  if(acc_x_offset_g >= 0.03 && acc_x_offset_g <= 0.07){ //do nothing this is okay
     runningState = 1;
  } else {
    Serial.print(" Out of range ");
    runningState = 0;
  }
  delay (3);
}
void checkState(){
  for(int i = buttonStateSize - 1; i > 0; i--){
    buttonStates[i] = buttonStates[i-1];
  }
  buttonStates[0] = digitalRead(buttonPin); Serial.print(buttonStates[0]);
  
  if(buttonStates[0] == false){ //0 = pressed, 1 = not pressed
    buttonPressLength += 1; //count how long the button has been pressed for
  } else { //button is unpressed
    if (buttonStates[1] == true && buttonStates[2] == false && buttonStates[3] == false){ //button is released
      Serial.print("Released after : ");Serial.print(buttonPressLength);
      if(buttonPressLength < 2){ //protects from stray values
        
      }
      else if(buttonPressLength < 20){ //quick press, just turn off
       Serial.print(" Short Press ");
        if(runningState == 2){ 
          runningState = 1;
          Serial.print(" new state 1 state in between");
        }
        else if(runningState == 1) {
          if(abs(cur_angle) <= 0.13){ 
            runningState = 2; 
            Serial.print(" new state 2 ");
          } else {
            digitalWrite(redLED, LOW);
            digitalWrite(greenLED, HIGH);
            delay(15);
            digitalWrite(redLED, HIGH);
            digitalWrite(greenLED, LOW);
          }
        }
        Serial.print(" Running State: ") ;Serial.print(runningState);
        
      } else { //long press, go back to before calibrate stage
        Serial.print(" Long Press ");
        runningState = 0;
      }
      buttonPressLength = 0; 
    }
    
  }
  if(runningState == 1){
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, LOW);
  }
  else if(runningState == 2){
    digitalWrite(greenLED, HIGH);
    digitalWrite(redLED, LOW);
  }
}
void calculateTurn(){
  int potReading = analogRead(potPin); //Returns between 0 and 1023
  //Serial.print("Pot: ");Serial.print(potReading);Serial.print("\t");
  steer = pow(((2/((float)(POT_MAX-POT_MIN)))*(potReading-(((float)(POT_MAX+POT_MIN))/2))),3.0); //Cubic representation of pot reading https://www.desmos.com/calculator/9fptmmiiqp
  steer *= (STEER_MAX);
  //Serial.print("str^3: ");Serial.print(steer);Serial.print("\t");
  steer = (float)((potReading-(((float)(POT_MAX+POT_MIN))/2)))/(2.5);
  if(potReading - POT_MAX > 10 || POT_MIN - potReading > 10){
    steer = 0;
  }
  //Serial.print("str1: ");Serial.print(steer);Serial.print("\t");
  if (steer < 0.03 && steer > -0.03) { steer = 0; }
}

float tunePIDwithPot (int pin){ //Useful if someone wanted to tune a potentiometer with a potentiometer, but not implemented
  float potReading = analogRead(pin);
  potReading = (potReading / 100) * (potReading / 100); 
  //Serial.print(potReading);
  SegwayPID.SetTunings(kP,kI,kD);
  return potReading;
  
}
void setMotors (){
  
  int motor1, motor2; //final numbers for the motors
  motor1percent = balance + steer; //may be outside of [-100,100]
  motor2percent = balance - steer;
 
  //Cap the percent at 100
  float multiplier = 1; 
  if(abs(motor1percent) >= abs(motor2percent) && abs(motor1percent) > 100){
    multiplier = (100/motor1percent);
  } else if (abs(motor2percent) >= abs(motor2percent) && abs(motor2percent) > 100){
    multiplier = (100/motor2percent);
  }
  if (multiplier < 0) multiplier *= -1; // multiplier should be positive
  motor1percent *= multiplier;
  motor2percent *= multiplier;
  //Serial.print("Mtr %: ");Serial.print(motor1percent);Serial.print("\t");Serial.print(motor2percent);Serial.print("\t");
  
  //Gives a num up to 63
  motor1 = (int) (motor1percent * 1.26 + 0.5); //Int typecast rounds down, so add 0.5 makes it round to nearest int
  motor2 = (int) (motor2percent * 1.26 + 0.5);
 
  //If Arduino is tipped too far, don't move motors
  if (cur_angle > ANGLE_MAX || cur_angle < ANGLE_MIN){
    motor1 = 0; 
    motor2 = 0;
    runningState = 1;
  }
  
  //Serial.print("Val : ");Serial.print(motor1);Serial.print(" ");Serial.print(motor2);Serial.print("\t");
  
//  if(motor1 > 110 || motor1 < -110 || motor2 > 110 || motor2 < -110){
//    motor1 = 0;
//    motor2 = 0;
//    runMotors = false;
//  }
  
  if(motor1 > 70){
    motor1 = 70;
  }
  if(motor1 < -70){
    motor1 = -70;
  }
  if(motor2 > 70){
    motor2 = 70;
  }
  if(motor2 < -70){
    motor2 = -70;
  }
  
  
  //Serial.print("Mtr : ");Serial.print(motor1);Serial.print(" ");Serial.print(motor2);Serial.print("\t");
  
  //Set the motors
  
  if(runningState == 2){ //run the motors
    ST.motor(1, -motor1);
    ST.motor(2, -motor2);
  } else{ //not ready to run the motors
    ST.motor(1, 0);
    ST.motor(2, 0);
  }
}
