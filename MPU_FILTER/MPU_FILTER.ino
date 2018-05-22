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
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

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
float gyro_integral = 0; 
float cur_angle = 0;

int cal_amt = 100;

float ypr[3];
Quaternion q;
VectorFloat gravity;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


#define LED_PIN 13
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */
  
    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
    Serial.println("Calibrating");
    calibrate();
}

void loop() {
   long loop_timer = micros();
   // read raw accel/gyro measurements from device
   //acc_x and gyro_y
   accelgyro.getMotion6(&acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);
   acc_x -= acc_x_offset;
    
   float acc_x_rad = ((float) (acc_x)) / 16384;
   if (acc_x_rad > 1) {
    acc_x_rad = 1;
   }
   if (acc_x_rad < -1) {
    acc_x_rad = -1;
   }
   gyro_y -= gyro_y_offset;
   float gyro_y_deg = ((float) (gyro_y)) / 131;
   
   acc_angle = asin(acc_x_rad)* 57.296;
   Serial.print(acc_angle);Serial.print("\t");
   gyro_angle = gyro_y_deg * 0.004;
   gyro_integral += gyro_angle;

   Serial.print(gyro_angle);Serial.print("\t");
   Serial.print(gyro_integral);Serial.print("\t");
   
   cur_angle = (gyro_angle+cur_angle) * 0.9 + acc_angle * 0.1;
   
   Serial.println(cur_angle);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
//        Serial.print("a/g:\t");
//       Serial.print("xraw/xAcc/xVel:\t");
        //Serial.print(fax); Serial.print("\t");
       //Serial.print(fay); Serial.print("\t");
        //Serial.print(faz); Serial.print("\t");
//        Serial.print(curAcc); Serial.print("\t");
//        Serial.println(curVel);
        //Serial.print(fgx); Serial.print("\t");
        //Serial.print(fgy); Serial.print("\t");
//        //Serial.print(roll);Serial.print("\t");
//        Serial.println(pitch);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    while (loop_timer - micros() < 4000){
      
    }
}
void calibrate(){
  
  for (int cal_int = 0; cal_int < cal_amt ; cal_int ++){                  //Read the raw acc and gyro data from the MPU-6050 for 1000 times
    Serial.println(cal_int);
    accelgyro.getMotion6(&acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);                                           
    gyro_x_offset += gyro_x;                                              //Add the gyro x offset to the x_offset variable
    gyro_y_offset += gyro_y;                                              //Add the gyro y offset to the y_offset variable
    gyro_z_offset += gyro_z;                                              //Add the gyro z offset to the z_offset variable
    acc_x_offset += acc_x;                                              //Add the acc x offset to the x_offset variable
    acc_y_offset += acc_y;                                              //Add the acc y offset to the y_offset variable
    acc_z_offset += acc_z;                                              //Add the acc z offset to the z_offset variable
    delay(3);                                                          //Delay 3us to have 250Hz for-loop
  }

  // divide by cal_amt to get average offset
  gyro_x_offset /= cal_amt;                                                 
  gyro_y_offset /= cal_amt;                                                 
  gyro_z_offset /= cal_amt;                                                 
  acc_x_offset /= cal_amt;                                                 
  acc_y_offset /= cal_amt;                                                 
  acc_z_offset /= cal_amt;  
  delay (3);
}

