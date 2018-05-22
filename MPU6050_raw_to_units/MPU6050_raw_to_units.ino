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

int16_t ax, ay, az;
int16_t gx, gy, gz;

float dx = 0;
float x7[7] = {0,0,0,0,0,0,0};
float x_offset = 0, y_offset = 0, z_offset = 0;
float gxo = 0, gyo = 0, gzo = 0; //gyro offsets
float pitch = 0, roll = 0;
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
    Serial.begin(38400);

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
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//    Serial.print(ax);Serial.print("\t");
//    Serial.print(ay);Serial.print("\t");
//    Serial.print(az);Serial.print("\t");
//    Serial.print(gx);Serial.print("\t");
//    Serial.print(gy);Serial.print("\t");
//    Serial.print(gz);Serial.println("\t");
    ax -= x_offset;
    ay -= y_offset;
    //Serial.print(ax);Serial.print("\t");
    az -= z_offset;
    gx -= gxo;
    gy -= gyo;
    gz -= gzo;
   
   float fax = ((float) (ax)) / 16384;
   if (fax > 1) {
    fax = 1;
   }
   if (fax < -1) {
    fax = -1;
   }
   float fay = ((float) (ay)) / 16384;
   //Serial.print(fax);Serial.print("\t");
   float faz = ((float) (az)) / 16384;
     
//   for (int i = 5; i > 0; i--){
//      x7[i+1] = x7[i];
//    }
//    x7[0] = fax;
//    float curAcc = 0;
//    for (int i = 0; i < 7; i++){
//      curAcc += x7[i];
//    }
//    curAcc /= 7;
//    float curVel = curAcc * 0.004;
//    dx += curVel*(0.004);

    float fgx = ((float) (gx)) / 131;
    float fgy = ((float) (gy)) / 131;
    float fgz = ((float) (gz)) / 131;
    //float acc_total_vector = sqrt((fax*fax)+(fay*fay)+(faz*faz));
    
    float acc_pitch = asin((float)fax)* 57.296;
    //Serial.print(acc_pitch);Serial.println("\t");
    pitch += fgy * 0.004;
    Serial.print(pitch);Serial.println("\t");
    pitch = pitch * 0.9996 + acc_pitch * 0.0004;
    //Serial.println(pitch);
    roll +=  fgx * 0.004;

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
  int calAmount = 1000;
  for(int i = 0; i < calAmount; i++){
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    x_offset += ax;
    y_offset += ay;
    z_offset += az;
    gxo += gx;
    gyo += gy;
    gzo += gz;
  }
  x_offset /= calAmount;
  y_offset /= calAmount;
  z_offset /= calAmount;
  gxo /= calAmount; gyo /= calAmount; gzo /= calAmount;
  delay (3);
}

