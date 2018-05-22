//Segway Code

//Gyro - Arduino UNO R3
//VCC  -  3.3V
//GND  -  GND
//SDA  - 10K Resistor - A4 
//SCL  - 10K Resistor - A5
//INT - port-2

#include <Wire.h>
#include <SoftwareSerial.h>
#include "MPU6050_6Axis_MotionApps20.h"

//Declaring global variables
float gyro_x, gyro_y, gyro_z;
float gyro_x_offset, gyro_y_offset, gyro_z_offset;
boolean set_gyro_angles;
float gyro_angle = 0;

float desired_y_angle = 0;

float acc_x, acc_y, acc_z, acc_total_vector;
float acc_x_offset, acc_y_offset, acc_z_offset;
float angle_roll_acc, angle_pitch_acc;


float acc_filter[7] = {0,0,0,0,0,0,0};

float angle_pitch, angle_roll;
float angle_pitch_output, angle_roll_output;
float angle = 0, previous_angle = 0;
signed char motor1percent, motor2percent;

long loop_timer;
int temp;

int potPin = 2;

#define POT_MAX 1023 //the physical limits of the Potentiometer
#define POT_MIN 0

int steer = 0;
#define STEER_MAX 27

//Note: Set Sabertooth dip switches on the board for simplified serial and 9600 Baudrate. 
#define SABER_TX_PIN  13 //Digital pin 13 is serial transmit pin to sabertooth
#define SABER_RX_PIN  12 //Not used but still initialised, Digital pin 12 is serial receive from Sabertooth
#define SABER_BAUDRATE  9600 //set baudrate to match sabertooth dip settings

//simplifierd serial limits for each motor
#define MOTOR1_FULL_FORWARD 1
#define MOTOR1_FULL_REVERSE 127
#define MOTOR2_FULL_FORWARD 128
#define MOTOR2_FULL_REVERSE 255

#define CAL 100

float pGain = 15, iGain = 1, dGain = 1;
float PID_error = 0;

#define SABER_ALL_STOP  0 //motor command to send when issuing full stop command

SoftwareSerial SaberSerial = SoftwareSerial (SABER_RX_PIN, SABER_TX_PIN );
                                             
void initSabertooth (void)  { //initialize software to communicate with sabertooth 
  pinMode ( SABER_TX_PIN, OUTPUT );
  SaberSerial.begin( SABER_BAUDRATE );
  SaberSerial.write((byte) 0);   //kill motors when first switched on
}


void setup() {
  //initSabertooth();
  
  Serial.begin(38400);
  Serial.println("Calibrating");
  Wire.begin(); //Start I2C as master
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 
  for (int cal_int = 0; cal_int < CAL ; cal_int ++){                  //Read the raw acc and gyro data from the MPU-6050 for 1000 times
    Serial.println(cal_int);
    read_mpu_6050_data();                                             
    gyro_x_offset += gyro_x;                                              //Add the gyro x offset to the x_offset variable
    gyro_y_offset += gyro_y;                                              //Add the gyro y offset to the y_offset variable
    gyro_z_offset += gyro_z;                                              //Add the gyro z offset to the z_offset variable
    acc_x_offset += acc_x;                                              //Add the acc x offset to the x_offset variable
    acc_y_offset += acc_y;                                              //Add the acc y offset to the y_offset variable
    acc_z_offset += acc_z;                                              //Add the acc z offset to the z_offset variable
    delay(3);                                                          //Delay 3us to have 250Hz for-loop
  }

  // divide by 1000 to get average offset
  gyro_x_offset /= CAL;                                                 
  gyro_y_offset /= CAL;                                                 
  gyro_z_offset /= CAL;                                                 
  acc_x_offset /= CAL;                                                 
  acc_y_offset /= CAL;                                                 
  acc_z_offset /= CAL;  
  
  Serial.print("X: ");Serial.print(gyro_x_offset);
  Serial.print(" Y: ");Serial.print(gyro_y_offset);
  Serial.print(" Z: ");Serial.println(gyro_z_offset);
  delay(2000);
  loop_timer = micros();                                               //Reset the loop timer
}

void loop(){
  
  read_mpu_6050_data();   
  //Subtract the offset values from the raw gyro values                                            
  //GYRO_Y and ACC_X
   acc_x -= acc_x_offset;
  //Serial.print((int)acc_x);Serial.print("\t");
   acc_x /= 4096;
   //Serial.print(acc_x);Serial.print("\t");
    //acc_y -= acc_y_offset;
    //acc_y /= 4096;
   //Serial.print(acc_x * 180/3.142);Serial.print("\t");
   //float acc_pitch = asin((float)fax)* 57.296;
//   acc_z -= acc_z_offset;
   //acc_z /= 16384;

   
   float acc_angle = asin((float)acc_x)* 57.296;
   //Serial.print(gyro_x);Serial.print("\t");
   gyro_x -= gyro_x_offset;
   //Serial.print(gyro_x);Serial.print("\t");
   gyro_x /= 65.5;
   
   gyro_y -= gyro_y_offset;
   gyro_y /= 65.5;
   gyro_z -= gyro_z_offset;
   gyro_z /= 65.5;

   
//Serial.print(gyro_x);Serial.print("\t");
   Serial.print(acc_x*180/3.142);Serial.print("\t");
   //Serial.print(acc_y*180/3.142);Serial.print("\t");
   //Serial.print(acc_z*180/3.142);Serial.print("\t");
   //Serial.print(gyro_x*180/3.142);Serial.println("\t");
   Serial.print(gyro_y*180/3.142);Serial.println("\t");
   //Serial.print(gyro_z);Serial.println("\t");
   gyro_angle += gyro_y * 0.004;
   //Serial.print(gyro_angle);Serial.print("\t");
   //Serial.print(acc_angle);Serial.print("\t");


   
   float filter_const = 0.98;
   angle = (gyro_angle + previous_angle) * filter_const + (1-filter_const) * acc_angle;
   previous_angle = angle;
   //Serial.println(angle*3);
    
   for (int i = 0; i < 6; i++){
    acc_filter [i] = acc_filter [i+1];
   }
   acc_filter[6] = acc_y;
   float filter_result = (float) ((-2*acc_filter[0]) + (3*acc_filter[1]) + (6*acc_filter[2]) + (7*acc_filter[3]) + (6*acc_filter[4]) + (3*acc_filter[5]) + (-2*acc_filter[6]))/21;
   
   angle_pitch = asin(acc_x)* 360;    

   //Serial.print(filter_result);Serial.print("\t");
   //Serial.println(angle_pitch);
      
  //Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5) //https://electronics.stackexchange.com/questions/39714/how-to-read-a-gyro-accelerometer?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa 
  //Gyro = (Delta Theta)/(Delta Time) -> Angle = Gyro*Time
   angle_pitch += (float)(gyro_y/131) * 0.004;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  //angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  //angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  //angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
// 
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value

  //PID_error = angle_pitch_output - (y_offset*0.0000611);
  float PID_output = pGain * PID_error;
  
//  Serial.print(" | Pitch  = "); Serial.print(angle_pitch);
//  Serial.print(" | Roll  = "); Serial.print(angle_roll);
//  Serial.print(" Error "); Serial.print(PID_error);
//  Serial.print(" Output "); Serial.println(PID_output);
  
  
  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();//Reset the loop timer
  
}


void setup_mpu_6050_registers(){
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


void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  if(Wire.available() < 14) Serial.println("connection lost");
  while(Wire.available() < 14);  
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();                                  
  temp = Wire.read()<<8|Wire.read();                                   
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();       

}
void calculateTurn(){
  int potReading = analogRead(potPin); //Returns between 0 and 1023
  steer = pow(3.0,((2/(POT_MAX-POT_MIN))*(potReading-((POT_MAX+POT_MIN)/2)))); //Cubic representation of pot reading https://www.desmos.com/calculator/9fptmmiiqp
  steer *= STEER_MAX;
}
void setMotors (){
  unsigned char motor1, motor2;


  
  float multiplier = 1; //Caps the percent at 100
  if(abs(motor1percent) >= abs(motor2percent) && abs(motor1percent) > 100){
    multiplier = motor2percent/motor1percent;
  } else if (abs(motor2percent) >= abs(motor2percent) && abs(motor2percent) > 100){
    multiplier = motor1percent/motor2percent;
  }
  motor1percent *= multiplier;
  motor2percent *= multiplier;
  motor1 = motor1percent * ((MOTOR1_FULL_REVERSE-MOTOR1_FULL_FORWARD)/100); //Gives a num between 1 and 127
  motor2 = motor2percent * ((MOTOR2_FULL_REVERSE-MOTOR2_FULL_FORWARD)/100) + MOTOR2_FULL_FORWARD;
  SaberSerial.write((byte) motor1);
  SaberSerial.write((byte) motor2);
}

















