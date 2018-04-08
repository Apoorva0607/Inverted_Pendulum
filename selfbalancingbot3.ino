
#include <Wire.h>
#include "Kalman.h"
#define Sprint(a)(Serial.print(a))

   

Kalman kalmanX; // Create the Kalman 
   

  /* IMU Data */

  double accX, accY, accZ;

  //int16_t tempRaw;

  double gyroX, gyroY, gyroZ;
   

  double accXangle;//, accYangle; // Angle calculate using the accelerometer

  double gyroXangle;//, gyroYangle; // Angle calculate using the gyro

  double kalAngleX;//, kalAngleY; // Calculate the angle using a Kalman filter

   

  unsigned long timer;

  uint8_t i2cData[700]; // Buffer for I2C data*/

  float CurrentAngle;

   

  // Motor controller pins

  const int AIN1 = 5;  // (pwm) pin 3 connected to pin AIN1

  const int AIN2 = 3;  // (pwm) pin 9 connected to pin AIN2

  const int BIN1 = 11; // (pwm) pin 10 connected to pin BIN1 

  const int BIN2 = 10;  // (pwm) pin 11 connected to pin BIN2

  const int enb1= 6;   // pwm output to enable1 of motor 1

  const int enb2= 9;  //pwm output to enable2 of motor 2



   

  int speed;

   

  // PID

   

  float pTerm, iTerm, dTerm, integrated_error, last_error, error, Kp,Ki,Kd;

  const float K = 1.9*1.12;

  #define   GUARD_GAIN   20.0

   

  #define runEvery(t) for (static typeof(t) _lasttime;(typeof(t))((typeof(t))millis() - _lasttime) > (t);_lasttime += (t))

   

   

  void setup() { 

    pinMode(AIN1, OUTPUT); // set pins to output

    pinMode(AIN2, OUTPUT);

    pinMode(BIN1, OUTPUT);

    pinMode(BIN2, OUTPUT);

     pinMode(9, OUTPUT);

      pinMode( 6, OUTPUT);
      pinMode( 13,OUTPUT);

      //digitalWrite(9,1);
      //digitalWrite(6,1);

    Serial.begin(9600);

    Wire.begin();

    i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz

    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling

    i2cData[2] = 0x00; // Set Gyro Full Scale Range to Â±250deg/s

    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to Â±2g
    
    while(i2cWrite(0x19,i2cData,4,false)); // Write to all four registers at once

    while(i2cWrite(0x6B,0x01,true)); // PLL with X axis gyroscope reference and disable sleep mode

   

      while(i2cRead(0x75,i2cData,1));

    if(i2cData[0] != 0x68) { // Read "WHO_AM_I" register

      Sprint(F("Error reading sensor"));

      while(1);

    } 

    delay(100); // Wait for sensor to stabilize

   

    /* Set kalman and gyro starting angle */

    while(i2cRead(0x3B,i2cData,6));

    accX = ((i2cData[0] << 8) | i2cData[1]);

    accY = ((i2cData[2] << 8) | i2cData[3]);

    accZ = ((i2cData[4] << 8) | i2cData[5]);

    accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
    //Sprint("accXangle\t");
    //Sprint(accXangle);

   

    kalmanX.setAngle(accXangle); // Set starting angle

    gyroXangle = accXangle;

    timer = micros();

  }

   

  void loop() {

    runEvery(1)  // run code @ 40 Hz

    {

      dof();
      
      



      if (CurrentAngle <= 180.2 && CurrentAngle >= 179.8)

      {

        stop();

      }

      else{

      if (CurrentAngle < 230 && CurrentAngle > 130)

      {
       
      

      Pid();

      Motors();
     
      }

      else

      {

        stop();

      }

    }

    }

  }

   

  void Motors(){
     analogWrite( enb1,abs(speed));
    analogWrite (enb2,abs(speed*0.75));
   if (speed >= 0)

    {

      //forward
       speed=speed*0.66;
      digitalWrite(AIN1, 1);

      digitalWrite(AIN2, 0);

      digitalWrite(BIN1, 1);

      digitalWrite(BIN2, 0);
      
      

    }

    else

    {

      // 

     
     //speed = map(speed,0,-255,0,255);
     speed=abs(speed*0.66);
      digitalWrite(AIN1, 0);

      digitalWrite(AIN2, 1);

      digitalWrite(BIN1, 0);

      digitalWrite(BIN2, 1);
      

    }
     
   

  }

   

  void stop()

  {

    digitalWrite(AIN1, 0);

    digitalWrite(AIN2, 0);

    digitalWrite(BIN1, 0);

    digitalWrite(BIN2, 0);

  }

  

   

  void Pid(){

    error = 180 - CurrentAngle;  // 180 = level
    //Sprint("error\t");
     //Sprint(error);
      float Kp = map(analogRead(A0),0,1023,0,10000);
  float Ki = map(analogRead(A1),0,1023,0,5000);
  float Kd = map(analogRead(A2),0,1023,0,5000);
   Kp= Kp/100;
   Ki= Ki/100;
   Kd= Kd/100;
   Sprint("\tKp ");
    Sprint(Kp);
    Sprint("\tKi ");
    Sprint(Ki);
    Sprint("\tKd  ");
    Sprint(Kd);
    
     
      pTerm = Kp * error;

    integrated_error += error;

    iTerm = Ki * constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);

    dTerm = Kd * (error - last_error);

    last_error = error;

   speed = constrain(K*(pTerm + iTerm + dTerm), -255, 255);
   //speed=100;
   
    
     Sprint("\tspeed\t");
     Sprint(speed);
     Sprint("\n");
     
 
     



  }

   

  void dof()
  {

    while(i2cRead(0x3B,i2cData,14));

    accX = ((i2cData[0] << 8) | i2cData[1]);

    accY = ((i2cData[2] << 8) | i2cData[3]);

    accZ = ((i2cData[4] << 8) | i2cData[5]);

    //tempRaw = ((i2cData[6] << 8) | i2cData[7]); 

    gyroX = ((i2cData[8] << 8) | i2cData[9]);

    gyroY = ((i2cData[10] << 8) | i2cData[11]);

    gyroZ = ((i2cData[12] << 8) | i2cData[13]);

    accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
    //Sprint("accXangle\t");
    //Sprint(accXangle);
    
    

    double gyroXrate = (double)gyroX/131.0;
    
    

    CurrentAngle = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000);
  
   // Sprint("CurrentAngle\t");
    //Sprint(CurrentAngle);
     //Sprint("\n");
     

    timer = micros();

  }








