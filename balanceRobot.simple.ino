/************************************************************************
*                                                                           
*                                                                    
*                          Self Balance Robot                                                           
*                              main.cpp                                      
*                                                                           
*                                MAIN                                      
*                                                                           
*                 Copyright (C) 2012 Ulrik Hoerlyk Hjort                   
*                                                                         
*  Self Balance Robot is free software;  you can  redistribute it                          
*  and/or modify it under terms of the  GNU General Public License          
*  as published  by the Free Software  Foundation;  either version 2,       
*  or (at your option) any later version.                                   
*  Self Balance Robot is distributed in the hope that it will be                           
*  useful, but WITHOUT ANY WARRANTY;  without even the  implied warranty    
*  of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  
*  See the GNU General Public License for  more details.                    
*  You should have  received  a copy of the GNU General                     
*  Public License  distributed with Yolk.  If not, write  to  the  Free     
*  Software Foundation,  51  Franklin  Street,  Fifth  Floor, Boston,       
*  MA 02110 - 1301, USA.                                                    
************************************************************************/
#include<Wire.h>

#define ANGLE_LIMIT 70
#define A_IN 7
#define B_IN 12
#define PWM_A 5
#define PWM_B 6
#define STANDBY 8
#define MPU_ADDR 0x68

struct Mpu6050 {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t temp;
  int16_t gx;
  int16_t gy;
  int16_t gz;  
 };

/*************************************************
 *  PID controller for the motor speed
 *************************************************/
float pid(float residual, float dt) {
    const float kp=25;
    const float ki=2;;
    const float kd=0.8;
    
    static float p_i = 0;
    static float previousResidual = 0;

    float p_p = kp*residual;    
    p_i = p_i+(ki*residual);      
    float p_d = kd*((residual - previousResidual)/dt);          
    previousResidual = residual;
      
    return ((p_p + p_d) *0.010) +100.0;
}

/*************************************************
 * Complementary filter for the MPU6050  
 * accelerometer and  gyroscope
 *************************************************/
float complementaryFilter(Mpu6050 data, float dt) {
  static float angle = 0.0;
  
  float accAngle =  atan2(data.ay , data.az) * 57.3;
  float gyroX = (data.gx - 128.1) / 131;  
  angle = 0.98 *(angle + gyroX*dt) + 0.02*accAngle;

  return angle;
}

/*************************************************
 * MPU data read
 *************************************************/
void mpuReadData(struct Mpu6050 *data) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,14,true);
  data->ax=Wire.read()<<8|Wire.read();
  data->ay=Wire.read()<<8|Wire.read();
  data->az=Wire.read()<<8|Wire.read();
  data->temp=Wire.read()<<8|Wire.read();
  data->gx=Wire.read()<<8|Wire.read();
  data->gy=Wire.read()<<8|Wire.read();
  data->gz=Wire.read()<<8|Wire.read();
}

/*************************************************
 *  Stop car
 *************************************************/  
void stop() {
  
  digitalWrite(A_IN,HIGH);
  digitalWrite(B_IN,LOW);
  digitalWrite(STANDBY,HIGH);
  analogWrite(PWM_A,0);
  analogWrite(PWM_B,0);
}

/*************************************************
 * Forward car
 *************************************************/
void forward(uint8_t speed) {
  
  digitalWrite(A_IN,0);
  digitalWrite(B_IN,0);
  analogWrite(PWM_A,speed);
  analogWrite(PWM_B,speed);
}

/*************************************************
 * Reverse car
 *************************************************/
void back(uint8_t speed) {
  
  digitalWrite(A_IN,1);
  digitalWrite(B_IN,1);
  analogWrite(PWM_A,speed);
  analogWrite(PWM_B,speed);
}

/*************************************************
 * Setup MPU and pins
 *************************************************/
void setup() {
  
  // Setup MPU
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Setup pin modes
  pinMode(A_IN,OUTPUT);
  pinMode(B_IN,OUTPUT);
  pinMode(PWM_A,OUTPUT);
  pinMode(PWM_B,OUTPUT);
  pinMode(STANDBY,OUTPUT);
  stop();
}

/*************************************************
 * Main loop
 *************************************************/
void loop() {  
  static uint32_t previousTime = 0;

  Mpu6050 data;
  mpuReadData(&data);  

  /* Delta time for last MPU reading */
  uint32_t t = millis();  
  float dt = (t - previousTime) / 1000.0; 
  previousTime = t;
  
  float angle = complementaryFilter(data, dt);  

  if ((angle < ANGLE_LIMIT) && (angle > -ANGLE_LIMIT)) {
      float p = pid(abs(angle),dt); 
      if (angle > 0) {
         forward(p);  
      } else if (angle < 0) {     
         back(p);
      }  else {
         stop();
      }       
  } else { /* Tilt angle to big to recover. Give up and stop. */
    stop(); 
  }
}
