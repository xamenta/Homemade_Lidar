#include <Arduino.h>
#include "ComponentsFile.h"


void setup() 
{
  Wire.begin();
  Serial.begin(115200);
  StepperInitialize(1000.0, 900.0, 1000.0);
  TfInitialize();
  ServoInitialization();
  // Serial.println("Initialization Complete");
  delay(5000);
}

void loop() 
{
  bool first_servo = true;
  bool first_stepper = true;
  for (int stp = stepper_start; stp <= stepper_end; stp+= stepper_division)
  {
    first_servo = true;
    stepper_angle = stp;
    MoveToDeg(stp);
    if (first_stepper)
      {
        delay(50);
        first_stepper = false;
      }
      else
      {
        delay(1);
      }

    for (int srv = servo_start; srv <= servo_end; srv +=servo_division)
    {
      servo_angle = 90 - srv;
      V_axis.write(srv); //SetAngle(srv);
      if (first_servo)
      {
        delay(50);
        first_servo = false;
      }
      else
      {
        delay(1);
      }
      Tfdistance = 0;
      for (int i = 0; i < avg_reading; i++)
      {
        Tfdistance = Tfdistance + TfReturnDistance();
        delay(2);
      }
      Tfdistance = Tfdistance/avg_reading;
      
      if (Tfdistance < 30000)
      {
        X_inertia = Tfdistance*cos(servo_angle*MY_PI/180)*cos(stp*MY_PI/180);
        Y_inertia = Tfdistance*cos(servo_angle*MY_PI/180)*sin(stp*MY_PI/180);
        Z_inertia = Tfdistance*sin(servo_angle*MY_PI/180);
        obstacle_pos.x = X_inertia;
        obstacle_pos.y = Y_inertia;
        obstacle_pos.z = Z_inertia;
        if (data_send)
        {
          byte SerSnd[12];
          doubleConv cnrtrSnd;
          cnrtrSnd.asdouble = obstacle_pos.x;
          SerSnd[0] = cnrtrSnd.asBytes[0]; SerSnd[1] = cnrtrSnd.asBytes[1]; SerSnd[2] = cnrtrSnd.asBytes[2]; SerSnd[3] = cnrtrSnd.asBytes[3]; 
          cnrtrSnd.asdouble = obstacle_pos.y;
          SerSnd[4] = cnrtrSnd.asBytes[0]; SerSnd[5] = cnrtrSnd.asBytes[1]; SerSnd[6] = cnrtrSnd.asBytes[2]; SerSnd[7] = cnrtrSnd.asBytes[3]; 
          cnrtrSnd.asdouble = obstacle_pos.z;
          SerSnd[8] = cnrtrSnd.asBytes[0]; SerSnd[9] = cnrtrSnd.asBytes[1]; SerSnd[10] = cnrtrSnd.asBytes[2]; SerSnd[11] = cnrtrSnd.asBytes[3]; 
          Serial.write(SerSnd, sizeof(SerSnd));
          delay(5);
        }
        else
        {
          // Serial.print("Servo Angle : ");
          // Serial.print(srv);
          // Serial.print(" : Stepper Angle : ");
          // Serial.print(stp);
          // Serial.print(" : Obstacle point [x,y,z]: [");
          Serial.print("Obstacle point [x,y,z]: [");
          Serial.print(obstacle_pos.x);
          Serial.print(" , ");
          Serial.print(obstacle_pos.y);
          Serial.print(" , ");
          Serial.print(obstacle_pos.z);
          Serial.println("] ");
          // delay(10);
        }
        
      }
    }
  }
}