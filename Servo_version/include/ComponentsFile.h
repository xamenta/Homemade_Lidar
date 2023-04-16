//:::::::::::::::::::::::::: NOTES :::::::::::::::::::::::::://

/*
All the units used in the code are in SI units. Distances measured by the Ultrasonic 
sensors and Time of Flight VL53L0X sensors are in metres 
*/
#ifndef COMP_DRIVER_H
#define COMP_DRIVER_H

//:::::::::::::::::::::::::: INCLUSIONS :::::::::::::::::::::::::://
#include <AccelStepper.h>
#include <Servo.h>
#include <TFLI2C.h>
#include <Wire.h>

//:::::::::::::::::::::::::: DEFINE PINS :::::::::::::::::::::::::://
#define SDAPin A4           //I2C SDA Pin
#define SCLPin A5           //I2C SCL Pin
#define StepperPin1 12      //Stepper IN1 Pin
#define StepperPin2 8       //Stepper IN2 Pin
#define StepperPin3 7       //Stepper IN3 Pin
#define StepperPin4 4       //Stepper IN4 Pin
#define VServo 3            //Vertical Servo pin
#define HServo 5            //Horizontal Servo pin
#define Optpin 2            //Opto pin

//:::::::::::::::::::::::::: I2C Addresses :::::::::::::::::::::::::://
#define TFLuna_address 0x10     //I2C address of the TF Luna TOF sensor

//:::::::::::::::::::::::::: DEFINE GLOBAL COMPONENTS :::::::::::::::::::::::::://
TFLI2C lidar;
Servo V_axis;
Servo H_axis;

//:::::::::::::::::::::::::: STRUCTURES :::::::::::::::::::::::::://
struct PosData
{
	double x;
	double y;
	double z;
};

union doubleConv
{
    byte asBytes[4];
    double asdouble;
};

//:::::::::::::::::::::::::: DEFINE GLOBAL VARIABLES :::::::::::::::::::::::::://
PosData obstacle_pos;
int16_t Lidar_measure;
int16_t Lidar_flux;
int16_t Lidar_temp;
uint16_t Tfdistance;
double X_inertia;
double Y_inertia;
double Z_inertia;
double H_angle;
double V_angle;
int current_h_servo_angle;
int current_v_servo_angle;

//:::::::::::::::::::::::::: DEFINE PARAMETERS :::::::::::::::::::::::::://
const double MY_PI = 3.141592653589793238462643;    //Value of pi
uint16_t tfFrame = TFL_DEF_FPS;                     //Default frame rate for TF Luna
const int Gear_Ratio = 2;                           //Gear ratio between the stepper motor and the lidar rotor
const int Lidar_limit = 650;                        //Limit of the range of the Lidar in cm. This is done to fit 1 byte unsigned int
const int avg_reading = 1;                          //Number of readings to average
const bool data_send = false;                       //Boolean to select either to send data or print to serial
const bool both_run = true;                        //Boolean to specify if to do vertical sweep in both directions

const int H_division = 2;                           //Degree divisions for the movement of the horizontal sweep (even numbers between 2 and 30 degrees!)
const int H_end = 180;                              //Rotation range for the movement of the horizontal sweep (Steps of 30 degrees up to 360 degrees!)
const int H_start = -180;                           //Degree offset from 0 for the start point of the movement of the horizontal sweep
const int first_H_delay = 50;                       //Wait after first horizontal servo movement
const int H_delay = 1;                              //Wait after each horizontal servo movement
const double servo_h_speed = 50.0;                  //Horizontal servo speed in degrees per second

const int V_division = 1;                           //Degree divisions for the movement of the vertical sweep
const int V_end = 110;                              //Rotation range for the movement of the vertical sweep
const int V_start = 0;                              //Degree offset from 0 for the start point of the movement of the vertical sweep
const int first_V_delay = 150;                      //Wait after first vertical servo movement
const int V_delay = 1;                              //Wait after each vertical servo movement
const double servo_v_speed = 300.0;                 //Vertical servo speed in degrees per second


//:::::::::::::::::::::::::: VERTICAL SERVO :::::::::::::::::::::::::://
void ServoInitialization()
{
    V_axis.attach(VServo,500,2500);
    V_axis.write(0);
    current_v_servo_angle = 0;
    delay(1000);
    V_axis.write(110);
    current_v_servo_angle = 110;
    delay(1000);
    V_axis.write(90);
    current_v_servo_angle = 90;
    delay(1000);
    H_axis.attach(HServo,500,2500);
    H_axis.write(80);
    current_h_servo_angle = 80;
    delay(1000);
    H_axis.write(100);
    current_h_servo_angle = 100;
    delay(1000);
    H_axis.write(90);
    current_h_servo_angle = 90;
}

void SetVAngle(int deg, bool use_spd)
{
    if (use_spd)
    {
        int servo_dir = (deg - current_v_servo_angle) / (abs(deg - current_v_servo_angle));
        if (servo_dir > 1)
        {
            for (int srv = current_v_servo_angle; srv <= deg; srv++)
            {
                V_axis.write(srv);
                delay((int)(1000 / servo_v_speed));
            }
        }
        else
        {
            for (int srv = current_v_servo_angle; srv >= deg; srv--)
            {
                V_axis.write(srv);
                delay((int)(1000 / servo_v_speed));
            }
        }
        V_axis.write(deg);
        current_v_servo_angle = deg;
    }
    else
    {
        V_axis.write(deg);
        current_v_servo_angle = deg;
    }
}

void SetHAngle(int deg, bool use_spd)
{
    if (use_spd)
    {
        int servo_dir = (deg - current_h_servo_angle) / (abs(deg - current_h_servo_angle));
        if (servo_dir > 1)
        {
            for (int srv = current_h_servo_angle; srv <= deg; srv++)
            {
                H_axis.write(srv);
                delay((int)(1000 / servo_h_speed));
            }
        }
        else
        {
            for (int srv = current_h_servo_angle; srv >= deg; srv--)
            {
                H_axis.write(srv);
                delay((int)(1000 / servo_h_speed));
            }
        }
        H_axis.write(deg);
        current_h_servo_angle = deg;
    }
    else
    {
        H_axis.write(deg);
        current_h_servo_angle = deg;
    }
}

//:::::::::::::::::::::::::: TF LUNA SENSOR :::::::::::::::::::::::::://
void TfInitialize() 
{
    // Serial.print( "Device Address: ");
    // Serial.println(TFLuna_address);
    // Serial.print("System Reset: ");
    if( lidar.Soft_Reset(TFLuna_address))
    {
        // Serial.println( "Passed QC Test");
    }
    else
    {
        while (!lidar.Soft_Reset(TFLuna_address))
        {
            lidar.printStatus();  //If there is a problem print status for troubleshooting
            Serial.println("Setup failed");
        }
        // Serial.println( "Passed QC Test");    
    }
    delay(500);

    //  Set frame ratefor the device
    // Serial.print( "Set Frame Rate to: ");
    if( lidar.Set_Frame_Rate( tfFrame, TFLuna_address))
    {
    //   Serial.println(tfFrame);
    }
    else 
    {
        lidar.printStatus();
    }
    delay(500);
    
    //  Read frame rate back from the device to confirm
    // Serial.print( "Get Frame Rate: ");
    if( lidar.Get_Frame_Rate( tfFrame, TFLuna_address))
    {
    //   Serial.println(  tfFrame);
    }
    else
    {
        lidar.printStatus();
    }
    lidar.Set_Trig_Mode(TFLuna_address);
    lidar.Set_Enable(TFLuna_address);
    delay(500);
    
    // Serial.println("TF Luna Booted Successfully!!! ");
}

uint16_t TfReturnDistance()
{
    // If data is read without error...
    int16_t adr = TFLuna_address;
    uint16_t dist_cm;
    lidar.Set_Trigger(TFLuna_address);
    if( lidar.getData(Lidar_measure, Lidar_flux, Lidar_temp, adr))
    {
        Lidar_temp = int16_t( Lidar_temp/100);
        if (Lidar_measure > Lidar_limit)
        {
            dist_cm = (uint16_t)30000;
        }
        else
        {
            dist_cm = (uint16_t)Lidar_measure;
        }
    }
    else
    {
        lidar.printStatus();        // else, print error status.
        dist_cm = (uint16_t)30000;
    }
    return dist_cm;
}

//:::::::::::::::::::::::::: LOOP FUNCTION :::::::::::::::::::::::::://
void RunLoopBoth()
{
    bool first_V = true;
    bool first_H = true;
    bool move_v_down = true;
    bool move_h_right = true;
    if (move_h_right)
    {
        move_h_right = false;
        for (int stp = H_start; stp <= H_end; stp += H_division)
        {
            first_V = true;
            H_angle = stp / Gear_Ratio + 90;
            if (abs(H_angle - current_h_servo_angle) > 10)
            {
                SetHAngle(H_angle, true);
            }
            else
            {
                SetHAngle(H_angle, false);
            }
            if (first_H)
            {
                delay(first_H_delay);
                first_H = false;
            }
            else
            {
                delay(H_delay);
            }

            if (move_v_down)
            {
                move_v_down = false;
                for (int srv = V_start; srv <= V_end; srv += V_division)
                {
                    V_angle = 90 - srv;
                    if (abs(V_angle - current_v_servo_angle) > 10)
                    {
                        SetVAngle(srv, true);
                    }
                    else
                    {
                        SetVAngle(srv, false);
                    }
                    if (first_V)
                    {
                        delay(first_V_delay);
                        first_V = false;
                    }
                    else
                    {
                        delay(V_delay);
                    }
                    Tfdistance = 0;
                    for (int i = 0; i < avg_reading; i++)
                    {
                        Tfdistance = Tfdistance + TfReturnDistance();
                        delay(2);
                    }
                    Tfdistance = Tfdistance / avg_reading;

                    if (Tfdistance < 30000)
                    {
                        X_inertia = Tfdistance * cos(V_angle * MY_PI / 180) * cos(stp * MY_PI / 180);
                        Y_inertia = Tfdistance * cos(V_angle * MY_PI / 180) * sin(stp * MY_PI / 180);
                        Z_inertia = Tfdistance * sin(V_angle * MY_PI / 180);
                        obstacle_pos.x = X_inertia;
                        obstacle_pos.y = -Y_inertia;
                        obstacle_pos.z = Z_inertia;
                        if (data_send)
                        {
                            byte SerSnd[12];
                            doubleConv cnrtrSnd;
                            cnrtrSnd.asdouble = obstacle_pos.x;
                            SerSnd[0] = cnrtrSnd.asBytes[0];
                            SerSnd[1] = cnrtrSnd.asBytes[1];
                            SerSnd[2] = cnrtrSnd.asBytes[2];
                            SerSnd[3] = cnrtrSnd.asBytes[3];
                            cnrtrSnd.asdouble = obstacle_pos.y;
                            SerSnd[4] = cnrtrSnd.asBytes[0];
                            SerSnd[5] = cnrtrSnd.asBytes[1];
                            SerSnd[6] = cnrtrSnd.asBytes[2];
                            SerSnd[7] = cnrtrSnd.asBytes[3];
                            cnrtrSnd.asdouble = obstacle_pos.z;
                            SerSnd[8] = cnrtrSnd.asBytes[0];
                            SerSnd[9] = cnrtrSnd.asBytes[1];
                            SerSnd[10] = cnrtrSnd.asBytes[2];
                            SerSnd[11] = cnrtrSnd.asBytes[3];
                            Serial.write(SerSnd, sizeof(SerSnd));
                            delay(5);
                        }
                        else
                        {
                            Serial.print("Obstacle point [x,y,z,s,stp]: [");
                            Serial.print(obstacle_pos.x);
                            Serial.print(" , ");
                            Serial.print(obstacle_pos.y);
                            Serial.print(" , ");
                            Serial.print(obstacle_pos.z);
                            Serial.print(" , ");
                            Serial.print(Tfdistance);
                            Serial.print(" , ");
                            Serial.print(stp);
                            Serial.println("] ");
                        }
                    }
                }
            }
            else
            {
                move_v_down = true;
                for (int srv = V_end; srv >= V_start; srv -= V_division)
                {
                    V_angle = 90 - srv;
                    if (abs(V_angle - current_v_servo_angle) > 10)
                    {
                        SetVAngle(srv, true);
                    }
                    else
                    {
                        SetVAngle(srv, false);
                    }
                    if (first_V)
                    {
                        delay(first_V_delay);
                        first_V = false;
                    }
                    else
                    {
                        delay(V_delay);
                    }
                    Tfdistance = 0;
                    for (int i = 0; i < avg_reading; i++)
                    {
                        Tfdistance = Tfdistance + TfReturnDistance();
                        delay(2);
                    }
                    Tfdistance = Tfdistance / avg_reading;

                    if (Tfdistance < 30000)
                    {
                        X_inertia = Tfdistance * cos(V_angle * MY_PI / 180) * cos(stp * MY_PI / 180);
                        Y_inertia = Tfdistance * cos(V_angle * MY_PI / 180) * sin(stp * MY_PI / 180);
                        Z_inertia = Tfdistance * sin(V_angle * MY_PI / 180);
                        obstacle_pos.x = X_inertia;
                        obstacle_pos.y = -Y_inertia;
                        obstacle_pos.z = Z_inertia;
                        if (data_send)
                        {
                            byte SerSnd[12];
                            doubleConv cnrtrSnd;
                            cnrtrSnd.asdouble = obstacle_pos.x;
                            SerSnd[0] = cnrtrSnd.asBytes[0];
                            SerSnd[1] = cnrtrSnd.asBytes[1];
                            SerSnd[2] = cnrtrSnd.asBytes[2];
                            SerSnd[3] = cnrtrSnd.asBytes[3];
                            cnrtrSnd.asdouble = obstacle_pos.y;
                            SerSnd[4] = cnrtrSnd.asBytes[0];
                            SerSnd[5] = cnrtrSnd.asBytes[1];
                            SerSnd[6] = cnrtrSnd.asBytes[2];
                            SerSnd[7] = cnrtrSnd.asBytes[3];
                            cnrtrSnd.asdouble = obstacle_pos.z;
                            SerSnd[8] = cnrtrSnd.asBytes[0];
                            SerSnd[9] = cnrtrSnd.asBytes[1];
                            SerSnd[10] = cnrtrSnd.asBytes[2];
                            SerSnd[11] = cnrtrSnd.asBytes[3];
                            Serial.write(SerSnd, sizeof(SerSnd));
                            delay(5);
                        }
                        else
                        {
                            Serial.print("Obstacle point [x,y,z,s,stp]: [");
                            Serial.print(obstacle_pos.x);
                            Serial.print(" , ");
                            Serial.print(obstacle_pos.y);
                            Serial.print(" , ");
                            Serial.print(obstacle_pos.z);
                            Serial.print(" , ");
                            Serial.print(Tfdistance);
                            Serial.print(" , ");
                            Serial.print(stp);
                            Serial.println("] ");
                        }
                    }
                }
            }
        }
    }
    else
    {
        move_h_right = true;
        for (int stp = H_end; stp <= H_start; stp -= H_division)
        {
            first_V = true;
            H_angle = stp / Gear_Ratio + 90;
            if (abs(H_angle - current_h_servo_angle) > 10)
            {
                SetHAngle(H_angle, true);
            }
            else
            {
                SetHAngle(H_angle, false);
            }
            if (first_H)
            {
                delay(first_H_delay);
                first_H = false;
            }
            else
            {
                delay(H_delay);
            }

            if (move_v_down)
            {
                move_v_down = false;
                for (int srv = V_start; srv <= V_end; srv += V_division)
                {
                    V_angle = 90 - srv;
                    if (abs(V_angle - current_v_servo_angle) > 10)
                    {
                        SetVAngle(srv, true);
                    }
                    else
                    {
                        SetVAngle(srv, false);
                    }
                    if (first_V)
                    {
                        delay(first_V_delay);
                        first_V = false;
                    }
                    else
                    {
                        delay(V_delay);
                    }
                    Tfdistance = 0;
                    for (int i = 0; i < avg_reading; i++)
                    {
                        Tfdistance = Tfdistance + TfReturnDistance();
                        delay(2);
                    }
                    Tfdistance = Tfdistance / avg_reading;

                    if (Tfdistance < 30000)
                    {
                        X_inertia = Tfdistance * cos(V_angle * MY_PI / 180) * cos(stp * MY_PI / 180);
                        Y_inertia = Tfdistance * cos(V_angle * MY_PI / 180) * sin(stp * MY_PI / 180);
                        Z_inertia = Tfdistance * sin(V_angle * MY_PI / 180);
                        obstacle_pos.x = X_inertia;
                        obstacle_pos.y = -Y_inertia;
                        obstacle_pos.z = Z_inertia;
                        if (data_send)
                        {
                            byte SerSnd[12];
                            doubleConv cnrtrSnd;
                            cnrtrSnd.asdouble = obstacle_pos.x;
                            SerSnd[0] = cnrtrSnd.asBytes[0];
                            SerSnd[1] = cnrtrSnd.asBytes[1];
                            SerSnd[2] = cnrtrSnd.asBytes[2];
                            SerSnd[3] = cnrtrSnd.asBytes[3];
                            cnrtrSnd.asdouble = obstacle_pos.y;
                            SerSnd[4] = cnrtrSnd.asBytes[0];
                            SerSnd[5] = cnrtrSnd.asBytes[1];
                            SerSnd[6] = cnrtrSnd.asBytes[2];
                            SerSnd[7] = cnrtrSnd.asBytes[3];
                            cnrtrSnd.asdouble = obstacle_pos.z;
                            SerSnd[8] = cnrtrSnd.asBytes[0];
                            SerSnd[9] = cnrtrSnd.asBytes[1];
                            SerSnd[10] = cnrtrSnd.asBytes[2];
                            SerSnd[11] = cnrtrSnd.asBytes[3];
                            Serial.write(SerSnd, sizeof(SerSnd));
                            delay(5);
                        }
                        else
                        {
                            Serial.print("Obstacle point [x,y,z,s,stp]: [");
                            Serial.print(obstacle_pos.x);
                            Serial.print(" , ");
                            Serial.print(obstacle_pos.y);
                            Serial.print(" , ");
                            Serial.print(obstacle_pos.z);
                            Serial.print(" , ");
                            Serial.print(Tfdistance);
                            Serial.print(" , ");
                            Serial.print(stp);
                            Serial.println("] ");
                        }
                    }
                }
            }
            else
            {
                move_v_down = true;
                for (int srv = V_end; srv >= V_start; srv -= V_division)
                {
                    V_angle = 90 - srv;
                    if (abs(V_angle - current_v_servo_angle) > 10)
                    {
                        SetVAngle(srv, true);
                    }
                    else
                    {
                        SetVAngle(srv, false);
                    }
                    if (first_V)
                    {
                        delay(first_V_delay);
                        first_V = false;
                    }
                    else
                    {
                        delay(V_delay);
                    }
                    Tfdistance = 0;
                    for (int i = 0; i < avg_reading; i++)
                    {
                        Tfdistance = Tfdistance + TfReturnDistance();
                        delay(2);
                    }
                    Tfdistance = Tfdistance / avg_reading;

                    if (Tfdistance < 30000)
                    {
                        X_inertia = Tfdistance * cos(V_angle * MY_PI / 180) * cos(stp * MY_PI / 180);
                        Y_inertia = Tfdistance * cos(V_angle * MY_PI / 180) * sin(stp * MY_PI / 180);
                        Z_inertia = Tfdistance * sin(V_angle * MY_PI / 180);
                        obstacle_pos.x = X_inertia;
                        obstacle_pos.y = -Y_inertia;
                        obstacle_pos.z = Z_inertia;
                        if (data_send)
                        {
                            byte SerSnd[12];
                            doubleConv cnrtrSnd;
                            cnrtrSnd.asdouble = obstacle_pos.x;
                            SerSnd[0] = cnrtrSnd.asBytes[0];
                            SerSnd[1] = cnrtrSnd.asBytes[1];
                            SerSnd[2] = cnrtrSnd.asBytes[2];
                            SerSnd[3] = cnrtrSnd.asBytes[3];
                            cnrtrSnd.asdouble = obstacle_pos.y;
                            SerSnd[4] = cnrtrSnd.asBytes[0];
                            SerSnd[5] = cnrtrSnd.asBytes[1];
                            SerSnd[6] = cnrtrSnd.asBytes[2];
                            SerSnd[7] = cnrtrSnd.asBytes[3];
                            cnrtrSnd.asdouble = obstacle_pos.z;
                            SerSnd[8] = cnrtrSnd.asBytes[0];
                            SerSnd[9] = cnrtrSnd.asBytes[1];
                            SerSnd[10] = cnrtrSnd.asBytes[2];
                            SerSnd[11] = cnrtrSnd.asBytes[3];
                            Serial.write(SerSnd, sizeof(SerSnd));
                            delay(5);
                        }
                        else
                        {
                            Serial.print("Obstacle point [x,y,z,s,stp]: [");
                            Serial.print(obstacle_pos.x);
                            Serial.print(" , ");
                            Serial.print(obstacle_pos.y);
                            Serial.print(" , ");
                            Serial.print(obstacle_pos.z);
                            Serial.print(" , ");
                            Serial.print(Tfdistance);
                            Serial.print(" , ");
                            Serial.print(stp);
                            Serial.println("] ");
                        }
                    }
                }
            }
        }
    }
}

void RunLoopOne()
{
    bool first_V = true;
    bool first_H = true;
    for (int stp = H_start; stp <= H_end; stp += H_division)
    {
        first_V = true;
        H_angle = stp/Gear_Ratio + 90;
        if (abs(H_angle - current_h_servo_angle) > 10)
        {
            SetHAngle(H_angle,true);
        }
        else
        {
            SetHAngle(H_angle,false);
        }
        
        if (first_H)
        {
            delay(first_H_delay);
            first_H = false;
        }
        else
        {
            delay(H_delay);
        }

        for (int srv = V_start; srv <= V_end; srv += V_division)
        {
            V_angle = 90 - srv;
            if (abs(V_angle - current_v_servo_angle) > 10)
            {
                SetVAngle(srv, true);
            }
            else
            {
                SetVAngle(srv, false);
            }
            if (first_V)
            {
                delay(first_V_delay);
                first_V = false;
            }
            else
            {
                delay(V_delay);
            }
            Tfdistance = 0;
            for (int i = 0; i < avg_reading; i++)
            {
                Tfdistance = Tfdistance + TfReturnDistance();
                delay(2);
            }
            Tfdistance = Tfdistance / avg_reading;

            if (Tfdistance < 30000)
            {
                X_inertia = Tfdistance * cos(V_angle * MY_PI / 180) * cos(stp * MY_PI / 180);
                Y_inertia = Tfdistance * cos(V_angle * MY_PI / 180) * sin(stp * MY_PI / 180);
                Z_inertia = Tfdistance * sin(V_angle * MY_PI / 180);
                obstacle_pos.x = X_inertia;
                obstacle_pos.y = -Y_inertia;
                obstacle_pos.z = Z_inertia;
                if (data_send)
                {
                    byte SerSnd[12];
                    doubleConv cnrtrSnd;
                    cnrtrSnd.asdouble = obstacle_pos.x;
                    SerSnd[0] = cnrtrSnd.asBytes[0];
                    SerSnd[1] = cnrtrSnd.asBytes[1];
                    SerSnd[2] = cnrtrSnd.asBytes[2];
                    SerSnd[3] = cnrtrSnd.asBytes[3];
                    cnrtrSnd.asdouble = obstacle_pos.y;
                    SerSnd[4] = cnrtrSnd.asBytes[0];
                    SerSnd[5] = cnrtrSnd.asBytes[1];
                    SerSnd[6] = cnrtrSnd.asBytes[2];
                    SerSnd[7] = cnrtrSnd.asBytes[3];
                    cnrtrSnd.asdouble = obstacle_pos.z;
                    SerSnd[8] = cnrtrSnd.asBytes[0];
                    SerSnd[9] = cnrtrSnd.asBytes[1];
                    SerSnd[10] = cnrtrSnd.asBytes[2];
                    SerSnd[11] = cnrtrSnd.asBytes[3];
                    Serial.write(SerSnd, sizeof(SerSnd));
                    delay(5);
                }
                else
                {
                    Serial.print("Obstacle point [x,y,z,s,stp]: [");
                    Serial.print(obstacle_pos.x);
                    Serial.print(" , ");
                    Serial.print(obstacle_pos.y);
                    Serial.print(" , ");
                    Serial.print(obstacle_pos.z);
                    Serial.print(" , ");
                    Serial.print(Tfdistance);
                    Serial.print(" , ");
                    Serial.print(stp);
                    Serial.println("] ");
                }
            }
        }
    }
}

#endif