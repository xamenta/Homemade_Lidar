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
#define VServo 9            //Vertical Servo pin
#define Optpin 2            //Opto pin

//:::::::::::::::::::::::::: I2C Addresses :::::::::::::::::::::::::://
#define TFLuna_address 0x10     //I2C address of the TF Luna TOF sensor

//:::::::::::::::::::::::::: DEFINE GLOBAL COMPONENTS :::::::::::::::::::::::::://
AccelStepper SensorStepper(AccelStepper::HALF4WIRE, StepperPin1, StepperPin3, StepperPin2, StepperPin4); // Define some steppers and the pins the will use
TFLI2C lidar;
Servo V_axis;

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
double stepper_angle;
double servo_angle;

//:::::::::::::::::::::::::: DEFINE PARAMETERS :::::::::::::::::::::::::://
const double MY_PI = 3.141592653589793238462643;    //Value of pi
uint16_t tfFrame = TFL_DEF_FPS;                     //Default frame rate for TF Luna
const int FullStep = 4096;                          //Steps per revolution for the stepper motor
const int Gear_Ratio = 4;                           //Gear ratio between the stepper motor and the lidar rotor
const int Lidar_limit = 650;                        //Limit of the range of the Lidar in cm. This is done to fit 1 byte unsigned int
const int avg_reading = 1;                          //Number of readings to average
const bool data_send = false;                       //Boolean to select either to send data or print to serial
const bool both_run = false;                        //Boolean to specify if to do vertical sweep in both directions

const int stepper_division = 1;                     //Degree divisions for the movement of the horizontal sweep (even numbers between 2 and 30 degrees!)
const int stepper_end = 360;                        //Rotation range for the movement of the horizontal sweep (Steps of 30 degrees up to 360 degrees!)
const int stepper_start = 0;                        //Degree offset from 0 for the start point of the movement of the horizontal sweep
const int first_stepper_delay = 50;
const int stepper_delay = 1;

const int servo_division = 1;                       //Degree divisions for the movement of the vertical sweep
const int servo_end = 140;                          //Rotation range for the movement of the vertical sweep
const int servo_start = 0;                          //Degree offset from 0 for the start point of the movement of the vertical sweep
const int first_servo_delay = 250;
const int servo_delay = 5;
const double srv_dir_ofst = 0.0;                    //Mismatch angle between sweeping up and sweeping down


//:::::::::::::::::::::::::: STEPPER MOTOR :::::::::::::::::::::::::://
void StepperInitialize(double mxspd, double spd, double accl)
{
    pinMode(Optpin,INPUT);
    SensorStepper.setMaxSpeed(mxspd);
    SensorStepper.setAcceleration(accl);
    SensorStepper.setSpeed(spd);
    delay(500);
    bool moveStep = true;
    while (moveStep)
    {
        SensorStepper.move(1);
        SensorStepper.runSpeed();
        if (digitalRead(Optpin) == LOW)
        {
            // Serial.println(digitalRead(Optpin));
            moveStep = false;
        }
        else
        {
            // Serial.println(digitalRead(Optpin));
        }
    }
    SensorStepper.setCurrentPosition(0);
    SensorStepper.setMaxSpeed(mxspd);
    SensorStepper.setAcceleration(accl);
    SensorStepper.setSpeed(spd);
}

void MoveToDeg(int deg) 
{
    double Fs = FullStep;
    double Gr = Gear_Ratio;
    double deg_move = ceil((Fs*deg/(360.0*Gr)));
    SensorStepper.moveTo((long)deg_move);
    // SensorStepper.setSpeed(960.0);
    SensorStepper.runToPosition();
    // Serial.print("Step Requested: ");
    // Serial.print(deg_move);
    // Serial.print("Current Step: ");
    // Serial.println(SensorStepper.currentPosition());
    // delay(10);
}

int StepperDegree()
{
    return (int)(360*Gear_Ratio*SensorStepper.currentPosition()/FullStep);
}

//:::::::::::::::::::::::::: VERTICAL SERVO :::::::::::::::::::::::::://
void ServoInitialization()
{
    V_axis.attach(VServo,500,2500);
    V_axis.write(0);
    delay(1000);
    V_axis.write(180);
    delay(1000);
    V_axis.write(90);
}

void SetAngle(int deg)
{
    V_axis.write(deg);
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

void RunLoopBoth()
{
    bool first_servo = true;
    bool first_stepper = true;
    bool move_down = true;
    for (int stp = stepper_start; stp <= stepper_end; stp += stepper_division)
    {
        first_servo = true;
        stepper_angle = stp;
        MoveToDeg(stp);
        if (first_stepper)
        {
            delay(first_stepper_delay);
            first_stepper = false;
        }
        else
        {
            delay(stepper_delay);
        }

        if (move_down)
        {
            move_down = false;
            for (int srv = servo_start; srv <= servo_end; srv += servo_division)
            {
                servo_angle = 90 - srv-srv_dir_ofst;
                V_axis.write(srv);
                if (first_servo)
                {
                    delay(first_servo_delay);
                    first_servo = false;
                }
                else
                {
                    delay(servo_delay);
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
                    X_inertia = Tfdistance * cos(servo_angle * MY_PI / 180) * cos(-stp * MY_PI / 180);
                    Y_inertia = Tfdistance * cos(servo_angle * MY_PI / 180) * sin(-stp * MY_PI / 180);
                    Z_inertia = Tfdistance * sin(servo_angle * MY_PI / 180);
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
            move_down = true;
            for (int srv = servo_end; srv >= servo_start; srv -= servo_division)
            {
                servo_angle = 90 - srv + srv_dir_ofst;
                V_axis.write(srv); // SetAngle(srv);
                if (first_servo)
                {
                    delay(first_servo_delay);
                    first_servo = false;
                }
                else
                {
                    delay(servo_delay);
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
                    X_inertia = Tfdistance * cos(servo_angle * MY_PI / 180) * cos(-stp * MY_PI / 180);
                    Y_inertia = Tfdistance * cos(servo_angle * MY_PI / 180) * sin(-stp * MY_PI / 180);
                    Z_inertia = Tfdistance * sin(servo_angle * MY_PI / 180);
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

void RunLoopOne()
{
    bool first_servo = true;
    bool first_stepper = true;
    for (int stp = stepper_start; stp <= stepper_end; stp += stepper_division)
    {
        first_servo = true;
        stepper_angle = stp;
        MoveToDeg(stp);
        if (first_stepper)
        {
            delay(first_stepper_delay);
            first_stepper = false;
        }
        else
        {
            delay(stepper_delay);
        }

        for (int srv = servo_start; srv <= servo_end; srv += servo_division)
        {
            servo_angle = 90 - srv-srv_dir_ofst;
            V_axis.write(srv);
            if (first_servo)
            {
                delay(first_servo_delay);
                first_servo = false;
            }
            else
            {
                delay(servo_delay);
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
                X_inertia = Tfdistance * cos(servo_angle * MY_PI / 180) * cos(-stp * MY_PI / 180);
                Y_inertia = Tfdistance * cos(servo_angle * MY_PI / 180) * sin(-stp * MY_PI / 180);
                Z_inertia = Tfdistance * sin(servo_angle * MY_PI / 180);
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