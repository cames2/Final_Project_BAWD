/*
  MSE 2202 Final Project - TEAM B.A.W.D.
  Language: Arduino
  Authors: Curtis Ames & Nick Baxter & Spencer Dale & Jenna Wemple
  Date: Mar 2018
  Rev 1 - Initial version
  Rev 2 - Update for MSEduino v. 2
*/

#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_ArmMotor;
Servo servo_GripMotor;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

// Uncomment keywords to enable debugging output

//#define DEBUG_MODE_DISPLAY
//#define DEBUG_MOTORS
//#define DEBUG_LINE_TRACKERS
//#define DEBUG_ENCODERS
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER_CALIBRATION
//#define DEBUG_MOTOR_CALIBRATION

boolean bt_Motors_Enabled = true;

//port pin constants
const int ci_frontUltrasonic_Ping = 11;   //input plug
const int ci_frontUltrasonic_Data = 12;   //output plug
const int ci_rearUltrasonic_Ping = 2;   //input plug
const int ci_rearUltrasonic_Data = 3;   //output plug
const int ci_Charlieplex_LED1 = 4;
const int ci_Charlieplex_LED2 = 5;
const int ci_Charlieplex_LED3 = 6;
const int ci_Charlieplex_LED4 = 7;
const int ci_Mode_Button = 7;
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Arm_Motor = 10;
const int ci_Grip_Motor = 13;//switched from11 testing Ultras
const int ci_Motor_Enable_Switch; //= 12; SWITCHED SO WE CAN USE ULTRASONIC//wire removed
const int ci_Right_Line_Tracker = A0;
const int ci_Middle_Line_Tracker = A1;
const int ci_Left_Line_Tracker = A2;
const int ci_Light_Sensor = A3;
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow

// Charlieplexing LED assignments
const int ci_Heartbeat_LED = 1;
const int ci_Indicator_LED = 4;
const int ci_Right_Line_Tracker_LED = 6;
const int ci_Middle_Line_Tracker_LED = 9;
const int ci_Left_Line_Tracker_LED = 12;

//constants

// EEPROM addresses
const int ci_Left_Line_Tracker_Dark_Address_L = 0;
const int ci_Left_Line_Tracker_Dark_Address_H = 1;
const int ci_Left_Line_Tracker_Light_Address_L = 2;
const int ci_Left_Line_Tracker_Light_Address_H = 3;
const int ci_Middle_Line_Tracker_Dark_Address_L = 4;
const int ci_Middle_Line_Tracker_Dark_Address_H = 5;
const int ci_Middle_Line_Tracker_Light_Address_L = 6;
const int ci_Middle_Line_Tracker_Light_Address_H = 7;
const int ci_Right_Line_Tracker_Dark_Address_L = 8;
const int ci_Right_Line_Tracker_Dark_Address_H = 9;
const int ci_Right_Line_Tracker_Light_Address_L = 10;
const int ci_Right_Line_Tracker_Light_Address_H = 11;
const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;

//speeds
const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Grip_Motor_Open = 140;         // Experiment to determine appropriate value
const int ci_Grip_Motor_Closed = 0;        //  "
const int ci_Arm_Servo_Retracted = 0;      //  "
const int ci_Arm_Servo_Extended = 40;      //  "
const int ci_Display_Time = 500;
const int ci_Line_Tracker_Calibration_Interval = 100;
const int ci_Line_Tracker_Cal_Measures = 20;
const int ci_Line_Tracker_Tolerance = 169;   // May need to adjust this
const int ci_Motor_Calibration_Cycles = 3;
const int ci_Motor_Calibration_Time = 5000;
const int minWall_distance = 4;//find intended distance - think about turn
const int maxWall_distance = 7;//"
const int wallTolerance = 1;//"

//variables
byte b_LowByte;
byte b_HighByte;
double ul_Echo_Time;
unsigned int ui_Left_Line_Tracker_Data;
unsigned int ui_Middle_Line_Tracker_Data;
unsigned int ui_Right_Line_Tracker_Data;
unsigned int ui_Motors_Speed = 1669;        // Default run speed
unsigned int ui_Motors_Reverse = 1400;
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
unsigned int ui_Left_Motor_Reverse;
unsigned int ui_Right_Motor_Reverse;
long l_Left_Motor_Position;
long l_Right_Motor_Position;
double rearWall_Distance;
double frontWall_Distance;

unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;

unsigned int ui_Cal_Count;
unsigned int ui_Cal_Cycle;
unsigned int ui_Left_Line_Tracker_Dark;
unsigned int ui_Left_Line_Tracker_Light;
unsigned int ui_Middle_Line_Tracker_Dark;
unsigned int ui_Middle_Line_Tracker_Light;
unsigned int ui_Right_Line_Tracker_Dark;
unsigned int ui_Right_Line_Tracker_Light;
unsigned int ui_Line_Tracker_Tolerance;

unsigned int  ui_Robot_State_Index = 0;
//0123456789ABCDEF
unsigned int  ui_Mode_Indicator[6] = {
  0x00,    //B0000000000000000,  //Stop
  0x00FF,  //B0000000011111111,  //Run
  0x0F0F,  //B0000111100001111,  //Calibrate line tracker light level
  0x3333,  //B0011001100110011,  //Calibrate line tracker dark level
  0xAAAA,  //B1010101010101010,  //Calibrate motors
  0xFFFF   //B1111111111111111   //Unused
};

unsigned int  ui_Mode_Indicator_Index = 0;

//display Bits 0,1,2,3, 4, 5, 6,  7,  8,  9,  10,  11,  12,  13,   14,   15
int  iArray[16] = {
  1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 65536
};
int  iArrayIndex = 0;

boolean bt_Heartbeat = true;
boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;

int stop_Counter = 0;
int run_State = 1;
int yes = 0;

void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);

  CharliePlexM::setBtn(ci_Charlieplex_LED1, ci_Charlieplex_LED2,
                       ci_Charlieplex_LED3, ci_Charlieplex_LED4, ci_Mode_Button);

  // set up ultrasonic
  pinMode(ci_frontUltrasonic_Ping, OUTPUT);
  pinMode(ci_frontUltrasonic_Data, INPUT);
  pinMode(ci_rearUltrasonic_Ping, OUTPUT);
  pinMode(ci_rearUltrasonic_Data, INPUT);

  // set up drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);

  // set up arm motors
  pinMode(ci_Arm_Motor, OUTPUT);
  servo_ArmMotor.attach(ci_Arm_Motor);
  pinMode(ci_Grip_Motor, OUTPUT);
  servo_GripMotor.attach(ci_Grip_Motor);

  // set up motor enable switch
  pinMode(ci_Motor_Enable_Switch, INPUT);

  // set up encoders. Must be initialized in order that they are chained together,
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward

  // set up line tracking sensors
  pinMode(ci_Right_Line_Tracker, INPUT);
  pinMode(ci_Middle_Line_Tracker, INPUT);
  pinMode(ci_Left_Line_Tracker, INPUT);
  ui_Line_Tracker_Tolerance = ci_Line_Tracker_Tolerance;

  // read saved values from EEPROM
  b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Left_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Left_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Middle_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Middle_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Right_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Right_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
  ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
  ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);
}

void loop()
{
  if ((millis() - ul_3_Second_timer) > 3000)
  {
    bt_3_S_Time_Up = true;
  }

  // button-based mode selection
  if (CharliePlexM::ui_Btn)
  {
    if (bt_Do_Once == false)
    {
      bt_Do_Once = true;
      ui_Robot_State_Index++;
      ui_Robot_State_Index = ui_Robot_State_Index & 7;
      ul_3_Second_timer = millis();
      bt_3_S_Time_Up = false;
      bt_Cal_Initialized = false;
      ui_Cal_Cycle = 0;
    }
  }
  else
  {
    bt_Do_Once = LOW;
  }

  // check if drive motors should be powered
  bt_Motors_Enabled = digitalRead(ci_Motor_Enable_Switch);

  // modes
  // 0 = default after power up/reset
  // 1 = Press mode button once to enter. Run robot.
  // 2 = Press mode button twice to enter. Calibrate line tracker light level.
  // 3 = Press mode button three times to enter. Calibrate line tracker dark level.
  // 4 = Press mode button four times to enter. Calibrate motor speeds to drive straight.
  switch (ui_Robot_State_Index)
  {
    case 0:    //Robot stopped
      {
        readLineTrackers();
        Ping(ci_frontUltrasonic_Ping, ci_frontUltrasonic_Data);
        servo_ArmMotor.write(ci_Arm_Servo_Retracted);
        servo_GripMotor.write(ci_Grip_Motor_Closed);
        encoder_LeftMotor.zero();
        encoder_RightMotor.zero();
        ui_Mode_Indicator_Index = 0;
        servo_GripMotor.write(ci_Grip_Motor_Open);
        break;
      }

    case 1:    //Robot Run after 3 seconds
      {
        if (bt_3_S_Time_Up)
        {
          // rearWall_Distance = (Ping(ci_rearUltrasonic_Ping, ci_rearUltrasonic_Data));
          frontWall_Distance = (Ping(ci_frontUltrasonic_Ping, ci_frontUltrasonic_Data));

          Serial.print("  front Distance = ");
          Serial.print(frontWall_Distance);
          Serial.print("     ");

        }
        /* readLineTrackers();

          #ifdef DEBUG_ENCODERS
          l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
          l_Right_Motor_Position = encoder_RightMotor.getRawPosition();

          Serial.print("Encoders L: ");
          Serial.print(l_Left_Motor_Position);
          Serial.print(", R: ");
          Serial.println(l_Right_Motor_Position);
          #endif

          // set motor speeds
          ui_Left_Motor_Speed = constrain(ui_Motors_Speed + ui_Left_Motor_Offset, 1600, 2100);
          ui_Right_Motor_Speed = constrain(ui_Motors_Speed + ui_Right_Motor_Offset, 1600, 2100);
          ui_Left_Motor_Reverse = constrain(ui_Motors_Reverse - ui_Left_Motor_Offset, 900, 1500);
          ui_Right_Motor_Reverse = constrain(ui_Motors_Reverse - ui_Right_Motor_Offset, 900, 1500);
          /***************************************************************************************
           Add line tracking code here.
           Adjust motor speed according to information from line tracking sensors and
           possibly encoder counts.
                            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
                            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
                            servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
                            servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
                            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Reverse);
                            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Reverse);
                            ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ci_Line_Tracker_Tolerance)
                            ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ci_Line_Tracker_Tolerance)
                            ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ci_Line_Tracker_Tolerance)
                            ul_Echo_Time
           /*************************************************************************************

          if (bt_Motors_Enabled)
          {
           if (run_State == 1)//turns until only middle one is on
           {
             // turn towards led line
             if ((!(ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ci_Line_Tracker_Tolerance))) &&
                 (ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ci_Line_Tracker_Tolerance)) &&
                 (!(ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ci_Line_Tracker_Tolerance))))
             {
               servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
               servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
               run_State += 1;
               //ui_Robot_State_Index = 0;
             }
             else
             {
               servo_RightMotor.writeMicroseconds(1420);
               servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
             }
             // open claw
             servo_GripMotor.write(ci_Grip_Motor_Open);
           }
           else if (run_State == 2)//if it hits the box line it stops and goes to next, otherwise it follows line
           {
             // approach platform
             if (ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ci_Line_Tracker_Tolerance) &&
                 ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ci_Line_Tracker_Tolerance))
             {
               servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
               servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
               run_State += 1;
               //ui_Robot_State_Index = 0;
             }
             else if (ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ci_Line_Tracker_Tolerance))
             {
               servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Reverse);
               servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
             }
             else if (ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ci_Line_Tracker_Tolerance))
             {
               servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
               servo_RightMotor.writeMicroseconds(ui_Right_Motor_Reverse);
             }
             else
             {
               servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
               servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
             }
           }
           else if (run_State == 3)//look for light by spinning on the spot
           {
             if (!(digitalRead(ci_Light_Sensor)))
             {
               yes = 1;
               servo_RightMotor.writeMicroseconds(1590);
               servo_LeftMotor.writeMicroseconds(1410);
             }
             if (yes == 1)
             {
               servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
               servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
               servo_ArmMotor.write(ci_Arm_Servo_Extended);
               stop_Counter++;
               if (stop_Counter == 500) {
                 servo_GripMotor.write(ci_Grip_Motor_Closed);
               }

               if (stop_Counter == 2000)
               {
                 servo_ArmMotor.write(ci_Arm_Servo_Retracted);
                 run_State++;
                 stop_Counter = 0;
               }
             }
             else
             {
               servo_LeftMotor.writeMicroseconds(1590);
               servo_RightMotor.writeMicroseconds(1410);
               stop_Counter = 0;
               servo_ArmMotor.write(ci_Arm_Servo_Retracted);
               servo_GripMotor.write(ci_Grip_Motor_Open);
             }
           }
           else if (run_State == 4) {//spin away from block
             stop_Counter++;
             if (stop_Counter < 2200) {
               servo_RightMotor.writeMicroseconds(1360);
               servo_LeftMotor.writeMicroseconds(1360);
             }
             if (stop_Counter > 2200) {
               run_State++;
             }
           }
           else if (run_State == 5)//look for line
           {
             if ((!(ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ci_Line_Tracker_Tolerance))) &&
                 (ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ci_Line_Tracker_Tolerance)) &&
                 (!(ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ci_Line_Tracker_Tolerance))))
             {
               servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
               servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
               run_State++;
               //ui_Robot_State_Index = 0; // testing
             }
             else
             {
               servo_LeftMotor.writeMicroseconds(1400);
               servo_RightMotor.writeMicroseconds(1600);
             }
           }
           else if (run_State == 6)//standard line following code
           {
             if (ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ci_Line_Tracker_Tolerance) &&
                 ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ci_Line_Tracker_Tolerance))
             {
               servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
               servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
               stop_Counter++;
               if (stop_Counter == 500)
               {
                 run_State++;
                 stop_Counter == 0;
                 //ui_Robot_State_Index = 0;
               }
             }

             else if (ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ci_Line_Tracker_Tolerance))
             {
               servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Reverse);
               servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
               stop_Counter = 0;
             }

             else if (ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ci_Line_Tracker_Tolerance))
             {
               servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
               servo_RightMotor.writeMicroseconds(ui_Right_Motor_Reverse);
               stop_Counter = 0;
             }

             else if (ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ci_Line_Tracker_Tolerance))
             {
               servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
               servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
               stop_Counter = 0;
             }
           }
           else if (run_State == 7)//look for line
           {
             if ((!(ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ci_Line_Tracker_Tolerance))) &&
                 (ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ci_Line_Tracker_Tolerance)) &&
                 (!(ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ci_Line_Tracker_Tolerance))))
             {
               servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
               servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
               run_State++;
               //ui_Robot_State_Index = 0; // testing
             }
             else
             {
               servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
               servo_RightMotor.writeMicroseconds(1415);
             }
           }
           else if (run_State == 8)//if it hits the box line it stops and goes to next, otherwise it follows line
           {
             // approach platform
             if (ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ci_Line_Tracker_Tolerance) &&
                 ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ci_Line_Tracker_Tolerance))
             {
               servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
               servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
               run_State += 1;
               servo_ArmMotor.write(ci_Arm_Servo_Extended);
               //ui_Robot_State_Index = 0;
             }
             else if (ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ci_Line_Tracker_Tolerance))
             {
               servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Reverse);
               servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
             }
             else if (ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ci_Line_Tracker_Tolerance))
             {
               servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
               servo_RightMotor.writeMicroseconds(ui_Right_Motor_Reverse);
             }
             else
             {
               servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
               servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
             }
           }
           else if (run_State == 9) {
             servo_GripMotor.write(ci_Grip_Motor_Open);
             stop_Counter++;
             if (stop_Counter == 1000)
             {
               run_State++;
               stop_Counter == 0;
               //ui_Robot_State_Index = 0;
             }
           }
           else if (run_State == 10) {
             stop_Counter++;
             servo_RightMotor.writeMicroseconds(1400);
             servo_LeftMotor.writeMicroseconds(1400);
             if (stop_Counter > 2600) {
               ui_Robot_State_Index = 0;
             }
           }

          }
          else
          {
           servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
           servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
          }
          #ifdef DEBUG_MOTORS
          Serial.print("Motors enabled: ");
          Serial.print(bt_Motors_Enabled);
          Serial.print(", Default: ");
          Serial.print(ui_Motors_Speed);
          Serial.print(", Left = ");
          Serial.print(ui_Left_Motor_Speed);
          Serial.print(", Right = ");
          Serial.println(ui_Right_Motor_Speed);
          #endif
          ui_Mode_Indicator_Index = 1;
          }*/
        break;
      }


    case 2:    //Calibrate line tracker light levels after 3 seconds
      {
        if (bt_3_S_Time_Up)
        {
          if (!bt_Cal_Initialized)
          {
            bt_Cal_Initialized = true;
            ui_Left_Line_Tracker_Light = 0;
            ui_Middle_Line_Tracker_Light = 0;
            ui_Right_Line_Tracker_Light = 0;
            ul_Calibration_Time = millis();
            ui_Cal_Count = 0;
          }
          else if ((millis() - ul_Calibration_Time) > ci_Line_Tracker_Calibration_Interval)
          {
            ul_Calibration_Time = millis();
            readLineTrackers();
            ui_Left_Line_Tracker_Light += ui_Left_Line_Tracker_Data;
            ui_Middle_Line_Tracker_Light += ui_Middle_Line_Tracker_Data;
            ui_Right_Line_Tracker_Light += ui_Right_Line_Tracker_Data;
            ui_Cal_Count++;
          }
          if (ui_Cal_Count == ci_Line_Tracker_Cal_Measures)
          {
            ui_Left_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
            ui_Middle_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
            ui_Right_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
#ifdef DEBUG_LINE_TRACKER_CALIBRATION
            Serial.print("Light Levels: Left = ");
            Serial.print(ui_Left_Line_Tracker_Light, DEC);
            Serial.print(", Middle = ");
            Serial.print(ui_Middle_Line_Tracker_Light, DEC);
            Serial.print(", Right = ");
            Serial.println(ui_Right_Line_Tracker_Light, DEC);
#endif
            EEPROM.write(ci_Left_Line_Tracker_Light_Address_L, lowByte(ui_Left_Line_Tracker_Light));
            EEPROM.write(ci_Left_Line_Tracker_Light_Address_H, highByte(ui_Left_Line_Tracker_Light));
            EEPROM.write(ci_Middle_Line_Tracker_Light_Address_L, lowByte(ui_Middle_Line_Tracker_Light));
            EEPROM.write(ci_Middle_Line_Tracker_Light_Address_H, highByte(ui_Middle_Line_Tracker_Light));
            EEPROM.write(ci_Right_Line_Tracker_Light_Address_L, lowByte(ui_Right_Line_Tracker_Light));
            EEPROM.write(ci_Right_Line_Tracker_Light_Address_H, highByte(ui_Right_Line_Tracker_Light));
            ui_Robot_State_Index = 0;    // go back to Mode 0
          }
          ui_Mode_Indicator_Index = 2;
        }
        break;
      }

    case 3:    // Calibrate line tracker dark levels after 3 seconds
      {
        if (bt_3_S_Time_Up)
        {
          if (!bt_Cal_Initialized)
          {
            bt_Cal_Initialized = true;
            ui_Left_Line_Tracker_Dark = 0;
            ui_Middle_Line_Tracker_Dark = 0;
            ui_Right_Line_Tracker_Dark = 0;
            ul_Calibration_Time = millis();
            ui_Cal_Count = 0;
          }
          else if ((millis() - ul_Calibration_Time) > ci_Line_Tracker_Calibration_Interval)
          {
            ul_Calibration_Time = millis();
            readLineTrackers();
            ui_Left_Line_Tracker_Dark += ui_Left_Line_Tracker_Data;
            ui_Middle_Line_Tracker_Dark += ui_Middle_Line_Tracker_Data;
            ui_Right_Line_Tracker_Dark += ui_Right_Line_Tracker_Data;
            ui_Cal_Count++;
          }
          if (ui_Cal_Count == ci_Line_Tracker_Cal_Measures)
          {
            ui_Left_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
            ui_Middle_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
            ui_Right_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
#ifdef DEBUG_LINE_TRACKER_CALIBRATION
            Serial.print("Dark Levels: Left = ");
            Serial.print(ui_Left_Line_Tracker_Dark, DEC);
            Serial.print(", Middle = ");
            Serial.print(ui_Middle_Line_Tracker_Dark, DEC);
            Serial.print(", Right = ");
            Serial.println(ui_Right_Line_Tracker_Dark, DEC);
#endif
            EEPROM.write(ci_Left_Line_Tracker_Dark_Address_L, lowByte(ui_Left_Line_Tracker_Dark));
            EEPROM.write(ci_Left_Line_Tracker_Dark_Address_H, highByte(ui_Left_Line_Tracker_Dark));
            EEPROM.write(ci_Middle_Line_Tracker_Dark_Address_L, lowByte(ui_Middle_Line_Tracker_Dark));
            EEPROM.write(ci_Middle_Line_Tracker_Dark_Address_H, highByte(ui_Middle_Line_Tracker_Dark));
            EEPROM.write(ci_Right_Line_Tracker_Dark_Address_L, lowByte(ui_Right_Line_Tracker_Dark));
            EEPROM.write(ci_Right_Line_Tracker_Dark_Address_H, highByte(ui_Right_Line_Tracker_Dark));
            ui_Robot_State_Index = 0;    // go back to Mode 0
          }
          ui_Mode_Indicator_Index = 3;
        }
        break;
      }

    case 4:    //Calibrate motor straightness after 3 seconds.
      {

        if (bt_3_S_Time_Up)
        {
          // servo_LeftMotor.writeMicroseconds(1800);
          //servo_RightMotor.writeMicroseconds(1800);
          /*if (!bt_Cal_Initialized)
            {
            bt_Cal_Initialized = true;
            encoder_LeftMotor.zero();
            encoder_RightMotor.zero();
            ul_Calibration_Time = millis();
            servo_LeftMotor.writeMicroseconds(ui_Motors_Speed);
            servo_RightMotor.writeMicroseconds(ui_Motors_Speed);
            }
            else if ((millis() - ul_Calibration_Time) > ci_Motor_Calibration_Time)
            {
            servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
            servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
            l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
            l_Right_Motor_Position = encoder_RightMotor.getRawPosition();
            if (l_Left_Motor_Position > l_Right_Motor_Position)
            {
              // May have to update this if different calibration time is used
              ui_Right_Motor_Offset = 0;
              ui_Left_Motor_Offset = (l_Left_Motor_Position - l_Right_Motor_Position) / 4;
            }
            else
            {
              // May have to update this if different calibration time is used
              ui_Right_Motor_Offset = (l_Right_Motor_Position - l_Left_Motor_Position) / 4;
              ui_Left_Motor_Offset = 0;
            }

            #ifdef DEBUG_MOTOR_CALIBRATION
            Serial.print("Motor Offsets: Left = ");
            Serial.print(ui_Left_Motor_Offset);
            Serial.print(", Right = ");
            Serial.println(ui_Right_Motor_Offset);
            #endif
            EEPROM.write(ci_Right_Motor_Offset_Address_L, lowByte(ui_Right_Motor_Offset));
            EEPROM.write(ci_Right_Motor_Offset_Address_H, highByte(ui_Right_Motor_Offset));
            EEPROM.write(ci_Left_Motor_Offset_Address_L, lowByte(ui_Left_Motor_Offset));
            EEPROM.write(ci_Left_Motor_Offset_Address_H, highByte(ui_Left_Motor_Offset));

            ui_Robot_State_Index = 0;    // go back to Mode 0
            }
            #ifdef DEBUG_MOTOR_CALIBRATION
            Serial.print("Encoders L: ");
            Serial.print(encoder_LeftMotor.getRawPosition());
            Serial.print(", R: ");
            Serial.println(encoder_RightMotor.getRawPosition());
            #endif
            ui_Mode_Indicator_Index = 4;
            }
            break;*/
        }
      }
    case 5:    //wall tracking testing //driving clockwise
      {
        /*  const int ci_frontUltrasonic_Ping = 2;   //input plug
          const int ci_frontUltrasonic_Data = 3;   //output plug
          const int ci_rearUltrasonic_Ping = 13;   //input plug
          const int ci_frontUltrasonic_Data*/
        if (bt_3_S_Time_Up)
        {

          rearWall_Distance = (Ping(ci_rearUltrasonic_Ping, ci_rearUltrasonic_Data));
          frontWall_Distance = (Ping(ci_frontUltrasonic_Ping, ci_frontUltrasonic_Data));

          Serial.print("  REAR Distance = ");
          Serial.print(rearWall_Distance);
          Serial.print("     ");

          Serial.print("  FRONT Distance = ");
          Serial.print(frontWall_Distance);
          Serial.print("     ");

          //these three should be able to hold it to the wall with correct tolerances... may need more explicit code
          /*if (!((rearWall_Distance <= maxWall_distance) && (rearWall_Distance >= minWall_distance) && (frontWall_Distance <= maxWall_distance) && (frontWall_Distance >= minWall_distance))) {
            servo_LeftMotor.writeMicroseconds(1400);
            servo_RightMotor.writeMicroseconds(1400);
            }*/
          if ((rearWall_Distance <= maxWall_distance) && (rearWall_Distance >= minWall_distance) && (frontWall_Distance <= maxWall_distance) && (frontWall_Distance >= minWall_distance)) {
            servo_LeftMotor.writeMicroseconds(1800);
            servo_RightMotor.writeMicroseconds(1800);//if perfect drive straight
          }
          if ((rearWall_Distance <= minWall_distance) && (frontWall_Distance <= minWall_distance)) {
            servo_LeftMotor.writeMicroseconds(1750);
            servo_RightMotor.writeMicroseconds(1600);// if too close, turn away
          }else if (rearWall_Distance <= (frontWall_Distance - wallTolerance)) {
            servo_LeftMotor.writeMicroseconds(1500);
            servo_RightMotor.writeMicroseconds(1750);//if turning towards turn away
          }
          if ((rearWall_Distance >= maxWall_distance) && (frontWall_Distance >= maxWall_distance)) {
            servo_LeftMotor.writeMicroseconds(1600);
            servo_RightMotor.writeMicroseconds(1750);// if too far, turn closer
          } else if (rearWall_Distance >= (frontWall_Distance + wallTolerance)) {
            servo_LeftMotor.writeMicroseconds(1750);
            servo_RightMotor.writeMicroseconds(1500);//if turning away turn towards.
          }





          /*stop_Counter++;
            if (stop_Counter < 10000) {
            servo_ArmMotor.write(ci_Arm_Servo_Retracted);
            }
            else if (stop_Counter < 20000) {
            servo_ArmMotor.write(ci_Arm_Servo_Extended);
            }
            else if (stop_Counter < 30000) {
            servo_GripMotor.write(ci_Grip_Motor_Closed);
            }
            else if (stop_Counter < 40000) {
            servo_ArmMotor.write(ci_Arm_Servo_Retracted);
            }
            else if (stop_Counter < 50000) {
            servo_GripMotor.write(ci_Grip_Motor_Open);
            if (stop_Counter == 60000) {
              stop_Counter = 0;
            }*/

        }
        break;
      }
  }

  if ((millis() - ul_Display_Time) > ci_Display_Time)
  {
    ul_Display_Time = millis();

#ifdef DEBUG_MODE_DISPLAY
    Serial.print("Mode: ");
    Serial.println(ui_Mode_Indicator[ui_Mode_Indicator_Index], DEC);
#endif
    bt_Heartbeat = !bt_Heartbeat;
    CharliePlexM::Write(ci_Heartbeat_LED, bt_Heartbeat);
    digitalWrite(13, bt_Heartbeat);
    Indicator();
  }
}

// set mode indicator LED state
void Indicator()
{
  //display routine, if true turn on led
  CharliePlexM::Write(ci_Indicator_LED, !(ui_Mode_Indicator[ui_Mode_Indicator_Index] &
                                          (iArray[iArrayIndex])));
  iArrayIndex++;
  iArrayIndex = iArrayIndex & 15;
}

// read values from line trackers and update status of line tracker LEDs
void readLineTrackers()
{
  ui_Left_Line_Tracker_Data = analogRead(ci_Left_Line_Tracker);
  ui_Middle_Line_Tracker_Data = analogRead(ci_Middle_Line_Tracker);
  ui_Right_Line_Tracker_Data = analogRead(ci_Right_Line_Tracker);

  if (ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Left_Line_Tracker_LED, HIGH);
  }
  else
  {
    CharliePlexM::Write(ci_Left_Line_Tracker_LED, LOW);
  }
  if (ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Middle_Line_Tracker_LED, HIGH);
  }
  else
  {
    CharliePlexM::Write(ci_Middle_Line_Tracker_LED, LOW);
  }
  if (ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Right_Line_Tracker_LED, HIGH);
  }
  else
  {
    CharliePlexM::Write(ci_Right_Line_Tracker_LED, LOW);
  }

#ifdef DEBUG_LINE_TRACKERS
  Serial.print("Trackers: Left = ");
  Serial.print(ui_Left_Line_Tracker_Data, DEC);
  Serial.print(", Middle = ");
  Serial.print(ui_Middle_Line_Tracker_Data, DEC);
  Serial.print(", Right = ");
  Serial.println(ui_Right_Line_Tracker_Data, DEC);
#endif

}

// measure distance to target using ultrasonic sensor
double Ping(unsigned int Input, unsigned int Output)
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(Input, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(Input, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW
  ul_Echo_Time = pulseIn(Output, HIGH, 10000);


  // Print Sensor Readings
#ifdef DEBUG_ULTRASONIC
  Serial.print("Time (microseconds): ");
  Serial.print(ul_Echo_Time, DEC);
  Serial.print(", Inches: ");
  Serial.print(ul_Echo_Time / 148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(ul_Echo_Time / 58); //divide time by 58 to get distance in cm
#endif
  return ((ul_Echo_Time / 30.0));
}

