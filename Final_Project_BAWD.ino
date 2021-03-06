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
//#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_ArmMotor;
Servo servo_GripMotor;
Servo servo_StringMotor;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;
I2CEncoder encoder_StringMotor;

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
const int ci_frontUltrasonic_Ping = A0;   //input plug
const int ci_frontUltrasonic_Data = A1;   //output plug
const int ci_rearUltrasonic_Ping = A2;    //input plug
const int ci_rearUltrasonic_Data = A3;    //output plug
const int ci_faceUltrasonic_Ping = 2;     //input plug
const int ci_faceUltrasonic_Data = 3;     //output plug
const int ci_lside_IR = 4;    // Was: ci_Charlieplex_LED1
const int ci_middle_IR = 5;   // Was: ci_Charlieplex_LED2
const int ci_rside_IR = 6;    // Was: ci_Charlieplex_LED3
const int ci_Mode_Button = 7; // Was ci_Charlieplex_LED4
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Arm_Motor = 10;
const int ci_Grip_Motor = 11;
const int ci_Cube_Microswitch = 12;
const int ci_Pyramid_Microswitch = 13;
const int ci_Motor_Enable_Switch; //= 12; SWITCHED SO WE CAN USE ULTRASONIC//wire removed
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow
const int ci_String_Motor = 5;

// Charlieplexing LED assignments
const int ci_Heartbeat_LED = 1;
const int ci_Indicator_LED = 4;
const int ci_Right_Line_Tracker_LED = 6;
const int ci_Middle_Line_Tracker_LED = 9;
const int ci_Left_Line_Tracker_LED = 12;

//constants

// EEPROM addresses
const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;

// Motor constants
const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Grip_Motor_Open = 100;         // Experiment to determine appropriate value
const int ci_Grip_Motor_Closed = 10;        //  "
const int ci_Arm_Servo_Up = 51;      //  "
const int ci_Arm_Servo_Down = 0;      //  "
const int ci_Display_Time = 500;
const int ci_Motor_Calibration_Cycles = 3;
const int ci_Motor_Calibration_Time = 5000;

// Motor variables
byte b_LowByte;
byte b_HighByte;
double ul_Echo_Time;
unsigned int ui_Motors_Speed = 1700;        // Default run speed
unsigned int ui_Motors_Reverse = 1400;
unsigned int ui_Left_Motor_Speed;
unsigned int ui_SAVED_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
unsigned int ui_Left_Motor_Reverse;
unsigned int ui_Right_Motor_Reverse;
unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Smoothing_Counter;
unsigned long ul_Calibration_Time;
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;

// Wall-following constants
const double wall_distance = 6;     // Centimeters

// Wall-following variables
long l_Left_Motor_Position;
long l_Right_Motor_Position;
double rearWall_distance;
double frontWall_distance;
double faceWall_distance;
double angle_error;
double front_error;
double rightP_factor = 0;
double leftP_factor = 0;
double angles[2] = {0};
double angle_diff[3] = {10, 10, 10};
double total_diff = 0;
double prev_total_diff = 0;
double begin_turn_distance = 18;    // Centimeters
double begin_pyramid_turn_distance = 19

                                     ;    // Centimeters
boolean continue_wall_following = 0;    // 0 = continue, 1 = begin pyramid search

// Pyramid-finding constants
unsigned int time_180deg_turn = 2900;    // Milliseconds to complete a 90-degree turn

// Pyramid location variables
boolean correct_pyramid = 1;    // 0 = AE, 1 = IO
int turn_counter = 0;
unsigned long current_time = 0;

// No clue what these do
unsigned int ui_Cal_Count;
unsigned int ui_Cal_Cycle;

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

boolean bt_Heartbeat = true;
boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;

int counter = 0;

void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);

  // Set up ultrasonic sensor pins
  pinMode(ci_frontUltrasonic_Ping, OUTPUT);
  pinMode(ci_frontUltrasonic_Data, INPUT);
  pinMode(ci_rearUltrasonic_Ping, OUTPUT);
  pinMode(ci_rearUltrasonic_Data, INPUT);
  pinMode(ci_faceUltrasonic_Ping, OUTPUT);
  pinMode(ci_faceUltrasonic_Data, INPUT);

  // Set up IR sensor pins
  pinMode(ci_lside_IR, INPUT);
  //pinMode(ci_middle_IR, INPUT);
  pinMode(ci_rside_IR, INPUT);

  // Set up Microswitch pin
  pinMode(ci_Cube_Microswitch, INPUT);
  pinMode(ci_Pyramid_Microswitch, INPUT);

  //set up button
  pinMode(ci_Mode_Button, INPUT);

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

  // Set up string motor
  pinMode(ci_String_Motor, OUTPUT);
  servo_StringMotor.attach(ci_String_Motor);

  // set up motor enable switch
  pinMode(ci_Motor_Enable_Switch, INPUT);

  // set up encoders. Must be initialized in order that they are chained together,
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward
  encoder_StringMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_StringMotor.setReversed(true);  // adjust for positive count when moving forward

  // read saved values from EEPROM
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
  if (!digitalRead(ci_Mode_Button)) { //CharliePlexM::ui_Btn
    if (bt_Do_Once == false)
    {
      bt_Do_Once = true;
      ui_Robot_State_Index++;
      ui_Robot_State_Index = ui_Robot_State_Index % 9;
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

  // Set motor speeds
  ui_Left_Motor_Speed = constrain(ui_Motors_Speed + ui_Left_Motor_Offset, 1600, 2100);
  ui_Right_Motor_Speed = constrain(ui_Motors_Speed + ui_Right_Motor_Offset, 1600, 2100);
  ui_Left_Motor_Reverse = constrain(ui_Motors_Reverse - ui_Left_Motor_Offset, 900, 1500);
  ui_Right_Motor_Reverse = constrain(ui_Motors_Reverse - ui_Right_Motor_Offset, 900, 1500);

  // Ping values every loop through
  rearWall_distance = Ping(ci_rearUltrasonic_Ping, ci_rearUltrasonic_Data);
  frontWall_distance = Ping(ci_frontUltrasonic_Ping, ci_frontUltrasonic_Data);
  faceWall_distance = Ping(ci_faceUltrasonic_Ping, ci_faceUltrasonic_Data);
  if (faceWall_distance == 0) {
    faceWall_distance = 30;
  }
  angle_error = frontWall_distance - rearWall_distance;
  front_error = frontWall_distance - wall_distance;

  // Check if drive motors should be powered
  bt_Motors_Enabled = digitalRead(ci_Motor_Enable_Switch);

  //Serial.println("Looping");
  //Serial.print("ui_Robot_State_Index");
  //Serial.println(ui_Robot_State_Index);

  // ======================================================= TABLE OF CONTENTS =======================================================

  //                                               0 = Default after power-up/reset
  //                                               1 = Calibrate motor straightness\
  //                                               2 = Drive towards a wall
  //                                               3 = Look for AE pyramid (default is IO pyramid)
  //                                               4 = Find tesseract
  //                                               5 = Turn 90-degrees at wall corners
  //                                               6 = Find pyramid
  //                                               7 = Grab & lift pyramid
  //                                               8 = Place pyramid over tesseract
  //                                               9 = Safety case in case magnet does not pull tesseract in far enough

  // =================================================== START OF SWITCH STATEMENT ===================================================

  switch (ui_Robot_State_Index)
  {
    // ---------------------------------------- 0 ------------------------------------------

    case 0:   // Default after power-up/reset
      {
        //Serial.println("Case 0");
        //ui_Robot_State_Index = 7;

        Ping(ci_frontUltrasonic_Ping, ci_frontUltrasonic_Data);
        servo_ArmMotor.write(ci_Arm_Servo_Up);
        encoder_LeftMotor.zero();
        encoder_RightMotor.zero();
        ui_Mode_Indicator_Index = 0;
        servo_GripMotor.write(ci_Grip_Motor_Open);
        ul_Smoothing_Counter = millis();

        if (digitalRead(ci_Cube_Microswitch)) {
          Serial.println("Cube Switch Engaged");
        }

        if (digitalRead(ci_Pyramid_Microswitch)) {
          Serial.println("Pyramid Switch Engaged");
        }


        Serial.print("Left_IR = ");
        Serial.print(digitalRead(ci_lside_IR));
        Serial.print(", Middle_IR = ");
        Serial.print(digitalRead(ci_lside_IR) + digitalRead(ci_rside_IR));
        Serial.print(", Right_IR = ");
        Serial.println(digitalRead(ci_rside_IR));

        /*if (digitalRead(ci_middle_IR)) {   // Middle IR sees correct pyramid
          Serial.println(ci_middle_IR);
          Serial.println("middle");

          }
          if (digitalRead(ci_lside_IR)) {   // Middle IR sees correct pyramid
          Serial.println(ci_lside_IR);
          Serial.println("left");

          }
          if (digitalRead(ci_rside_IR)) {   // Middle IR sees correct pyramid
          Serial.println(ci_rside_IR);
          Serial.println("right");

          }*/
        ul_Smoothing_Counter = millis();


        /*Serial.print("Front = ");
          Serial.println(frontWall_distance);
          Serial.print("Rear  = ");
          Serial.println(rearWall_distance);*/
        break;
      }

    // ---------------------------------------- 1 ------------------------------------------

    case 1:   // Calibrate motor straightness
      {
        if (bt_3_S_Time_Up)
        {
          Serial.println("Case 1");

          if (!bt_Cal_Initialized)
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
          ui_Mode_Indicator_Index = 1;
        }
        break;
      }

    // ---------------------------------------- 2 ------------------------------------------

    case 2: {  // Drive towards wall & look for IO pyramid by default
        if (bt_3_S_Time_Up)
        {
          Serial.println("Case 2");

          servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed - 20);
          servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed - 20);

          Serial.print("faceWall_distance = ");
          Serial.println(faceWall_distance);

          if (faceWall_distance < begin_pyramid_turn_distance - 2) {
            ui_Robot_State_Index = 5;
          }
        }
        break;
      }


    // ---------------------------------------- 3 ------------------------------------------

    case 3:   // Look for AE pyramid (default is IO pyramid)
      {
        if (bt_3_S_Time_Up)
        {
          Serial.println("Case 3");

          correct_pyramid = 0;
          ui_Robot_State_Index = 2;

          break;
        }
      }

    // ---------------------------------------- 4 ------------------------------------------

    case 4:    // Find tesseract
      {
        if (bt_3_S_Time_Up)
        {
          Serial.println("Case 4");

          if (digitalRead(ci_Cube_Microswitch)) {
            ui_Robot_State_Index = 6;
            break;
          }

          // Serial.print("Angle = ");
          // Serial.println(angle_error);
          Serial.print("frontWall_distance = ");      // Absolutely need this uncommented, goes to shit otherwise
          //Serial.println(frontWall_distance);

          if ((frontWall_distance == 0) || (rearWall_distance == 0)) {  // Robot points towards wall
            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed + 30);
            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed - 30);

            break;
          }

          if ((front_error < 2) && (front_error > -2)) {
            if (angle_error > 0) {     // Robot points away from wall
              leftP_factor += angle_error * (50.0);
              Serial.println("Angled out");
              // rightP_factor -= angle_error * (50.0/2);
            }

            if (angle_error < 0) {   // Robot points towards wall
              rightP_factor -= angle_error * (50.0);
              Serial.println("Angled in");
              // leftP_factor += angle_error * (50.0/2);
            }
          }

          if (front_error > 0) {     // Front of robot is too far from wall
            leftP_factor += front_error * (40.0);
            Serial.println("Too far");
            //rightP_factor -= front_error * (50.0 / 2);
          }

          if (front_error < 0) {   // Front of robot is too close to wall
            rightP_factor -= front_error * (80.0);
            leftP_factor -= front_error * (35.0);
            Serial.println("Too close");
            //leftP_factor += front_error * (50.0 / 2);
          }

          servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed + 50 - leftP_factor);
          servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed + 50 - rightP_factor);

          /*Serial.println(leftP_factor);
            Serial.println(rightP_factor);
            Serial.println();*/


          leftP_factor = 0;
          rightP_factor = 0;

          /*Serial.print("Front_error = ");
            Serial.print(front_error);

            Serial.print(",  frontWALLDISTANCE = ");
            Serial.println(frontWall_distance);*/

          Serial.print("face distance = ");
          Serial.println(faceWall_distance);


          if (digitalRead(ci_Cube_Microswitch)) {
            ui_Robot_State_Index = 6;
            continue_wall_following = 1;
            Serial.print("cube found ");

            /* if (counter < 1 ) {
               ui_SAVED_Left_Motor_Speed = ui_Left_Motor_Speed;
               ui_Left_Motor_Speed += 120;
               continue_wall_following = 1;
               counter++;
              }*/
          }

          if (faceWall_distance < begin_turn_distance) {
            if (continue_wall_following == 0) {
              ui_Robot_State_Index = 5;
            }
            else {
              ui_Robot_State_Index = 6;
              ui_Left_Motor_Speed = ui_SAVED_Left_Motor_Speed;
            }
          }

        }
        break;
      }

    // ---------------------------------------- 5 ------------------------------------------

    case 5:   // Turn 90-degrees at wall corners
      {
        if (bt_3_S_Time_Up) {
          Serial.println("Case 5");

          if (Turn90Deg()) {
            ui_Robot_State_Index = 4;
          }
        }
        break;
      }

    // ---------------------------------------- 6 ------------------------------------------

    case 6:   // Find pyramid
      {
        if (bt_3_S_Time_Up) {
          //Serial.println("Case 6");

          // Drive straight
          servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed - 70 + 69);
          servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed - 70 + 69);
          Serial.println("Driving straight");
          Serial.print("faceDistance = ");
          Serial.println(faceWall_distance);
          Serial.print("frontDistance = ");
          Serial.println(frontWall_distance);


          if ((frontWall_distance < 11) && (frontWall_distance != 0)) {
            current_time = millis();
            while (millis() - current_time < time_180deg_turn - 1300) {
              servo_LeftMotor.writeMicroseconds(1670);
              servo_RightMotor.writeMicroseconds(1420);
              Serial.print("turning off wall");
            }
          }

          // Turn if face ultrasonic sensor sees a wall
          if (faceWall_distance < begin_pyramid_turn_distance) {
            //  if (turn_counter % 2 == 0) {
            current_time = millis();

            //Serial.println("Turning right 180 deg");
            while (millis() - current_time < time_180deg_turn) {
              servo_LeftMotor.writeMicroseconds(1660);
              servo_RightMotor.writeMicroseconds(1390);
            }
            turn_counter++;
            // }
            /*else {    // 180-degree turn
              current_time = millis();
              while (millis() - current_time < time_180deg_turn) {
                Serial.println("Turning left 180 deg");
                servo_LeftMotor.writeMicroseconds(1390);
                servo_RightMotor.writeMicroseconds(1660);
              }
              turn_counter++;
              }*/
          }

          if (digitalRead(ci_lside_IR)) {    // Left IR sees correct pyramid
            servo_LeftMotor.writeMicroseconds(1390);
            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed - 55);
            begin_pyramid_turn_distance = 0;
            Serial.println("Left IR sees");
            while ((digitalRead(ci_lside_IR) + digitalRead(ci_rside_IR) < 2) && !(digitalRead(ci_rside_IR))) {
              Serial.println("Left while loop");
            }
          }
          if (digitalRead(ci_rside_IR)) {    // Right IR sees correct pyramid
            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed - 55);
            servo_RightMotor.writeMicroseconds(1390);
            begin_pyramid_turn_distance = 0;
            Serial.println("Right IR sees");
            while ((digitalRead(ci_lside_IR) + digitalRead(ci_rside_IR) < 2) &&
                   !(digitalRead(ci_lside_IR))) {
              Serial.println("Right while loop");
            }
            //Serial.println("OUT OF LOOP");

          }
          if ((digitalRead(ci_lside_IR)) && (digitalRead(ci_rside_IR))) {   // Middle IR sees correct pyramid
            Serial.println("Middle IR sees");
            Serial.println(digitalRead(ci_Pyramid_Microswitch));

            while (!(digitalRead(ci_Pyramid_Microswitch))) {
              servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed - 40);
              servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed - 40);
              begin_pyramid_turn_distance = 0;
              Serial.println("Middle IR LOOP");

            }
            servo_LeftMotor.writeMicroseconds(1500);
            servo_RightMotor.writeMicroseconds(1500);
            ui_Robot_State_Index = 7;
            ul_Smoothing_Counter = millis();
          }
        }
        break;
      }

    // ---------------------------------------- 7 ------------------------------------------

    case 7:   // Grab & lift pyramid
      {
        if (bt_3_S_Time_Up) {
          Serial.println("Case 7");

          servo_StringMotor.writeMicroseconds(2000);

          if ( millis() - ul_Smoothing_Counter < 500) {
            servo_LeftMotor.writeMicroseconds(1380); //FIDDLE WITH
            servo_RightMotor.writeMicroseconds(1380);
          }
          else if ( millis() - ul_Smoothing_Counter < 900) {
            servo_LeftMotor.writeMicroseconds(1500); //FIDDLE WITH
            servo_RightMotor.writeMicroseconds(1500);
          }
          else if ( millis() - ul_Smoothing_Counter < 13000) {
            servo_ArmMotor.write(ci_Arm_Servo_Down + 35);
          }
          else if ( millis() - ul_Smoothing_Counter < 13300) {
            servo_ArmMotor.write(ci_Arm_Servo_Down + 25);
          }
          else if ( millis() - ul_Smoothing_Counter < 13700) {
            servo_ArmMotor.write(ci_Arm_Servo_Down + 15);
          }
          else if ( millis() - ul_Smoothing_Counter < 14100) {
            servo_ArmMotor.write(ci_Arm_Servo_Down + 9);
            servo_GripMotor.write(ci_Grip_Motor_Closed);
          }
          else if ( millis() - ul_Smoothing_Counter < 14900) {
            servo_ArmMotor.write(ci_Arm_Servo_Up - 10);
          }
          else if ( millis() - ul_Smoothing_Counter < 15500) {
            servo_ArmMotor.write(ci_Arm_Servo_Up + 10);
            ui_Robot_State_Index = 8;
            ul_Smoothing_Counter = millis();
          }
        }
        //Serial.print("faceWall_distance = ");
        //Serial.println(faceWall_distance);
        break;
      }

    // ---------------------------------------- 8 ------------------------------------------

    case 8:   // Place pyramid over tesseract
      {
        if (bt_3_S_Time_Up)
        {
          if ( millis() - ul_Smoothing_Counter < 3000) {
            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Reverse);
            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Reverse);
          }
          else if ( millis() - ul_Smoothing_Counter < 3500) {
            servo_LeftMotor.writeMicroseconds(1500);
            servo_RightMotor.writeMicroseconds(1500);
            servo_ArmMotor.write(ci_Arm_Servo_Down + 35);
          }
          else if ( millis() - ul_Smoothing_Counter < 3900) {
            servo_ArmMotor.write(ci_Arm_Servo_Down + 30);
          }
          else if ( millis() - ul_Smoothing_Counter < 4100) {
            servo_ArmMotor.write(ci_Arm_Servo_Down + 25);
          }
          else if ( millis() - ul_Smoothing_Counter < 4300) {
            servo_ArmMotor.write(ci_Arm_Servo_Down + 17);
            servo_GripMotor.write(ci_Grip_Motor_Open);
          }
          else if ( millis() - ul_Smoothing_Counter < 5000) {
            servo_ArmMotor.write(ci_Arm_Servo_Up - 30);
            ui_Robot_State_Index = 0;
          }
        }
        break;
      }

    // ---------------------------------------- 9 ------------------------------------------

    case 9:   // Safety case in case magnet does not pull tesseract in far enough
      {
        if (bt_3_S_Time_Up)
        {
          servo_ArmMotor.write(ci_Arm_Servo_Down);
          break;
        }
      }
  }

  if ((millis() - ul_Display_Time) > ci_Display_Time)
  {
    ul_Display_Time = millis();

#ifdef DEBUG_MODE_DISPLAY
    Serial.print("Mode: ");
    Serial.println(ui_Mode_Indicator[ui_Mode_Indicator_Index], DEC);
#endif
  }
}

// ==================================================== END OF SWITCH STATEMENT ====================================================

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
  return ((ul_Echo_Time / 58.0));
}

boolean Turn90Deg() {
  servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed + 160);
  servo_RightMotor.writeMicroseconds(ui_Right_Motor_Reverse - 20);

  Serial.print("TFangle_error = ");
  Serial.print(angle_error);

  total_diff = 0;
  angles[1] = angles[0];
  angles[0] = angle_error;
  angle_diff[2] = angle_diff[1];
  angle_diff[1] = angle_diff[0];
  angle_diff[0] = angles[1] - angles[0];
  prev_total_diff = total_diff;
  for (int i = 0; i < 3; i++) {
    if (angle_diff[i] > 0) total_diff += angle_diff[i];
    else total_diff -= angle_diff[i];
  }

  Serial.print(", angle_diff[0] = ");
  Serial.print(angle_diff[0]);
  Serial.print(", angle_diff[1] = ");
  Serial.print(angle_diff[1]);
  Serial.print(", angle_diff[2] = ");
  Serial.print(angle_diff[2]);
  Serial.print(", total_diff = ");
  Serial.println(total_diff);

  if ((prev_total_diff < total_diff) && (angle_error < 2)) return 1;
  else return 0;
}

// -----------------------------------------------------------------------------------------
// --------------------------------------Pyramid Decoding------------------------------------
/*
  // Serial.print(digitalRead(ci_Light_Sensor));
  if (digitalRead(ci_Left_Light_Sensor)==0) //if left ir sensor sees the light it should turn left until the
  left doesn't see it anymore and then forward towards it until the middle sensor sees the pyramid
  {
  //***need turning and driving part above here****

  if(digitalRead(ci_Middle_Light_Sensor) == 0) {
    //for AE pyramid because A is (01000001) and has 5 zeros in a row, so when we detect 5 1's in a row we know its the correct pyramid
    if (analogRead(A3)==1) //set equal to 1 because it's the opposite, when it's low, it prints out 1;
       zeroCounter++;
    if (analogRead(A3)==0)
       zeroCounter=0;
    if (zeroCounter== 5)
       correctPyramid= true;
    }
  }
  if (digitalRead(ci_Right_Light_Sensor)==0) //if left ir sensor sees the light it should turn left
  until the left doesn't see it anymore and then forward towards it until the middle sensor sees the pyramid
  {
  //***need turning and driving part above here***

  if(digitalRead(ci_Middle_Light_Sensor) == 0) {
      //for AE pyramid because A is (01000001) and has 5 zeros in a row, so when we detect 5 1's in a row we know its the correct pyramid
    if (analogRead(A3)==1) //set equal to 1 because it's the opposite, when it's low, it prints out 1;
       zeroCounter++;
    if (analogRead(A3)==0)
       zeroCounter=0;
    if (zeroCounter== 5)
      correctPyramid = true;
  }
  }
  if(digitalRead(ci_Middle_Light_Sensor) == 0) {    // Check if IR sensor sees light (1 =  no light, 0 = light)
                                                  // if middle sensor sees the light goes right into decoding mode
  {
  //for AE pyramid because A is (01000001) and has 5 zeros in a row, so when we detect 5 1's in a row we know its the correct pyramid
  if (analogRead(A3)==1) //set equal to 1 because it's the opposite, when it's low, it prints out 1;
  zeroCounter++;
  if (analogRead(A3)==0)
  zeroCounter=0;
  if (zeroCounter== 5)
  return true;
  //******************************IO pyramid decode****************************
  // same above code but swithc if statements, and then depending on the switch it would go to either one
  //for IO pyramid because O is (01001111) and has 4 ones in a row, so when we detect 4 zeroes in a row we know its the correct pyramid
  if (analogRead(A3)==1) //set equal to 1 because it's the opposite, when it's low, it prints out 1;
  zeroCounter=0;
  if (analogRead(A3)==0)
  zeroCounter++;
  if (zeroCounter== 4)
  correctPyramid= true;
*/
//still need to figure out way for it regonize if it's the INCORRECT pyramid

// ----------------------------------------------------------------------------------------
// -----------------------LOCATING PYRAMID DIRECTLY IN FRONT ------------------------------

// if correct pyramid will turn until middle sensors see the light and the drive forward until mircoswitch is activiated on ziptie
/*
  if (correctPyramid==true)
  {
  if(digitalRead(ci_Middle_Light_Sensor) == 1) //turns right when middle sensor doesn't see the light
  {
    servo_LeftMotor.writeMicroseconds(1600);
    servo_RightMotor.writeMicroseconds(1430);
  }
  //once middle sensor sees the light it will drive straight
  servo_LeftMotor.writeMicroseconds(1600);
  servo_RightMotor.writeMicroseconds(1600);

  delay(10); //need this delay before it reads it, because due to the PULLUP function it reads 0 right away and then goes to 1
  pressSwitch = digitalRead(ci_microswitchTesseract);
  Serial.println (pressSwitch, DEC);
  if(pressSwitch == 0) //reads 1 when not pressed and 0 when pressed
  {
     Serial.println("Switch Pressed!"); //this means pyramid is right in front of the robot and can go into Tesseract Placement Mode
     pyramidSwitch= true;

     //makes the robot stop as soon as pyramid is right in front
     servo_LeftMotor.writeMicroseconds(1500);
     servo_RightMotor.writeMicroseconds(1500);
   //delay(1000); can use to be able to read it slower
  }

  if(pyramidSwitch==true) //Tesseract Placement Mode
  {

  }
*/

// ----------------------------------------------------------------------------------------

