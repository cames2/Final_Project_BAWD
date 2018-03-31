


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
const int ci_frontUltrasonic_Ping = A0;   //input plug
const int ci_frontUltrasonic_Data = A1;   //output plug
const int ci_rearUltrasonic_Ping = A2;   //input plug
const int ci_rearUltrasonic_Data = A3;   //output plug
const int ci_faceUltrasonic_Ping = 2;   //input plug
const int ci_faceUltrasonic_Data = 3;   //output plug
const int ci_Charlieplex_LED1 = 4;//for IR now
const int ci_Charlieplex_LED2 = 5;//for IR now
const int ci_Charlieplex_LED3 = 6;
const int ci_Charlieplex_LED4 = 7;
const int ci_Mode_Button = 7;
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Arm_Motor = 10;
const int ci_Grip_Motor = 11;//12 will be last IR
const int ci_Motor_Enable_Switch; //= 12; SWITCHED SO WE CAN USE ULTRASONIC//wire removed
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
const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;

//speeds
const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Grip_Motor_Open = 100;         // Experiment to determine appropriate value
const int ci_Grip_Motor_Closed = 10;        //  "
const int ci_Arm_Servo_Up = 58;      //  "
const int ci_Arm_Servo_Down = 5;      //  "
const int ci_Display_Time = 500;
const int ci_Line_Tracker_Calibration_Interval = 100;
const int ci_Line_Tracker_Cal_Measures = 20;
const int ci_Line_Tracker_Tolerance = 169;   // May need to adjust this
const int ci_Motor_Calibration_Cycles = 3;
const int ci_Motor_Calibration_Time = 5000;
const double wall_distance = 5;     // Centimeters
const double wall_tolerance = 0.5;   // Centimeters
int spinTester = 0;

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

unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;

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

//display Bits 0,1,2,3, 4, 5, 6,  7,  8,  9,  10,  11,  12,  13,   14,   15
int  iArray[16] = {
  1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 65536
};
int  iArrayIndex = 0;

boolean bt_Heartbeat = true;
boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;

int counter = 0;

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
  pinMode(ci_faceUltrasonic_Ping, OUTPUT);
  pinMode(ci_faceUltrasonic_Data, INPUT);

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

  //ping values every loop through
  rearWall_distance = Ping(ci_rearUltrasonic_Ping, ci_rearUltrasonic_Data);
  frontWall_distance = Ping(ci_frontUltrasonic_Ping, ci_frontUltrasonic_Data);
  faceWall_distance = Ping(ci_faceUltrasonic_Ping, ci_faceUltrasonic_Data);
  if (faceWall_distance == 0) {
    faceWall_distance = 20;
  }
  angle_error = frontWall_distance - rearWall_distance;
  front_error = frontWall_distance - wall_distance;

  // check if drive motors should be powered
  bt_Motors_Enabled = digitalRead(ci_Motor_Enable_Switch);

  // modes
  // 0 = default after power up/reset
  // 1 = Run robot.
  // 2 = Grabbing pyramid & Lifting pyramid
  // 3 =
  // 4 =
  // 5 = Calibrate motor speeds to drive straight.
  // 6 = Wall Turn function
  switch (ui_Robot_State_Index)
  {
    case 0:    //Robot stopped
      {
        Ping(ci_frontUltrasonic_Ping, ci_frontUltrasonic_Data);
        servo_ArmMotor.write(ci_Arm_Servo_Up);
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
          // ---------------------------- WALL FOLLOWING with P CONTROLLER ----------------------------

          // Serial.print("Angle = ");
          // Serial.println(angle_error);
          Serial.print("frontWall_distance = ");
          //     Serial.println(frontWall_distance);

          if (angle_error > 0) {     // Robot points away from wall
            leftP_factor += angle_error * 40.0;
          }

          if (angle_error < 0) {   // Robot points towards wall
            rightP_factor -= angle_error * 40.0;
          }

          if (front_error > 0) {     // Front of robot is too far from wall
            leftP_factor += front_error * 40.0;
          }

          if (front_error < 0) {   // Front of robot is too close to wall
            rightP_factor -= front_error * 50.0;
          }

          servo_LeftMotor.writeMicroseconds(1650 - leftP_factor);
          servo_RightMotor.writeMicroseconds(1650 - rightP_factor);

          //Serial.println(leftP_factor);
          //Serial.println(rightP_factor);

          leftP_factor = 0;
          rightP_factor = 0;

          // -----------------------------------------------------------------------------------------

          //Serial.print("  front Distance = ");
          //Serial.print(frontWall_distance);
          //Serial.print("     ");

          if (faceWall_distance < 12)
          {
            ui_Robot_State_Index = 6;
          }
        }

        /*#ifdef DEBUG_ENCODERS
          l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
          l_Right_Motor_Position = encoder_RightMotor.getRawPosition();

          Serial.print("Encoders L: ");
          Serial.print(l_Left_Motor_Position);
          Serial.print(", R: ");
          Serial.println(l_Right_Motor_Position);
          #endif*/

        // set motor speeds
        /*  ui_Left_Motor_Speed = constrain(ui_Motors_Speed + ui_Left_Motor_Offset, 1600, 2100);
          ui_Right_Motor_Speed = constrain(ui_Motors_Speed + ui_Right_Motor_Offset, 1600, 2100);
          ui_Left_Motor_Reverse = constrain(ui_Motors_Reverse - ui_Left_Motor_Offset, 900, 1500);
          ui_Right_Motor_Reverse = constrain(ui_Motors_Reverse - ui_Right_Motor_Offset, 900, 1500);
        */
        break;
      }

    case 2://when pyramid is in right spot
      {
        if (bt_3_S_Time_Up)
        {
          if (faceWall_distance < 7) {
            counter++;
          }

          if (counter < 10) {
            servo_ArmMotor.write(ci_Arm_Servo_Down + 25);
          }

          else if (counter < 25) {
            servo_ArmMotor.write(ci_Arm_Servo_Down + 18);
          }

          else if (counter < 45) {
            servo_ArmMotor.write(ci_Arm_Servo_Down + 12);
          }

          else if (counter < 60) {
            servo_GripMotor.write(ci_Grip_Motor_Closed);
          }
          else if (counter < 80) {
            servo_ArmMotor.write(ci_Arm_Servo_Up - 35);
          }
          else if (counter < 100) {
            servo_ArmMotor.write(ci_Arm_Servo_Up - 25);
          }
          else if (counter > 100) {
            servo_ArmMotor.write(ci_Arm_Servo_Up - 12);
            counter = 1000;
            ui_Robot_State_Index = 3;
          }
        }
        Serial.print("faceWall_distance = ");
        Serial.println(faceWall_distance);
        break;
      }
    case 3:
      {
        if (bt_3_S_Time_Up)
        {
          counter--;

          if (counter > 870) {
            servo_LeftMotor.writeMicroseconds(1400);
            servo_RightMotor.writeMicroseconds(1400);
          }
          else if (counter > 800) {
            servo_LeftMotor.writeMicroseconds(1500);
            servo_RightMotor.writeMicroseconds(1500);
            servo_ArmMotor.write(ci_Arm_Servo_Down + 35);
          }
          else if (counter > 790) {
            servo_ArmMotor.write(ci_Arm_Servo_Down + 30);
          }
          else if (counter > 780) {
            servo_ArmMotor.write(ci_Arm_Servo_Down + 25);
          }
          else if (counter > 750) {
            servo_ArmMotor.write(ci_Arm_Servo_Down + 17);
            servo_GripMotor.write(ci_Grip_Motor_Open);
          }
          else if (counter > 740) {
            servo_ArmMotor.write(ci_Arm_Servo_Up - 30);
          }
          else {
            ui_Robot_State_Index = 0;
          }
        }
        break;
      }

    case 4://tesseract dropping if we dont have enough range with the pyramid pick up
      {
        servo_ArmMotor.write(ci_Arm_Servo_Down
        
        );
        break;
      }



    case 5:    //Calibrate motor straightness after 3 seconds.
      {

        if (bt_3_S_Time_Up)
        {
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
          ui_Mode_Indicator_Index = 5;
        }
        break;

      }

    case 6: //turn function
      {
        if (bt_3_S_Time_Up) {
          servo_LeftMotor.writeMicroseconds(1620);
          servo_RightMotor.writeMicroseconds(1440);
          Serial.print("angle_error = ");
          Serial.print(angle_error);
          total_diff = 0;
          angles[1] = angles[0];
          angles[0] = angle_error;
          angle_diff[2] = angle_diff[1];
          angle_diff[1] = angle_diff[0];
          angle_diff[0] = angles[1] - angles[0];
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
        }

        if (total_diff < 2 && angle_error < 0) {
          ui_Robot_State_Index = 1;
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

 // -----------------------------------------------------------------------------------------
// --------------------------------------Pyramid Decoding------------------------------------
/*
// Serial.print(digitalRead(ci_Light_Sensor));
if (digitalRead(ci_Left_Light_Sensor)==0) //if left ir sensor sees the light it should turn left until the left doesn't see it anymore and then forward towards it until the middle sensor sees the pyramid
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
if (digitalRead(ci_Right_Light_Sensor)==0) //if left ir sensor sees the light it should turn left until the left doesn't see it anymore and then forward towards it until the middle sensor sees the pyramid
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

