

#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <SoftwareSerial.h>

// IR pin numbers
const int lside_input_pin = 2;
const int rside_input_pin = 3;
const int middle_input_pin = 4;
const int lside_output_pin = 5;
const int rside_output_pin = 6;
const int middle_output_pin = 7;

// CharliePlex pin numbers
const int ci_Charlieplex_LED1;    // Not in use
const int ci_Charlieplex_LED2;    // Not in use
const int ci_Charlieplex_LED3 = 8;
const int ci_Charlieplex_LED4 = 9;
const int ci_Mode_Button = 9;

// MicroSwitch pin numbers
const int cube_switch_input_pin = 10;
const int pyramid_switch_input_pin = 11;
const int cube_switch_output_pin = 12;
const int pyramid_switch_output_pin = 13;


// Charlieplexing LED assignments
const int ci_Heartbeat_LED = 1;
const int ci_Indicator_LED = 4;
const int ci_Right_IR_LED = 12;
const int ci_Middle_IR_LED = 9;
const int ci_Left_IR_LED = 6;

unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;

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

// Initialize software serial ports
SoftwareSerial lside_IR(lside_input_pin, lside_output_pin);
SoftwareSerial rside_IR(rside_input_pin, rside_output_pin);
SoftwareSerial middle_IR(middle_input_pin, middle_output_pin);

// IR output variables
int sensor_index = 0;
char left_output = NULL;
char right_output = NULL;
char middle_output = NULL;
boolean correct_pyramid = 0;  // 0 = AE, 1 = IO
unsigned long current_time = 0;

// MicroSwitch variables
boolean switch_output = 0;    // 0 = not pressed, 1 = pressed
unsigned long microswitch_time = 0;

void setup() {
  MicroSwitch_setup();
  CharliePlex_setup();
  IR_setup();
}

void MicroSwitch_setup() {
  pinMode(cube_switch_input_pin, INPUT_PULLUP);
  pinMode(pyramid_switch_input_pin, INPUT_PULLUP);
  pinMode(cube_switch_output_pin, OUTPUT);
  pinMode(pyramid_switch_output_pin, OUTPUT);
  delay(100);   // Pullup function causes an initial reading of 0, must wait for it to read 1
}

void CharliePlex_setup() {
  CharliePlexM::setBtn(ci_Charlieplex_LED1, ci_Charlieplex_LED2,
                       ci_Charlieplex_LED3, ci_Charlieplex_LED4, ci_Mode_Button);
}

void IR_setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  if (sensor_index == 0) {
    lside_IR.begin(2400);
  }
  if (sensor_index == 1) {
    rside_IR.begin(2400);
  }
  if (sensor_index == 2) {
    middle_IR.begin(2400);
  }
  Serial.print("Begin");
  current_time = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
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

  // MicroSwitch readings
  if (digitalRead(cube_switch_input_pin) == 0) {
    digitalWrite(cube_switch_output_pin, HIGH);
    switch_output = 1;
    microswitch_time = millis();
  }
  if (digitalRead(pyramid_switch_input_pin) == 0) {
    digitalWrite(pyramid_switch_output_pin, HIGH);
    switch_output = 2;
    microswitch_time = millis();
  }
  if ((switch_output == 1) && (millis() - microswitch_time > 1000)) {
    digitalWrite(cube_switch_output_pin, LOW);
    switch_output = 0;
  }
  if ((switch_output == 2) && (millis() - microswitch_time > 1000)) {
    digitalWrite(pyramid_switch_output_pin, LOW);
    switch_output = 0;
  }

  // IR serial decoding
  switch (ui_Robot_State_Index) {
    case 0: {   // Leave empty
        break;
      }
    case 1: {   // Look for AE pyramid
        if (bt_3_S_Time_Up) {
          correct_pyramid = 0;
          ui_Robot_State_Index = 3;
          break;
        }
      }
    case 2: {   // Look for IO pyramid
        if (bt_3_S_Time_Up) {
          correct_pyramid = 1;
          ui_Robot_State_Index = 3;
          break;
        }
      }
    case 3: {
        if (bt_3_S_Time_Up) {
          if (sensor_index == 0) {
            if (lside_IR.available()) {
              Serial.print("Left sensor available.   Output = ");
              left_output = lside_IR.read();
              Serial.print(left_output);
              Serial.println();
              if ((left_output == 'A') || (left_output == 'E')) {
                if (correct_pyramid == 0) digitalWrite(lside_output_pin, HIGH);
              }
              if ((left_output == 'I') || (left_output == 'O')) {
                if (correct_pyramid == 1) digitalWrite(lside_output_pin, HIGH);
              }
            }
          }
          if (sensor_index == 1) {
            if (rside_IR.available()) {
              Serial.print("Right sensor available.  Output = ");
              right_output = rside_IR.read();
              Serial.print(right_output);
              Serial.println();
              if ((right_output == 'A') || (right_output == 'E')) {
                if (correct_pyramid == 0) digitalWrite(rside_output_pin, HIGH);
              }
              if ((right_output == 'I') || (right_output == 'O')) {
                if (correct_pyramid == 1) digitalWrite(rside_output_pin, HIGH);
              }
            }
          }

          if (sensor_index == 2) {
            if (middle_IR.available()) {
              Serial.print("Middle sensor available. Output = ");
              middle_output = middle_IR.read();
              Serial.write(middle_output);
              Serial.println();
              if ((middle_output == 'A') || (middle_output == 'E')) {
                if (correct_pyramid == 0) digitalWrite(middle_output_pin, HIGH);
              }
              if ((middle_output == 'I') || (middle_output == 'O')) {
                if (correct_pyramid == 1) digitalWrite(middle_output_pin, HIGH);
              }
            }
          }

          if (++sensor_index == 3) {
            sensor_index = 0;
          }
          if (millis() - current_time > 100) {
            lside_IR.end();
            rside_IR.end();
            middle_IR.end();
            Serial.end();
            IR_setup();
          }

          break;
        }
      }
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
