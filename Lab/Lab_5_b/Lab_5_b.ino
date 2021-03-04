
//Pin assignments

const int ciPB1 = 27;
const int ciPB2 = 26;
const int ciPot1 = A4;    //GPIO 32  - when JP2 has jumper installed Analog pin AD4 is connected to Poteniometer R1
const int ciLimitSwitch = 26;
const int ciIRDetector = 15;
const int ciMotorLeftA = 4;
const int ciMotorLeftB = 18;
const int ciMotorRightA = 19;
const int ciMotorRightB = 12;
const int ciEncoderLeftA = 17;
const int ciEncoderLeftB = 5;
const int ciEncoderRightA = 14;
const int ciEncoderRightB = 13;
const int ciStepperMotorDir = 22;
const int ciStepperMotorStep = 21;

volatile uint32_t vui32test1;
volatile uint32_t vui32test2;

#include "0_Core_Zero.h"

#include <esp_task_wdt.h>

#include <Math.h>
#include "Motion.h";
#include "MyWEBserver.h"
#include "BreakPoint.h"
#include "WDT.h";

void loopWEBServerButtonresponce(void);

const int CR1_ciMainTimer =  1000;
const int CR1_ciMotorRunTime = 1500; // might have to change to allow for full operation, time was extended to allow for initial operation
const long CR1_clDebounceDelay = 50;
const long CR1_clReadTimeout = 220;
int count = 0; // used to break the loop in the cases

const uint8_t ci8RightTurn = 18;
const uint8_t ci8LeftTurn = 17;

unsigned char CR1_ucMainTimerCaseCore1;
uint8_t CR1_ui8LimitSwitch;

char CR1_ui8IRDatum; // char was used to automatically decode signal
uint8_t CR1_ui8WheelSpeed;
uint8_t CR1_ui8LeftWheelSpeed;
uint8_t CR1_ui8RightWheelSpeed;

uint32_t CR1_u32Now;
uint32_t CR1_u32Last;
uint32_t CR1_u32Temp;
uint32_t CR1_u32Avg;

unsigned long CR1_ulLastDebounceTime;
unsigned long CR1_ulLastByteTime;

unsigned long CR1_ulMainTimerPrevious;
unsigned long CR1_ulMainTimerNow;

unsigned long CR1_ulMotorTimerPrevious;
unsigned long CR1_ulMotorTimerNow;
unsigned char ucMotorStateIndex = 0;

unsigned long CR1_ulHeartbeatTimerPrevious;
unsigned long CR1_ulHeartbeatTimerNow;

boolean btHeartbeat = true;
boolean btRun = false;
boolean btToggle = true;
int iButtonState;
int iLastButtonState = HIGH;

void setup() {

  Serial.begin(115200);
  Serial2.begin(2400, SERIAL_8N1, ciIRDetector);  // IRDetector on RX2 receiving 8-bit words at 2400 baud

  Core_ZEROInit();

  WDT_EnableFastWatchDogCore1();
  WDT_ResetCore1();
  WDT_vfFastWDTWarningCore1[0] = 0;
  WDT_vfFastWDTWarningCore1[1] = 0;
  WDT_vfFastWDTWarningCore1[2] = 0;
  WDT_vfFastWDTWarningCore1[3] = 0;
  WDT_ResetCore1();
  WDT_vfFastWDTWarningCore1[4] = 0;
  WDT_vfFastWDTWarningCore1[5] = 0;
  WDT_vfFastWDTWarningCore1[6] = 0;
  WDT_vfFastWDTWarningCore1[7] = 0;
  WDT_ResetCore1();
  WDT_vfFastWDTWarningCore1[8] = 0;
  WDT_vfFastWDTWarningCore1[9] = 0;
  WDT_ResetCore1();

  setupMotion();
  pinMode(ciPB1, INPUT_PULLUP);
  pinMode(ciLimitSwitch, INPUT_PULLUP);
}

void loop()
{

  //average the encoder tick times
  ENC_Averaging();

  int iButtonValue = digitalRead(ciPB1);       // read value of push button 1
  if (iButtonValue != iLastButtonState) {      // if value has changed
    CR1_ulLastDebounceTime = millis();        // reset the debouncing timer
  }

  if ((millis() - CR1_ulLastDebounceTime) > CR1_clDebounceDelay) {
    if (iButtonValue != iButtonState) {        // if the button state has changed
      iButtonState = iButtonValue;               // update current button state

      // only toggle the run condition if the new button state is LOW
      if (iButtonState == LOW)
      {
        ENC_ClearLeftOdometer();
        ENC_ClearRightOdometer();
        btRun = !btRun;
        Serial.println(btRun);
        // if stopping, reset motor states and stop motors
        if (!btRun)
        {
          ucMotorStateIndex = 0;
          count = 0; // resets the count along with the other variables when button is used
          ucMotorState = 0;
          move(0);
        }

      }
    }
  }
  iLastButtonState = iButtonValue;             // store button state

  if (!digitalRead(ciLimitSwitch))
  {
    btRun = 0; //if limit switch is pressed stop bot
    ucMotorStateIndex = 0;
    ucMotorState = 0;
    move(0);
  }

  if (Serial2.available() > 0) {               // check for incoming data
    CR1_ui8IRDatum = Serial2.read();          // automatically converts signal into U or A
    //Serial.println(CR1_ui8IRDatum);        // prints the signal into the serial monitor, used for testing purposes
    CR1_ulLastByteTime = millis();            // capture time last byte was received
  }
  else
  {
    // check to see if elapsed time exceeds allowable timeout
    if (millis() - CR1_ulLastByteTime > CR1_clReadTimeout) {
      CR1_ui8IRDatum = 0;                     // if so, clear incoming byte
      //Serial.println('F');                  // prints F if no signal was detected, used for testing purposes
    }
  }

  if (CR1_ui8IRDatum == 'U' && ucMotorStateIndex > 7 && ucMotorStateIndex < 10) { // if the index is  inbetween the ranges and U is detected it locks in forward case
    ucMotorStateIndex = 9;
  }
  if (CR1_ui8IRDatum == 'A' && ucMotorStateIndex > 7 && ucMotorStateIndex < 10) { // if the index is  inbetween the ranges and A is detected it locks in reverse case
    ucMotorStateIndex = 10;
  }
  if (CR1_ui8IRDatum != 'A' && CR1_ui8IRDatum != 'U' && ucMotorStateIndex > 7 && ucMotorStateIndex < 10) { // allows for a loop for searching
    ucMotorStateIndex = 8;
  }
  Serial.println(ucMotorStateIndex); // prints index state, for testing purposes

  CR1_ulMainTimerNow = micros();
  if (CR1_ulMainTimerNow - CR1_ulMainTimerPrevious >= CR1_ciMainTimer)
  {
    WDT_ResetCore1();
    WDT_ucCaseIndexCore0 = CR0_ucMainTimerCaseCore0;

    CR1_ulMainTimerPrevious = CR1_ulMainTimerNow;

    switch (CR1_ucMainTimerCaseCore1) //full switch run through is 1mS
    {
      //###############################################################################
      case 0:
        {

          if (btRun)
          {



            CR1_ulMotorTimerNow = millis();
            if (CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorRunTime)
            {
              CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;
              switch (ucMotorStateIndex)
              {

                case 0: // case 0 - 7 are manual inputs to navigate around the box
                  {

                    ENC_SetDistance(250, 250);
                    ucMotorState = 1;   //forward
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    ucMotorStateIndex = 1;

                    break;
                  }

                case 1:
                  {
                    ENC_SetDistance(-(ci8LeftTurn), ci8LeftTurn);
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    ucMotorStateIndex = 2;
                    ucMotorState = 2;  //left

                    break;
                  }

                case 2:
                  {

                    ENC_SetDistance(100, 100);
                    ucMotorState = 1;   //forward
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    ucMotorStateIndex = 3;

                    break;
                  }

                case 3:
                  {
                    ENC_SetDistance(ci8RightTurn, -(ci8RightTurn));
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    ucMotorStateIndex =  4;
                    ucMotorState = 3;  //right

                    break;
                  }

                case 4:
                  {

                    ENC_SetDistance(175, 175);
                    ucMotorState = 1;   //forward
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    ucMotorStateIndex = 5;

                    break;
                  }

                  case 5:
                  {
                    ENC_SetDistance(ci8RightTurn, -(ci8RightTurn));
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    ucMotorStateIndex =  6;
                    ucMotorState = 3;  //right

                    break;
                  }
                  
                  case 6:
                  {

                    ENC_SetDistance(100, 100);
                    ucMotorState = 1;   //forward
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    ucMotorStateIndex = 7;

                    break;
                  }

                  case 7: 
                  {
                    ENC_SetDistance(-(ci8LeftTurn), ci8LeftTurn);
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    ucMotorStateIndex = 8;
                    ucMotorState = 2;  //left

                    break;
                  }

                case 8:   // automatic searching takes place here, the robot will rotate until beacon is detected
                  {
                    ENC_SetDistance(-5, 5);
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed - 65; // offsets were put in place to reduce sway from path
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed - 55;
                    ucMotorState = 2;  //left

                    break;
                  }

                case 9:   // automatic following of the beacon takes place here, robot inches forward until it hits beacon(go to 10) or looses beacon (go to 8)
                  {

                    ENC_SetDistance(20, 20);
                    ucMotorState = 1;   //forward
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;

                    break;
                  }

                case 10:  // when the beacon is hit and the char changes the robot will reverse then move to the next state
                  {
                    ucMotorState = 4;  //reverse
                    ENC_SetDistance(-200, -200);
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    ucMotorStateIndex = 11;

                    break;
                  }

                case 11:  // the robot will rotate 180 degrees here
                  {
                    ENC_SetDistance(ci8RightTurn, -(ci8RightTurn));
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    ucMotorState = 3;  //right
                    count++;
                    //Serial.println(count);
                    if (count == 2) {
                      ucMotorStateIndex = 12;
                    }
                    break;
                  }

                case 12: // robot find resting place
                  {
                    ucMotorState = 5;
                    move(0);
                    break;
                  }
              }
            }
          }
          CR1_ucMainTimerCaseCore1 = 1;

          break;
        }
      //###############################################################################
      case 1:
        {
          //read pot 1 for motor speeds
          CR1_ui8WheelSpeed = map(analogRead(ciPot1), 0, 4096, 130, 255);  // adjust to range that will produce motion

          CR1_ucMainTimerCaseCore1 = 2;
          break;
        }

      // case 2 actually does the moving of the robot can move it into case 0 to eliminate this case and make it non time restricted
      case 2:
        {
          //move bot X number of odometer ticks
          if (ENC_ISMotorRunning())
          {
            MoveTo(ucMotorState, CR1_ui8LeftWheelSpeed, CR1_ui8RightWheelSpeed + 15); // right wheel is slighly increase to make up for offset of friction
          }
          
          CR1_ucMainTimerCaseCore1 = 0;
          break;
        }    
    }
  } 
}
