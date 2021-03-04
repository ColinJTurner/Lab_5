
//Pin assignments

const int ciHeartbeatLED = 2;
const int ciPB1 = 27;
const int ciPB2 = 26;
const int ciPot1 = A4;
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
const int ciSmartLED = 25;
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
const int CR1_ciHeartbeatInterval = 500;
const int CR1_ciMotorRunTime = 250; // might have to change to allow for full operation
const long CR1_clDebounceDelay = 50;
const long CR1_clReadTimeout = 220;
int count = 0;

const uint8_t ci8RightTurn = 18;
const uint8_t ci8LeftTurn = 17;

unsigned char CR1_ucMainTimerCaseCore1;
uint8_t CR1_ui8LimitSwitch;

char CR1_ui8IRDatum; // char is used to convert signal directly into U or A
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

  pinMode(LED_BUILTIN, OUTPUT);
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
  pinMode(ciHeartbeatLED, OUTPUT);
  pinMode(ciPB1, INPUT_PULLUP);
  pinMode(ciLimitSwitch, INPUT_PULLUP);

}

void loop()
{

  //WSVR_BreakPoint(1);

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
          count = 0;// resets count whenever button is pressed
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

  //ir stuff here

  if (Serial2.available() > 0) {               // check for incoming data
    CR1_ui8IRDatum = Serial2.read();          // read the incoming byte and turns into char
    Serial.println(CR1_ui8IRDatum);        // Prints either a U or A
    CR1_ulLastByteTime = millis();            // capture time last byte was received
  }
  else
  {
    // check to see if elapsed time exceeds allowable timeout
    if (millis() - CR1_ulLastByteTime > CR1_clReadTimeout) {
      CR1_ui8IRDatum = 0;                     // if so, clear incoming byte
      Serial.println('F'); // prints F if no signal is found, was used to test the TSOP shield used to prevent IR light from leaking in
    }
  }

  if (CR1_ui8IRDatum == 'U' && ucMotorStateIndex < 2) { //creates a loop for the robot to move forward when U is detected and when its not in the end of the cycle
    ucMotorStateIndex = 1;
  }
  if (CR1_ui8IRDatum == 'A' && ucMotorStateIndex < 2) { // creates a reverse loop while A is detected until the count if statement is triggered
    ucMotorStateIndex = 2;
  }
  if (CR1_ui8IRDatum != 'A' && CR1_ui8IRDatum != 'U' && ucMotorStateIndex < 2) { // creates a searching loop when both A or U is not detected
    ucMotorStateIndex = 0;
  }


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

                case 0: // Searching Loop, previous if statement locks its here until U is detected
                  {
                    ENC_SetDistance(-5, 5);
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed - 65;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed - 55;
                    ucMotorState = 2;  //left

                    break;
                  }

                case 1: // Forward loop, causing the robot to move towards the beacon when U is detected
                  {

                    ENC_SetDistance(20, 20);
                    ucMotorState = 1;   //forward
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;

                    break;
                  }

                case 2: // Reverse loop, revereses the robot while A is detected until it has travelled back half the course distance
                  {
                    ucMotorState = 4;  //reverse
                    ENC_SetDistance(-40, -40);
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    count++;
                    //Serial.println(count);
                    if (count == 8) { // when the robot has cycles 8 times it moves to the 180 degrees turn
                      ucMotorStateIndex = 3;
                    }

                    break;
                  }

                case 3: // 180 degrees turn when count is 8
                  {
                    ENC_SetDistance(ci8RightTurn, -(ci8RightTurn));
                    CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
                    CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
                    ucMotorState = 3;  //right
                    count++;
                    //Serial.println(count);
                    if (count == 11) { // when count reaches 11 the robot moves the the breaking loop
                      ucMotorStateIndex = 4;
                    }
                    break;
                  }

                case 4: // breaking loop
                  {
                    ucMotorState = 5;
                    move(0);
                    break;
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
              MoveTo(ucMotorState, CR1_ui8LeftWheelSpeed, CR1_ui8RightWheelSpeed + 10); // right motor is slightly increase to make up for physical resistance
            }
            CR1_ucMainTimerCaseCore1 = 0;
            break;
          }
        }
    }
  }
}
