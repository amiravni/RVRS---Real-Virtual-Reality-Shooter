#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <PID_v1.h>

#define DEBUG 1
#define PRINTDEBUG(STR) \
  {  \
    if (DEBUG) Serial.print(STR); \
  }
#define PRINTLNDEBUG(STR) \
  {  \
    if (DEBUG) Serial.println(STR); \
  }
//Define Pins
#define Motor0PinA 7
#define Motor0PinB 8
#define Motor0EncoderPinA 2   // Encoder Pin A pin 2 and pin 3 are inturrpt pins (MUST BE 2 OR 3)
#define Motor0EncoderPinB 4   // Encoder Pin B
#define Motor0PWM 5

#define Motor1PinA 10
#define Motor1PinB 9
#define Motor1EncoderPinA 3   // Encoder Pin A pin 2 and pin 3 are inturrpt pins (MUST BE 2 OR 3)
#define Motor1EncoderPinB 12   // Encoder Pin B
#define Motor1PWM 11

#define stepMotorStepPin 6
#define stepMotorDirPin 13

#define INT0PIN 2  // <-- change this when not using Arduino Uno
#define INT1PIN 3  // <-- change this when not using Arduino Uno

#define PIDLIM 100000

//Initialize Variables
int encoder0PinA = -1;
int encoder0PinB = -1;
int encoder1PinA = -1;
int encoder1PinB = -1;
long counts0 = 0; //counts the encoder counts.
long counts1 = 0; //counts the encoder counts.
long counts1Last = 0;
long commandTest = 0;
long commandTestUpdate = -60000;//millis();

double Setpoint = 0;
double Input = 0;
double Output = 0;
int kp = 50;
int ki = 5;
int kd = 0;
double k_amp = 3.75;

#include "classes.h"

motor motorA(Motor0PinA, Motor0PinB, Motor0EncoderPinA, Motor0EncoderPinB , Motor0PWM);
motor motorB(Motor1PinA, Motor1PinB, Motor1EncoderPinA, Motor1EncoderPinB , Motor1PWM);
motor stepMotor(stepMotorStepPin, stepMotorDirPin);
PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, P_ON_E, DIRECT); //P_ON_M specifies that Proportional on Measurement be used

void setup() {
  if (DEBUG) Serial.begin(115200);
  //attachInterrupt(0, readEncoder0, CHANGE); //attach interrupt to PIN 2
  attachInterrupt(1, readEncoder1, CHANGE); //attach interrupt to PIN 3
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-PIDLIM, PIDLIM);

}

void loop() {
  //motorA.moveMotor();
  Setpoint = (double)commandTest;
  Input = counts1;

  myPID.Compute();
  double nOutput = constrain(map(Output, -PIDLIM*k_amp, PIDLIM*k_amp, -255, 255),-255,255);
  if (abs(nOutput) > 3 && abs(nOutput) < 29) {
    if (nOutput > 0) nOutput = 29;
    else nOutput = -29;
  }
  motorB.moveMotor(nOutput);

  if (millis() - commandTestUpdate > 60000) {
    commandTestUpdate = millis();
    PRINTLNDEBUG(counts1);
    commandTest = commandTest + 1120*6;
    motorA.setCommand(commandTest);
    motorB.setCommand(commandTest);
    stepMotor.moveStepMotor(50,true,1);
/*    for (int iii = 0; iii < 20; iii++) {
    myPID.Compute();
    PRINTDEBUG(Setpoint);
    PRINTDEBUG(" , ");
    PRINTLNDEBUG(Output);
    }
    while(1) {
      
    }*/
  }

  /* Stepper Test */ /*
    digitalWrite(dir, LOW);
    for (int jjj = 0 ; jjj < 4 ; jjj++) {
    for (int iii = 0 ; iii < 50 ; iii++) {
      digitalWrite(stp, HIGH);
      delay(1);
      digitalWrite(stp, LOW);
      delay(1);
    }

    delay(1000);
    }
    digitalWrite(dir, HIGH);
    for (int iii = 0 ; iii < 50 ; iii++) {
      digitalWrite(stp, HIGH);
      delay(1);
      digitalWrite(stp, LOW);
      delay(1);
    }
    delay(1000);
  */
}

void readEncoder0() //this function is triggered by the encoder CHANGE, and increments the encoder counter
{
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB) ) counts0--;
  else counts0++;
}

void readEncoder1() //this function is triggered by the encoder CHANGE, and increments the encoder counter
{
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB) ) counts1--;
  else counts1++;
}


