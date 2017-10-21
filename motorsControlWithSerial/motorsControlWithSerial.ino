#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <PID_v1.h>

#define DEBUG 0
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

#define Motor2PinA 42
#define Motor2PinB 43
#define Motor2EncoderPinA 21   // Encoder Pin A pin 2 and pin 3 are inturrpt pins (MUST BE 2 OR 3)
#define Motor2EncoderPinB 17   // Encoder Pin B
#define Motor2PWM 44

#define airPressure 22
#define shootButton 23

#define stepMotorStepPin 6
#define stepMotorDirPin 13

#define INT0PIN 2  // <-- change this when not using Arduino Uno
#define INT1PIN 3  // <-- change this when not using Arduino Uno
#define INT2PIN 21  // <-- change this when not using Arduino Uno

#define PIDLIM 100000

//Initialize Variables
int encoder0PinA = -1;
int encoder0PinB = -1;
int encoder1PinA = -1;
int encoder1PinB = -1;
int encoder2PinA = -1;
int encoder2PinB = -1;
long counts0 = 0; //counts the encoder counts.
long counts1 = 0; //counts the encoder counts.
long counts2 = 0; //counts the encoder counts.

boolean shootFlag = false;
boolean readyToShoot = true;;

long counts1Last = 0;
long commandTest = 0;
long commandTestUpdate = -60000;//millis();

double Setpoint = 0;
double Input = 0;
double Output = 0;
int kp = 300;//100;
int ki = 0;
int kd = 9;//18;
double k_amp = 1;//3.75;

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

#include "classes.h"

motor motorA(Motor0PinA, Motor0PinB, Motor0EncoderPinA, Motor0EncoderPinB , Motor0PWM);
motor motorB(Motor1PinA, Motor1PinB, Motor1EncoderPinA, Motor1EncoderPinB , Motor1PWM);
motor motorC(Motor2PinA, Motor2PinB, Motor2EncoderPinA, Motor2EncoderPinB , Motor2PWM);
//motor stepMotor(stepMotorStepPin, stepMotorDirPin);
PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, P_ON_E, DIRECT); //P_ON_M specifies that Proportional on Measurement be used

void setup() {
  Serial.begin(115200);
  pinMode(airPressure,OUTPUT);
  digitalWrite(airPressure,LOW);
  pinMode(shootButton,INPUT);
  //attachInterrupt(0, readEncoder0, CHANGE); //attach interrupt to PIN 2
  //attachInterrupt(1, readEncoder1, CHANGE); //attach interrupt to PIN 3
  attachInterrupt(2, readEncoder2, CHANGE); //attach interrupt to PIN 21
  attachInterrupt(3, shootCommand, FALLING); //attach interrupt to PIN 20
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-PIDLIM, PIDLIM);
  inputString.reserve(200);

}

void loop() {
  if ((shootFlag) && (readyToShoot)) {
    shoot();
    shootFlag = false;
  }
  //motorA.moveMotor();
  Setpoint = (double)commandTest;
  Input = counts1;
  myPID.Compute();
  double nOutput = constrain(map(Output, -PIDLIM*k_amp, PIDLIM*k_amp, -255, 255),-255,255);
 // motorC.moveMotor(nOutput);

  if (stringComplete) {
    //Serial.println(inputString);
    // clear the string:
    commandTest = (((byte)inputString[0]) - 128) * 4;//40;
    //Serial.println(commandTest);
    inputString = "";
    stringComplete = false;
  }

/*  if (millis() - commandTestUpdate > 10000) {
    commandTestUpdate = millis();
    PRINTLNDEBUG(counts1);
    //commandTest = commandTest + 1120*2;
    if (commandTest == 0) commandTest = 1120;
    else commandTest = 0;
    motorA.setCommand(commandTest);
    motorB.setCommand(commandTest);
    stepMotor.moveStepMotor(50,true,1);
  }*/
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

void readEncoder2() //this function is triggered by the encoder CHANGE, and increments the encoder counter
{
  if (digitalRead(encoder2PinA) == digitalRead(encoder2PinB) ) counts1--;
  else counts2++;
}

void shootCommand() //this function is triggered by the encoder CHANGE, and increments the encoder counter
{
   shootFlag = true;
}

void shoot() {
  digitalWrite(airPressure,HIGH);
  delay(100);
  digitalWrite(airPressure,LOW);
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
