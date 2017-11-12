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

// Motor0 -> Azimuth
#define Motor0PinA 39
#define Motor0PinB 47
#define Motor0EncoderPinA 20   // Encoder Pin A pin 2 and pin 3 are inturrpt pins (MUST BE 2 OR 3)
#define Motor0EncoderPinB 25  // Encoder Pin B
#define Motor0PWM 46
#define Motor0INT 3

// Motor1 -> Elevation
#define Motor1PinA 51
#define Motor1PinB 49
#define Motor1EncoderPinA 21   // Encoder Pin A pin 2 and pin 3 are inturrpt pins (MUST BE 2 OR 3)
#define Motor1EncoderPinB 31   // Encoder Pin B
#define Motor1PWM 44
#define Motor1INT 2

// Motor1 -> Reload
#define Motor2PinA 43
#define Motor2PinB 41
#define Motor2EncoderPinA 18   // Encoder Pin A pin 2 and pin 3 are inturrpt pins (MUST BE 2 OR 3)
#define Motor2EncoderPinB 27   // Encoder Pin B
#define Motor2PWM 45
#define Motor2INT 5

#define laserPin 37
#define airPressurePin 35
#define airPressurePinLow 33
#define shootButton 2
#define shootButtonInt 0

#define stepMotorStepPin 6
#define stepMotorDirPin 13

#define PIDLIM 50000
#define EL_MAXLIM_ENC 500 
#define EL_MINLIM_ENC -500 

#define MOTOR_C_QRT_RND 1050

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

volatile boolean shootFlag = false;
boolean readyToShoot = true;;

long counts1Last = 0;
long commandAz = 0;
long commandEl = 0;
long commandEnc = 0;
long commandTestUpdate = -60000;//millis();

// PID0
double Setpoint0 = 0;
double Input0 = 0;
double Output0 = 0;
int kp0 = 100;
int ki0 = 0;
int kd0 = 18;
double k_att0 = 1;//3.75;


// PID1
double Setpoint1 = 0;
double Input1 = 0;
double Output1 = 0;
int kp1 = 200;
int ki1 = 0;
int kd1 = 20;
double k_att1 = 1;//3.75;

// PID2
double Setpoint2 = 0;
double Input2 = 0;
double Output2 = 0;
int kp2 = 100;
int ki2 = 5;
int kd2 = 15;
double k_att2 = 2;//3.75;

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete


#include "classes.h"

motor motor0(Motor0PinA, Motor0PinB, Motor0EncoderPinA, Motor0EncoderPinB , Motor0PWM,0);
motor motor1(Motor1PinA, Motor1PinB, Motor1EncoderPinA, Motor1EncoderPinB , Motor1PWM,1);
motor motor2(Motor2PinA, Motor2PinB, Motor2EncoderPinA, Motor2EncoderPinB , Motor2PWM,2);
PID myPID0(&Input0, &Output0, &Setpoint0, kp0, ki0, kd0, P_ON_E, DIRECT); //P_ON_M specifies that Proportional on Measurement be used
PID myPID1(&Input1, &Output1, &Setpoint1, kp1, ki1, kd1, P_ON_E, DIRECT); //P_ON_M specifies that Proportional on Measurement be used
PID myPID2(&Input2, &Output2, &Setpoint2, kp2, ki2, kd2, P_ON_E, DIRECT); //P_ON_M specifies that Proportional on Measurement be used

void setup() {
  Serial.begin(115200);
  pinMode(airPressurePin,OUTPUT);
  pinMode(airPressurePinLow,OUTPUT);
  digitalWrite(airPressurePinLow,LOW);
  digitalWrite(airPressurePin,LOW);  
  pinMode(laserPin,OUTPUT);
  digitalWrite(laserPin,HIGH);    
  pinMode(shootButton,INPUT_PULLUP);  
  attachInterrupt(Motor0INT, readEncoder0, CHANGE); //attach interrupt to PIN 2
  attachInterrupt(Motor1INT, readEncoder1, CHANGE); //attach interrupt to PIN 3
  attachInterrupt(Motor2INT, readEncoder2, CHANGE); //attach interrupt to PIN 21
  attachInterrupt(shootButtonInt, shootCommand, FALLING); //attach interrupt to PIN 20
  myPID0.SetMode(AUTOMATIC);
  myPID0.SetOutputLimits(-PIDLIM, PIDLIM);
  myPID1.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(-PIDLIM, PIDLIM);
  myPID2.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(-PIDLIM, PIDLIM);
  inputString.reserve(200);

}

void loop() {
  if ((shootFlag) && (readyToShoot)) {
    shoot();
    shootFlag = false;
    readyToShoot = false;
    commandEnc = commandEnc + MOTOR_C_QRT_RND;
  }


  Setpoint0 = (double)commandAz;
  Input0 = counts0;   //<===========================
  myPID0.Compute();
  double nOutput0 = constrain(map(Output0, -PIDLIM*k_att0, PIDLIM*k_att0, -255, 255),-255,255);
  motor0.moveMotor(nOutput0); //<===========================

  Setpoint1 = (double)commandEl;
  Input1 = counts1;   //<===========================
  myPID1.Compute();
  double nOutput1 = constrain(map(Output1, -PIDLIM*k_att1, PIDLIM*k_att1, -255, 255),-255,255);
  motor1.moveMotor(nOutput1); //<===========================

 

  Setpoint2 = (double)commandEnc;
  Input2 = counts2;   //<===========================
  myPID2.Compute();
  double nOutput2 = constrain(map(Output2, -PIDLIM*k_att2, PIDLIM*k_att2, -255, 255),-255,255);
  motor2.moveMotor(nOutput2); //<===========================

 /* if (commandEnc > 0) {
    PRINTDEBUG("-->");
    PRINTDEBUG( Setpoint2);
    PRINTDEBUG(" , ");
    PRINTDEBUG( Input2)
    PRINTDEBUG(" , ");
    PRINTLNDEBUG( nOutput2)
  }*/

  if (abs(commandEl) > 0) {
    PRINTDEBUG("-->");
    PRINTDEBUG( Setpoint1);
    PRINTDEBUG(" , ");
    PRINTDEBUG( Input1)
    PRINTDEBUG(" , ");
    PRINTLNDEBUG( nOutput1)
  }  
  
  if (motor2.getState() == 0) readyToShoot = true;
  else readyToShoot = false;

  if (counts1 > EL_MAXLIM_ENC || counts1 < EL_MINLIM_ENC) {
    motor1.moveMotorStopEmergency();
  }

  if (stringComplete) {
    //Serial.println(inputString);
    // clear the string:
    commandAz = (((byte)inputString[0]) - 128) * 20;//40;
    commandEl = -(((byte)inputString[1]) - 128) * 5;//40;
    //Serial.println(commandTest);
    inputString = "";
    stringComplete = false;
  }

  //commandEl = -200;
  
/*
  if (millis() - commandTestUpdate > 1000) {
    commandTestUpdate = millis();
    PRINTLNDEBUG(counts1);
    //commandTest = commandTest + 1120*2;
    if (commandTest == 0) commandTest = 1120/2;
    else commandTest = 0;
    //motorC.setCommand(commandTest); //<===========================
    motorA.setCommand(commandTest);
    //stepMotor.moveStepMotor(50,true,1);
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
  if (digitalRead(encoder2PinA) == digitalRead(encoder2PinB) ) counts2--;
  else counts2++;
}

void shootCommand() //this function is triggered by the encoder falling, and increments the encoder counter
{
   shootFlag = true;
}

void shoot() {
  digitalWrite(airPressurePin,HIGH);
  delay(100);
  digitalWrite(airPressurePin,LOW);
  PRINTLNDEBUG("SHOOT2!")
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
