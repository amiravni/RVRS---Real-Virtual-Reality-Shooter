#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>


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

#define INT0 2  // <-- change this when not using Arduino Uno
#define INT1 3  // <-- change this when not using Arduino Uno

//Initialize Variables
int encoder0PinA = -1;
int encoder0PinB = -1;
int encoder1PinA = -1;
int encoder1PinB = -1;
long counts0 = 0; //counts the encoder counts.
long counts1 = 0; //counts the encoder counts.
long commandTest = 0;
long commandTestUpdate = millis();

#include "classes.h"

motor motorA(Motor0PinA, Motor0PinB, Motor0EncoderPinA, Motor0EncoderPinB , Motor0PWM);
motor motorB(Motor1PinA, Motor1PinB, Motor1EncoderPinA, Motor1EncoderPinB , Motor1PWM);

void setup() {
  if (DEBUG) Serial.begin(115200);
  attachInterrupt(0, readEncoder0, CHANGE); //attach interrupt to PIN 2
  attachInterrupt(1, readEncoder1, CHANGE); //attach interrupt to PIN 3
}

void loop() {
  motorA.moveMotor();
  motorB.moveMotor();
  if (millis() - commandTestUpdate > 1000) {
    commandTestUpdate = millis();
    PRINTLNDEBUG(counts0);
    commandTest = commandTest + 1120;
    motorA.setCommand(commandTest);
    motorB.setCommand(commandTest);

  }

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


