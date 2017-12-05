class motor {
  public:
    motor(int mA, int mB, int eA, int eB, int mPWM, int mNum) {
      motorPinA = mA;
      motorPinB = mB;
      motorEncA = eA;
      motorEncB = eB;
      motorPWMPin = mPWM;
      motorNumber = mNum;      
      pinMode(motorPinA, OUTPUT); //initialize Encoder Pins
      pinMode(motorPinB, OUTPUT);
      pinMode(motorEncA, INPUT); //initialize Motors Pins
      pinMode(motorEncB, INPUT);
      pinMode(motorPWMPin, OUTPUT);
      digitalWrite(motorPinA, LOW); //initialize Pin States
      digitalWrite(motorPinB, LOW);
      digitalWrite(motorEncA, LOW); //initialize Pin States
      digitalWrite(motorEncB, LOW);
      motorPWMPinVal = 255;
      motorDirection = true;
      analogWrite(motorPWMPin, motorPWMPinVal);
      if (motorNumber == 0) {
        encoder0PinA = motorEncA;
        encoder0PinB = motorEncB;
        PRINTLNDEBUG("-----0-----");        

      }
      else if (motorNumber == 1) {
        encoder1PinA = motorEncA;
        encoder1PinB = motorEncB;
        PRINTLNDEBUG("-----1-----");        
      }
      else if (motorNumber == 2) {
        encoder2PinA = motorEncA;
        encoder2PinB = motorEncB;
        PRINTLNDEBUG("-----2-----");
      }      
      currentEncoderValue = 0;
    }

    motor(int _stepPin, int _dirPin) {
      motorPinA = _stepPin;
      motorPinB = _dirPin;
      pinMode(motorPinA, OUTPUT); //initialize Encoder Pins
      pinMode(motorPinB, OUTPUT);   
      digitalWrite(motorPinA, LOW); //initialize Pin States
      digitalWrite(motorPinB, LOW);
    }

    void moveStepMotor(int steps, boolean dir, int dly) {
      digitalWrite(motorPinB, dir);
      for (int iii = 0 ; iii < steps ; iii++) {
        digitalWrite(motorPinA, HIGH);
        delay(dly);
        digitalWrite(motorPinA, LOW);
        delay(dly);
      }
    }

    void setCommand(long numOfSteps) {
      lastStepsCommand = numOfSteps;
    }

    void moveMotorCCW(int PWMVal) {
      analogWrite(motorPWMPin, PWMVal);
      digitalWrite(motorPinA, HIGH);
      digitalWrite(motorPinB, LOW);
     // PRINTLNDEBUG(" CCW ");
    }

    void moveMotorCW(int PWMVal) {
      analogWrite(motorPWMPin, PWMVal);
      digitalWrite(motorPinA, LOW);
      digitalWrite(motorPinB, HIGH);
     // PRINTLNDEBUG(" CW ");
    }

    void moveMotorStop() {
      digitalWrite(motorPinA, LOW);
      digitalWrite(motorPinB, LOW);
     // PRINTLNDEBUG(" ST ");
    }

    void moveMotorStopEmergency() {
      digitalWrite(motorPinA, LOW);
      digitalWrite(motorPinB, LOW);
      //PRINTLNDEBUG(" EMG_ST ");      
      while(1);
    }    

    void moveMotorPID() {
      if (motorDirection) moveMotorCW(motorPWMPinVal);
      else moveMotorCCW(motorPWMPinVal);
    }

    void moveMotor(double PIDOutput) {
      motorPWMPinVal = abs((int)PIDOutput);
      motorDirection = (PIDOutput > 0);
      if (motorNumber == 0) currentEncoderValue = counts0;
      if (motorNumber == 1) currentEncoderValue = counts1;
      if (motorNumber == 2) currentEncoderValue = counts2;
      analogWrite(motorPWMPin, motorPWMPinVal);
     /* PRINTDEBUG(currentEncoderValue);
      PRINTDEBUG(" , ");
      PRINTDEBUG(currentEncoderValue - counts1Last);
      PRINTDEBUG(" , ");      
      PRINTDEBUG(lastStepsCommand);
      PRINTDEBUG(" , ");
      PRINTDEBUG(PIDOutput);
      PRINTDEBUG(" , ");      
      PRINTDEBUG(motorPWMPinVal);
      PRINTDEBUG(" , ");  
      PRINTDEBUG(motorNumber);
      PRINTDEBUG(" , ");  */
      counts1Last = currentEncoderValue;       
      if (motorPWMPinVal < 30) moveMotorStop();
      else       moveMotorPID();
      /*if (lastStepsCommand - currentEncoderValue < -10) moveMotorCCW(motorPWMPinVal, true);
      else if (lastStepsCommand - currentEncoderValue > 10) moveMotorCW(motorPWMPinVal, true);
      else  moveMotorStop();*/
      
    }

  int getState() {
   /* if (digitalRead(motorPinA) || digitalRead(motorPinB)) {
    PRINTDEBUG("===1>");
    PRINTLNDEBUG(digitalRead(motorPinA));
    PRINTDEBUG("===2>");
    PRINTLNDEBUG(digitalRead(motorPinB));    
    }*/
    if (digitalRead(motorPinA) == LOW && digitalRead(motorPinB) == LOW) return 0;
    else if (digitalRead(motorPinA) == HIGH && digitalRead(motorPinB) == LOW) return 1;
    else return -1;
  }

  private:
    int motorPinA;
    int motorPinB;
    int motorEncA;
    int motorEncB;
    long currentEncoderValue;
    long lastStepsCommand;
    int motorPWMPin;
    int motorPWMPinVal;
    boolean motorDirection;
    int lastCommand = 0;
    int motorNumber = -1;
};

