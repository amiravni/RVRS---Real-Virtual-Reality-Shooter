class motor {
  public:
    motor(int mA, int mB, int eA, int eB, int mPWM) {
      motorPinA = mA;
      motorPinB = mB;
      motorEncA = eA;
      motorEncB = eB;
      motorPWMPin = mPWM;
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
      if (motorEncA == INT0PIN) {
        encoder0PinA = motorEncA;
        encoder0PinB = motorEncB;

      }
      else if (motorEncA == INT1PIN) {
        encoder1PinA = motorEncA;
        encoder1PinB = motorEncB;
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
      PRINTLNDEBUG(" CCW ");
    }

    void moveMotorCW(int PWMVal) {
      analogWrite(motorPWMPin, PWMVal);
      digitalWrite(motorPinA, LOW);
      digitalWrite(motorPinB, HIGH);
      PRINTLNDEBUG(" CW ");
    }

    void moveMotorStop() {
      digitalWrite(motorPinA, LOW);
      digitalWrite(motorPinB, LOW);
      PRINTLNDEBUG(" ST ");
    }

    void moveMotorPID() {
      if (motorDirection) moveMotorCW(motorPWMPinVal);
      else moveMotorCCW(motorPWMPinVal);
    }

    void moveMotor(double PIDOutput) {
      motorPWMPinVal = abs((int)PIDOutput);
      motorDirection = (PIDOutput > 0);
      if (motorEncA == INT0PIN) currentEncoderValue = counts0;
      if (motorEncA == INT1PIN) currentEncoderValue = counts1;
      analogWrite(motorPWMPin, motorPWMPinVal);
      PRINTDEBUG(currentEncoderValue);
      PRINTDEBUG(" , ");
      PRINTDEBUG(currentEncoderValue - counts1Last);
      PRINTDEBUG(" , ");      
      PRINTDEBUG(lastStepsCommand);
      PRINTDEBUG(" , ");
      PRINTDEBUG(PIDOutput);
      PRINTDEBUG(" , ");      
      PRINTDEBUG(motorPWMPinVal);
      PRINTDEBUG(" , ");  
      counts1Last = currentEncoderValue;       
      moveMotorPID();
      /*if (lastStepsCommand - currentEncoderValue < -10) moveMotorCCW(motorPWMPinVal, true);
      else if (lastStepsCommand - currentEncoderValue > 10) moveMotorCW(motorPWMPinVal, true);
      else  moveMotorStop();*/
      
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
};

