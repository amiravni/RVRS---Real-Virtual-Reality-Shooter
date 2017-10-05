
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
      analogWrite(motorPWMPin, motorPWMPinVal);
      if (motorEncA == INT0) {
        encoder0PinA = motorEncA;
        encoder0PinB = motorEncB;

      }
      else if (motorEncA == INT1) {
        encoder1PinA = motorEncA;
        encoder1PinB = motorEncB;
      }
      currentEncoderValue = 0;
    }

    void setCommand(long numOfSteps) {
      lastStepsCommand = numOfSteps;
    }
    
    void moveMotorCCW(int PWMVal, boolean moving) {
      analogWrite(motorPWMPin, PWMVal);
      digitalWrite(motorPinA, HIGH);
      digitalWrite(motorPinB, LOW);
      PRINTLNDEBUG(" CCW ");
      if (moving) lastCommand = -1;
    }

    void moveMotorCW(int PWMVal, boolean moving) {
      analogWrite(motorPWMPin, PWMVal);
      digitalWrite(motorPinA, LOW);
      digitalWrite(motorPinB, HIGH);
      PRINTLNDEBUG(" CW ");
      if (moving) lastCommand = 1;
    }

    void moveMotorStop() {
      digitalWrite(motorPinA, LOW);
      digitalWrite(motorPinB, LOW);
      PRINTLNDEBUG(" ST ");
    }    

    void moveMotor() {
      if (motorEncA == INT0) currentEncoderValue = counts0;
      if (motorEncA == INT1) currentEncoderValue = counts1;
      //int motorPWMPinVal = max(128,min(255,int(abs(lastStepsCommand - currentEncoderValue) / 2 )));
      //analogWrite(motorPWMPin, motorPWMPinVal);
      PRINTDEBUG(currentEncoderValue);
      PRINTDEBUG(" , ");
      PRINTDEBUG(lastStepsCommand);
      PRINTDEBUG(" , ");
      motorPWMPinVal = 255;
      if (lastStepsCommand - currentEncoderValue < -100) moveMotorCCW(motorPWMPinVal,true);
      else if (lastStepsCommand - currentEncoderValue > 100) moveMotorCW(motorPWMPinVal,true);
      else { /*
        motorPWMPinVal = 170;
        if (lastCommand == -1) moveMotorCW(motorPWMPinVal,false);
        else if (lastCommand == 1) moveMotorCCW(motorPWMPinVal,false);
        if (abs(lastStepsCommand - currentEncoderValue) < 20)  moveMotorStop();*/
        moveMotorStop();
      }
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
    int lastCommand = 0;
};

