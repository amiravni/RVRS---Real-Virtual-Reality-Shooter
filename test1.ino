
int stepDirectionVer = 8;
int stepTriggerVer = 9;
int stepDirectionHor = 10;
int stepTriggerHor = 11;
int dir;
char stepsVer;
char stepsHor;
int x = 0;


void stepVer(){
  Serial.println ("stepping ver");
    digitalWrite(stepTriggerVer, HIGH);   
    delayMicroseconds(700);     
    digitalWrite(stepTriggerVer, LOW);   
    delayMicroseconds(700);  
}

void stepHor(){
  Serial.println ("stepping hor");
    digitalWrite(stepTriggerHor, HIGH);   
    delayMicroseconds(700);     
    digitalWrite(stepTriggerHor, LOW);   
    delayMicroseconds(700);  
}

void setup() {
  // pins for steper
  pinMode(stepDirectionHor, OUTPUT);
  pinMode(stepTriggerHor, OUTPUT);
  pinMode(stepDirectionVer, OUTPUT);
  pinMode(stepTriggerVer, OUTPUT);
    
  // initialize serial communication:
  Serial.begin(115200);
}


void serialEvent() {
    stepsVer = Serial.read();
    stepsHor = Serial.read();
}
           
           
    // the loop function runs over and over again forever
    void loop() {

      Serial.print(stepsVer + 42);
      Serial.print("-");
      Serial.println(stepsHor + 42);
      
            if (stepsVer<0)
            {
                  digitalWrite(stepDirectionVer, HIGH);
                  stepsVer = stepsVer * -1;
            }else {
                  digitalWrite(stepDirectionVer, LOW);
            }
            
            if (stepsHor<0)
              {
               digitalWrite(stepDirectionHor, HIGH);
               stepsHor = stepsHor * -1;
               }else {
                digitalWrite(stepDirectionHor, LOW);
              }
             
             while(stepsHor > 0 || stepsVer > 0){
                 if(stepsHor > 0){
                   stepsHor--;
                   stepHor();
                 }
                 
                 if(stepsVer > 0){
                   stepsVer--;
                   stepVer();
                 }
             }
             /* 
            for  (x=0; x<stepsVer ; x++)
            {
               Serial.println ("ver");
                stepVer();
            }
            
            stepsVer = 0;

              
            
              for  (x=0; x<stepsHor ; x++)
              {
                  stepHor();
              }
               
             stepsHor = 0;*/
}

