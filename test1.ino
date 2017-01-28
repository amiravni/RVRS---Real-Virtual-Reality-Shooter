
int stepDirection = 8;
int stepTrigger = 9;
int dir;
char steps;
int x=0;


void step(){
    digitalWrite(stepTrigger, HIGH);   
    delayMicroseconds(700);     
    digitalWrite(stepTrigger, LOW);   
    delayMicroseconds(700);  
}


void setup() {
  // pins for steper
  pinMode(stepDirection, OUTPUT);
  pinMode(stepTrigger, OUTPUT);
    
  // initialize serial communication:
  Serial.begin(9600);
}

           
           
    // the loop function runs over and over again forever
    void loop() {

      
        if (Serial.ready){
            steps = Serial.read();
            if (steps<0)
            {
                digitalWrite(stepDirection, HIGH);
                steps = steps * -1;
            }else {
                digitalWrite(stepDirection, LOW);
            }
            
            for  (x=0; x<steps ; x++)
            {
                step();
            }
            
            
        }
        
}









