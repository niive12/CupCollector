
#include <R2WD_ROB5.h>
#include <MotorWheel.h> 
#include <PID_Beta6.h>
#include <PinChangeInt.h> 
#include <PinChangeIntConfig.h>

#define DIAMETER_MM 143
#define CIRCUMFERENCE_MM  (DIAMETER_MM*PI)
#define ENCODER_SAMPLE_TIME 100
#define STRAIGHT_DISTANCE 1000
#define RADIAN_ROTATE PI/2
#define NUMBER_OF_SQUARES 10
#define SPEEDMMPS 250
#define KEY_S7 3


irqISR(irq1,isr1); // Intterrupt function.on the basis of the pulse, work for wheel1
MotorWheel wheel1(9,8,4,5,&irq1,REDUCTION_RATIO,int(CIRCUMFERENCE_MM));
irqISR(irq2,isr2);
MotorWheel wheel2(10,11,6,7,&irq2,REDUCTION_RATIO,int(CIRCUMFERENCE_MM));
R2WD_ROB5 robot(&wheel1,&wheel2,WHEELSPAN);


long testvar;
unsigned long timestamp = 0;
char state = 'A';
int count = 0;
boolean serial_info = true;
boolean GO = false;
boolean wait = true;
int incomingByte = 0;

void setup() {
  TCCR1B=TCCR1B&0xf8|0x01;
  pinMode(KEY_S7, INPUT);
  
  robot.PIDEnable(0.26,0.02,0,10); // Enable PID
  Serial.begin(9600);
    while (!Serial) {;}
}



void loop() {
//Sorry, i needed some test code :) 
  /*Serial.print("THIS IS A LOOOOONG TEST STRING\n");
while(wait);
*/

  
  if(!GO){
    if(!digitalRead(KEY_S7)){
      GO = true;
      Serial.print("UMB_START\n");
      delay(300); // To allow laserrange to get cracker-lacking
    }
  }
  
  
  if(millis() % ENCODER_SAMPLE_TIME == 0 && serial_info && GO){
    testvar = wheel1.getCurrPulse();
    Serial.print(millis());
    Serial.print(",");
    Serial.print(CIRCUMFERENCE_MM * testvar/(CPR*REDUCTION_RATIO));  //circumference * ticks/(Counts_per_resolution * redtuction_ratio)
    Serial.print(",");
  
    testvar = wheel2.getCurrPulse();
    Serial.print(-CIRCUMFERENCE_MM * testvar/(CPR*REDUCTION_RATIO));  //circumference * ticks/(Counts_per_resolution * redtuction_ratio)
    Serial.print("\n"); 
    
    wheel1.resetCurrPulse();
    wheel2.resetCurrPulse();
  }

  if(GO){
    switch(state){
      case 'A' :      //Take timestamp and make the motors go straight
        timestamp = millis();
        robot.setCarAdvance(SPEEDMMPS);
        state = 'B';
        break;
      
      case 'B' :    //Wait a bit
        if(millis() > timestamp + robot.getStraightTime(SPEEDMMPS, STRAIGHT_DISTANCE)){
         state = 'C'; 
        }
        break;
    
      case 'C' :    //Stop the motors, take timestamp and start turning
        robot.setCarStop();
        timestamp = millis();
        robot.setCarRotateLeft(SPEEDMMPS);
        state = 'D';
        break;
      
      case 'D' :    // Wait a bit
        if(millis() > timestamp + robot.getRotationTime(SPEEDMMPS, RADIAN_ROTATE)){
           count++;
           if(count == NUMBER_OF_SQUARES*4){
             Serial.print("UMB_STOP\n"); 
             state = 'Ø';
           }else{
             state = 'A';
           }
        }
        break;
      
    
      case 'Ø' :
        robot.setCarStop();
        serial_info = 0;
        break;
      
      
      default :
        //DO
        break;
    }
  }
  robot.RUN_regulate();        //IMPORTANT! Runs the PIDRegulate every SAMPLE_TIME
  
}

