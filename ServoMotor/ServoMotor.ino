/*Nathan Hoang and James Bell
 * Program designed to take commands from transceiver and move Servo motors
 At 0 degrees , black wheel is on the right
 */

#include <Servo.h>
#include <SPI.h>
#include "RF24.h"

//initialize radio
RF24 radio(7,8);
byte addresses[][6] = {"1Node","2Node"};

//declare Servo 
Servo servoA; //3
Servo servoB; //5
Servo servoC; //9
Servo servoD; //11

//initialize Servo pins
int servoPinA = 3;
int servoPinB = 5;
int servoPinC = 6;
int servoPinD = 10;

//create array to store incoming doubles
float container[4];
//array to store speeds of motors
float writeSpeed[4];  
//current values of motors
float vx, vy, theta, thetaROC;
float wheelRad=.025;//2.5 cm
float bodyRad=.065;//6.5 cm, related to body radius
float k = 75;//scaling constant



//pids
p_v=75;
p_w=346.15;

  
void setup(){
  //stop movement
  stopMove();
  //begin Serial
  Serial.begin(115200);
  Serial.println(F("Servo (Receiver)"));
  radio.begin();
  //set up transceiver
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);
  radio.startListening();
  
}

void loop(){
    
    //create msg for transceiver to write to
    float msg;
    //check for incoming message
    if( radio.available()){
      //begin loop to recieve four numbers                                                              
                                       
        Serial.println("Recieved: ");
        for(int i = 0; i<4; i++){
          //read message, store in container array
          radio.read( &msg, sizeof(float) );
          container[i] = msg;
          //print for debug
          Serial.println(container[i]);
          //allows buffer to fill
          delay(3);
          
        }
        
      
        //transfer from container to variables
        vx = container[0];
        vy = container[1];
        theta = container[2];
        thetaROC = container[3];
        
        //debug
        /*
        vx = 1;
        vy = 1;
        theta = 0;
        thetaROC = 0;
        */
        
        //change wheel velocities based on new input values
        updateWheels();
        
        //reset the message and respond to confirm arrival of message
        radio.stopListening();                                        
        
        for(int i = 0; i<4; i++){
          msg = writeSpeed[i];
          radio.write( &msg, sizeof(float) );
        }
       
        msg = 0;
        //reset to listening immediately after sending
        radio.startListening();                                           
       
  }
  
}
void updateWheels(){
    attachAll();
    //Vx, Vy, orientation, orientationROC, radius
    
    writeSpeed[0] = -(p_v*(-sin(theta) * vx + cos(theta)*vy) + p_w*bodyRadius*thetaROC);
    writeSpeed[1] = ((p_v*(cos(theta) * vx + sin(theta)*vy) - p_w*bodyRadius*thetaROC));
    writeSpeed[2] = (p_v*(-sin(theta) * vx + cos(theta)*vy) - p_w*bodyRadius*thetaROC);
    writeSpeed[3] = -(p_v*(cos(theta) * vx + sin(theta)*vy) + p_w*bodyRadius*thetaROC);
    
    for(int c = 0; c<4; c++){
       if(writeSpeed[c]<9&&writeSpeed[c]>1){//What is the purpose of this? Is 10 the minimum?
         writeSpeed[c] = 10;
       }
    
    }

    servoA.write(writeSpeed[0]+90);//90 is pretty much the zero offset. Arduino doesn't quite get that servos can have 90/180/360/continuous rotation
    Serial.print("Servo A Speed: ");
    Serial.println(writeSpeed[0]);

    servoB.write(writeSpeed[1]+90);
    Serial.print("Servo B Speed: ");
    Serial.println(writeSpeed[1]);

    servoC.write(writeSpeed[2]+90);
    Serial.print("Servo C Speed: ");
    Serial.println(writeSpeed[2]);
 
    servoD.write(writeSpeed[3]+90);
    Serial.print("Servo D Speed: ");
    Serial.println(writeSpeed[3]);
    }
}

void attachAll(){
  servoA.attach(servoPinA);
  servoC.attach(servoPinC);
  servoB.attach(servoPinB);
  servoD.attach(servoPinD);
 }

void stopMove(){
  servoA.detach();
  servoB.detach();
  servoC.detach();
  servoD.detach();
}


void runFor(int time){
  delay(time*1000);
  stopMove();
}









