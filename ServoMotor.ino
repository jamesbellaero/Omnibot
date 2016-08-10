/*Nathan Hoang
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
float vx, vy, theta, thetaROC, radius=.3;
float k = 75;

  
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
        updateWheel();
        
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

//move forward with time constraint
void moveForward(double speed, int time){
  attachBD();
  servoB.write(90+speed);
  servoD.write(90-speed);
  Serial.print("Vertical Speed (Forward): ");
  Serial.println(speed);
  Serial.println();
  delay(time*1000);
  stopMove();
}
//no time
void moveForward(double speed){
  attachBD();
  servoB.write(90+speed);
  servoD.write(90-speed);
  Serial.print("Vertical Speed (Forward): ");
  Serial.println(speed);
  Serial.println();
}
//timed move backward
void moveBackward(double speed, int time){
  attachBD();
  servoB.write(90-speed);
  servoD.write(90+speed); 
  Serial.print("Vertical Speed (Backward): ");
  Serial.println(speed);
  Serial.println();
  delay(time*1000);
  stopMove();
}
//untimed
void moveBackward(double speed){
  attachBD();
  servoB.write(90-speed);
  servoD.write(90+speed); 
  Serial.print("Vertical Speed (Backward): ");
  Serial.println(speed);
  Serial.println();
}
//timed move right
void moveRight(double speed, int time){
  attachAC();
  servoA.write(90+speed);
  servoC.write(90-speed);
  Serial.print("Horizontal Speed (Right): ");
  Serial.println(speed);
  Serial.println();
  delay(time*1000);
  stopMove();
}
//untimed
void moveRight(double speed){
  attachAC();
  servoA.write(90+speed);
  servoC.write(90-speed);
  Serial.print("Horizontal Speed (Right): ");
  Serial.println(speed);
  Serial.println();
}
//timed move left
void moveLeft(double speed, int time){
  attachAC();
  servoA.write(90-speed);
  servoC.write(90+speed);
  Serial.print("Horizontal Speed (Left): ");
  Serial.println(speed);
  Serial.println();
  delay(time*1000);
  stopMove();
}
//untimed
void moveLeft(double speed){
  attachAC();
  servoA.write(90-speed);
  servoC.write(90+speed);
  Serial.print("Horizontal Speed (Left): ");
  Serial.println(speed);
  Serial.println();
}
//timed turn clockwise
void turnCW(int speed, double time){
  attachAll();
  servoA.write(90+speed);
  servoB.write(90+speed);
  servoC.write(90+speed);
  servoD.write(90+speed);
  delay(time); //240-270
  stopMove();
}
//timed turn counter clockwise
void turnCC(int speed, double time){
 attachAll();
 servoA.write(90-speed);
 servoB.write(90-speed);
 servoC.write(90-speed);
 servoD.write(90-speed);
 delay(time);
 stopMove();
}

void goDirection(int angle, int velocity, int time){
  double rad;
  double bd;
  double ac;
  Serial.print("Angle: ");
  Serial.println(angle);
  
  if(angle>0&&angle<90){ 
    rad = angle*3.14/180;
    bd = velocity*sin(rad);
    ac = velocity*cos(rad);
    moveForward(bd);
    moveRight(ac);
    
  }
  if(angle>90&&angle<180){    
    rad = angle*3.14/180;
    ac = -velocity*cos(rad);
    bd = velocity*sin(rad);
    moveForward(bd);
    moveLeft(ac);
    
  }
  if(angle>180&&angle<270){
    rad = angle*3.14/180;
    ac = -velocity*cos(rad);
    bd = -velocity*sin(rad);
    moveBackward(bd);
    moveLeft(ac);
      
  }
  if(angle>270&&angle<360){
    rad = angle*3.14/180;
    ac = velocity*cos(rad);
    bd = -velocity*sin(rad);
    moveBackward(bd);
    moveRight(ac);
    
  }
  if(angle==90){
    ac=00;
    bd=velocity;
    attachBD();
    moveForward(bd);
  }
  if(angle==0||angle==360){
    ac=velocity;
    bd=0;
    attachAC();
    moveRight(ac);
  }
  if(angle==270){
    ac=0;
    bd=velocity;
    attachBD();
    moveBackward(bd);
  }
  if(angle==180){
    ac=velocity;
    bd=0;
    attachAC();
    moveLeft(ac);
  }  
}

void stopMove(){
  servoA.detach();
  servoB.detach();
  servoC.detach();
  servoD.detach();
}

void attachAC(){
 servoA.attach(servoPinA);
 servoC.attach(servoPinC); 
 }

void attachBD(){
 servoB.attach(servoPinB);
 servoD.attach(servoPinD); 
 }

void attachAll(){
  servoA.attach(servoPinA);
  servoC.attach(servoPinC);
  servoB.attach(servoPinB);
  servoD.attach(servoPinD);
 }

void runFor(int time){
  delay(time*1000);
  stopMove();
}

void updateWheel(){
    attachAll();
    //Vx, Vy, orientation, orientationROC, radius
    
    writeSpeed[0] =  (-(-sin(theta)*vx+cos(theta)*vy +radius*thetaROC))*k;
    writeSpeed[1] =  ((cos(theta)*vx + sin(theta)*vy-radius*thetaROC))*k;
    writeSpeed[2] = (-sin(theta)*vx+cos(theta)*vy-radius*thetaROC)*k;
    writeSpeed[3] = (cos(theta)*vx + sin(theta)*vy+radius*thetaROC)*-k;
    /*
    float a = .1;
    float b = 0.0872665;
    if(vx<=a&&vx>=-a
        &&vy<=a&&vy>=-a
          &&thetaROC<=b&&thetaROC>=-b){
        k = 0;
    }
    else{
      
      float maxValue = writeSpeed[0];
      for(int q = 0; q<4; q++){
        if(writeSpeed[q]>maxValue) maxValue =writeSpeed[q];
      
      }
      k = 90/maxValue;
      
    }
    
    writeSpeed[0] =  (-(-sin(theta)*vx+cos(theta)*vy +radius*thetaROC))*k;
    writeSpeed[1] =  ((cos(theta)*vx + sin(theta)*vy-radius*thetaROC))*k;
    writeSpeed[2] = (-sin(theta)*vx+cos(theta)*vy-radius*thetaROC)*k;
    writeSpeed[3] = (cos(theta)*vx + sin(theta)*vy+radius*thetaROC)*-k;
    */
    
    for(int c = 0; c<4; c++){
       if(writeSpeed[c]<9&&writeSpeed[c]>1){
         writeSpeed[c] = 10;
       }
    
    }
    
    for(int b = 0; b<4; b++){
      if (b == 0){
        
        servoA.write(writeSpeed[0]+90);
        Serial.print("Servo A Speed: ");
        Serial.println(writeSpeed[0]);
      }
      else if (b == 1){
        
        servoB.write(writeSpeed[1]+90);
        Serial.print("Servo B Speed: ");
        Serial.println(writeSpeed[1]);
      }
      else if (b == 2){
        
        servoC.write(writeSpeed[2]+90);
        Serial.print("Servo C Speed: ");
        Serial.println(writeSpeed[2]);
      }
      else if (b == 3){
        
        servoD.write(writeSpeed[3]+90);
        Serial.print("Servo D Speed: ");
        Serial.println(writeSpeed[3]);
      }
      
    }
    //k = 1;
}







