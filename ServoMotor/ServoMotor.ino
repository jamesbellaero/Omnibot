/*Nathan Hoang and James Bell
 * Program designed to take commands from transceiver and move Servo motors
 At 0 degrees , black wheel is on the right
 */

#include <Servo.h>
#include <SPI.h>
#include <XBee.h>
#include <SoftwareSerial.h>
//initialize radio
XBee xbee = XBee();
SoftwareSerial serial = SoftwareSerial(10, 11);
Rx16Response rx16 = Rx16Response();

//declare Servo 
Servo servoA; //3
Servo servoB; //5
Servo servoC; //6
Servo servoD; //9

//initialize Servo pins
int servoPinA = 3;
int servoPinB = 5;
int servoPinC = 6;
int servoPinD = 9;

//create array to store incoming doubles
float container[4];
//array to store speeds of motors
float writeSpeed[4];  
//current values of motors
float dx, dy, theta, dTheta;
float wheelRad=.025;//2.5 cm
float bodyRad=.065;//6.5 cm, related to body radius

//pids
//float k_v[];
//float k_w[];

//errors
float ex[5];//dx, dx_sum, dx_rate, dt
float ey[5];
float eTheta[5];
long tLast;//time of last update
float eps=.05;//error at which it assumes it has reached the destination for integral gain
  
void setup(){
  //ideally, read these from file
//  k_v[0]=75;
//  k_v[1]=0;
//  k_v[2]=0;
//  k_w[0]=346.15;
//  k_w[1]=0;
//  k_w[2]=0;
  //stop movement
//  stopMove();
  //begin radio connection
  serial.begin(9600);
  serial.listen();
  xbee.setSerial(serial);
  Serial.begin(9600);
  //for pids
  tLast=millis();
}

void loop(){
//  xbee.readPacket();
//  if (xbee.getResponse().isAvailable()) {
//    if(xbee.getResponse().getApiId()==AT_COMMAND_REQUEST){
//      XBeeResponse res = xbee.getResponse();
//    }
//    Serial.write("\nsomething happened\n");
//    String toWrite=String(xbee.getResponse().getApiId());
//    Serial.print(toWrite);
//    //RxResponse rx = xbee.getResponse();
////    uint8_t* data=rx.getData();
////    for(uint8_t i=0;i<rx.getDataLength();i++){
////      Serial.write(rx.getData()[i]);
////    }
//  }
  if(serial.available()>0){
    uint8_t var= serial.read();
    Serial.write(var.read());
  }
  if(Serial.available()>0){
    uint8_t var= Serial.read();
    serial.write(var);
  }
  
    
//    //create msg for transceiver to write to
//    float msg;
//    //check for incoming message
//    if( ){
//      //begin loop to recieve four numbers                                                              
//                                       
//        if (xbee.getResponse().isAvailable()) {
//          if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
//            xbee.getResponse().getRx16Response(rx16);
//          }
//        }
//        for(int i = 0; i<4; i++){
//          //read message, store in container array
//          radio.read( &msg, sizeof(float) );
//          container[i] = msg;
//          //print for debug
//          Serial.println(container[i]);
//          //allows buffer to fill
//          delay(3);
//          
//        }
//        
//      
//        //transfer from container to variables
//        ex[0] = container[0];
//        ey[0] = container[1];
//        theta = container[2];
//        eTheta[0] = container[3];
//        
//        //debug
//        /*
//        dx = 1;
//        dy = 1;
//        theta = 0;
//        dTheta = 0;
//        */
//        
//        //change wheel velocities based on new input values
//        updateWheels();
//        
//        //reset the message and respond to confirm arrival of message
//        radio.stopListening();                                        
//        
//        for(int i = 0; i<4; i++){
//          msg = writeSpeed[i];
//          radio.write( &msg, sizeof(float) );
//        }
//       
//        msg = 0;
//        //reset to listening immediately after sending
//        radio.startListening();                                           
//       
//  }
  
}
//void updateWheels(){
//  attachAll();
//  //dx, dy, orientation, orientationROC, radius
//  
//  updatePids();
//  float angularSpeeds[4];
//  float linearSpeeds[4];
//  float vx=0;
//  float vy=0;
//  float vTheta=0;
//
//  for(i=0;i<3;i++){
//    vx=vx+k_v[i]*ex[i];
//    vy=vy+k_v[i]*ey[i];
//    vTheta=vTheta+k_w[i]*eTheta[i];
//  }
//  
//  linearSpeeds[0]=(sin(theta) * vx - cos(theta) * vy);
//  linearSpeeds[1]=(cos(theta) * vx + sin(theta) * vy);
//  linearSpeeds[2]=(-sin(theta) * vx + cos(theta) * vy);
//  linearSpeeds[3]=(-cos(theta) * vx - sin(theta) * vy);
//
//  angularSpeeds[0]=-(bodyRadius*vTheta);
//  angularSpeeds[0]=-(bodyRadius*vTheta);
//  angularSpeeds[0]=-(bodyRadius*vTheta);
//  angularSpeeds[0]=-(bodyRadius*vTheta);
//
//  for(i=0;i<4;i++){
//    writeSpeed[i]=linearSpeeds[i]+angularSpeeds[i];
//    if(writeSpeed[i]<9&&writeSpeed[i]>1){//What is the purpose of this? Is 10 the minimum?
//       writeSpeed[i] = 10;
//     }
//  }
//
//  servoA.write(writeSpeed[0]+90);//90 is pretty much the zero offset. Arduino doesn't quite get that servos can have 90/180/360/continuous rotation
//  Serial.print("Servo A Speed: ");
//  Serial.println(writeSpeed[0]);
//
//  servoB.write(writeSpeed[1]+90);
//  Serial.print("Servo B Speed: ");
//  Serial.println(writeSpeed[1]);
//
//  servoC.write(writeSpeed[2]+90);
//  Serial.print("Servo C Speed: ");
//  Serial.println(writeSpeed[2]);
//
//  servoD.write(writeSpeed[3]+90);
//  Serial.print("Servo D Speed: ");
//  Serial.println(writeSpeed[3]);
//}
//updatePids(){
//  //  double error = Setpoint - Input;
//  //  errSum += (error * timeChange);
//  //  double dErr = (error - lastErr) / timeChange;
//  //  Output = kp * error + ki * errSum + kd * dErr;
//  
//  long tCurr=millis();
//  float dt=(float)(tCurr-tLast);
//  tLast=tCurr;
//
//  if(ex[0]<eps){//Threshold for integral gain, seems reasonable to me. Could also just set i-term low I guess
//    ex[1]=0;
//    ex[3]=tCurr;//Time since last off target
//  }
//  ex[1]=ex[1]*dt;//set the integral term
//  ex[2]=(ex[0]-ex[4])/dt;//set the derivate term
//  ex[4]=ex[0];//set the previous error
//
//  if(ey[0]<eps){
//    ey[1]=0;
//    ey[3]=tCurr;
//  }
//  ey[1]=ey[1]*dt;
//  ey[2]=(ey[0]-ey[4])/dt;
//  ey[4]=ey[0];
//
//  if(eTheta[0]<eps){
//    eTheta[1]=0;
//    eTheta[3]=tCurr;
//  }
//  eTheta[1]=eTheta[1]*dt;
//  eTheta[2]=(eTheta[0]-eTheta[4])/dt;
//  eTheta[4]=eTheta[0];
//  
//
//}
//
//void attachAll(){
//  servoA.attach(servoPinA);
//  servoC.attach(servoPinC);
//  servoB.attach(servoPinB);
//  servoD.attach(servoPinD);
// }
//
//void stopMove(){
//  servoA.detach();
//  servoB.detach();
//  servoC.detach();
//  servoD.detach();
//}
//
//
//void runFor(int time){
//  delay(time*1000);
//  stopMove();
//}
//
//
//
//
//
//
//
//

