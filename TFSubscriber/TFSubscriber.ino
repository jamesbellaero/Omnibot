//sudo chmod a+rw /dev/ttyUSB0
#include <ArduinoHardware.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <RF24_config.h>
#include <printf.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <SPI.h>
#include <Stream.h>
#include <quaternion.h>

ros::NodeHandle  nh;
float vx,vy,theta,thetaROC;

RF24 radio(9,10);
byte addresses[][6] = {"1Node","2Node"};
unsigned long currentSend, lastSend =0;
float k = 1; 
Vec3 tarLoc;
Vec3 tarAtt;
Vec3 loc;
Vec3 att;

boolean doOnce = true;

void messageCallback( const geometry_msgs::TransformStamped& t){
    
  String a =  t.header.frame_id;
  //Set target
  //Vec4 x;
  if(a=="tar"){
    tarLoc[0] = t.transform.translation.x;
    tarLoc[1] = t.transform.translation.y;
    tarLoc[2] = t.transform.translation.z;
    x.v[0] = t.transform.rotation.w;
    x.v[1] = t.transform.rotation.x;
    x.v[2] = t.transform.rotation.y;
    x.v[3] = t.transform.rotation.z;
    tarAtt = Quat2RPY(x);
    logmsg(t.transform.translation);
  }
  else{
    loc[0] = t.transform.translation.x;
    loc[1] = t.transform.translation.y;
    loc[2] = t.transform.translation.z;
    x.v[0] = t.transform.rotation.w;
    x.v[1] = t.transform.rotation.x;
    x.v[2] = t.transform.rotation.y;
    x.v[3] = t.transform.rotation.z;
    att = Quat2RPY(x);
  }
  //edit formulas!
  vx = (float)(tarLoc[0] - loc[0]);
  
  vy = (float)(tarLoc[1] - loc[1]);
  
  theta = (float)att[2];//yaw
  
  thetaROC  = (float)(tarAtt[2]-att[2]);
        
       
  sendMessage();
  
}
/*
void goalCb(const geometry_msgs::Twist& tw){
       
     if(tw.angular.z == 1){
        tarX = tw.linear.x;
        tarY = tw.linear.y;
        tarTheta = tw.angular.x;
        logmsg(tarX);
     }

}
*/

ros::Subscriber<geometry_msgs::TransformStamped> sub("vicon/omnibot/omnibot_throttle", &messageCallback );
//ros::Subscriber<geometry_msgs::Twist> suba("target", &goalCb );
//done
void setup()
{
  //Initialize USB connection
  nh.getHardware()->setBaud(115200);
  //Initialize node loop
  nh.initNode();
  //Subcribe to geometry posting
  nh.subscribe(sub);
  
  //Initialize radio
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1,addresses[0]);
  radio.stopListening();
// nh.subscribe(suba);
}
void loop()
{  
  
  nh.spinOnce();
  delay(1);
  
}

void sendMessage(){
      
  radio.stopListening(); 
  
  radio.write( &vx, sizeof(float) );
  radio.write( &vy, sizeof(float) );
  radio.write( &theta, sizeof(float) );
  radio.write( &thetaROC, sizeof(float) );
  radio.startListening();                                   
  
  unsigned long started_waiting_at = micros();               
  boolean timeout = false;                                           
  while ( ! radio.available() ){                             
    if (micros() - started_waiting_at > 200000 ){          
        timeout = true;
        break;
    }      
  }
  if ( timeout ){                                             
    
  }
  else{
    if (radio.available()){
     
      for(int i = 0; i<4; i++){ 
        char log_msg [50]; 
        float got_time;                                 
        radio.read( &got_time, sizeof(float) );
        
        delay(3);
     }
     
    }
  }
}

void logmsg(float a){
  char log_msg[50];
  float temp = a;
  int temp1 = (temp - (int)temp) * 100;
  sprintf(log_msg, "Float = %0d.%d", (int)temp, temp1);
  nh.loginfo(log_msg);
              
}



