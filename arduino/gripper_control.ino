/*
 Gripper Controller
 Code by Noah
 arudino subscribes to /pneu_gripper and safely sets gripper 
   to state sent in message
 
 Note: the control board is asserted 0v
 
 GRIP  = 0 CLOSED
       = 1 NEUTRAL
       = 2 OPEN
       
 error blink codes
    1 = message pressure limits outside of acceptable range
    2 = grip request out of range
    3 = gripper pressure too high
*/

#include <ros.h>
#include <mgrip_controller/pneu_gripper.h>


//user config stuff happens in this block
const int pin1 = A0;  //close gripper solenoid
const int pin2 = A1;  //open gripper solenoid
const int pin3 = A2;  //turns air pump on
const int gripPress = A3;  //read gripper pressure output
const int vRef = A4;  //connect to 3.3v source on arduino
const int maxP = 100;  //maximum allowable gripper pressure in 1/10ths of PSI (recommend 100)
const int minP = -50;  //minimum allowable gripper pressure in 1/10ths of PSI (recommend -50)


int prevState = -1;


void updateGripper(const mgrip_controller::pneu_gripper& gripper_msg){  //callback for when new gripper message received
  if((gripper_msg.minPres < minP) || (gripper_msg.maxPres > maxP)){
    isError(1);
  }
  
  if(gripper_msg.grip == prevState){  //TODO is this assuming too much? should rely on pressure reading?
    return;
  }
  prevState = gripper_msg.grip;
  
  if(gripper_msg.grip == 0){
    relaxGrip();
    openGrip(gripper_msg.minPres);
  }else if(gripper_msg.grip == 1){
    relaxGrip();
  }else if(gripper_msg.grip == 2){
    relaxGrip();
    closeGrip(gripper_msg.maxPres);
  }else{
    isError(2);
  } 
}


void isError(int code){  //error reporting I guess
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
  digitalWrite(pin3, HIGH);
  
  while(1){
    for(byte i = 0; i < code; i++){
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
    delay(1500);
  }
}


int getPSI(){ //returns pressure in gripper in 1/10ths of PSI
  float comp = 3.3 / analogRead(vRef);
  return (int)((7.288016*analogRead(gripPress)*comp - 21.65923)*10);
  //PSI = a(voltage) + b
  //a = 7.288016
  //b = -21.65923
}


//TODO merge open and close grip into one assertGrip function
void openGrip(int p){  //open gripper
  while(getPSI() > p){
    digitalWrite(pin1, LOW);
    digitalWrite(pin3, LOW);
  }
  digitalWrite(pin1, HIGH);
  digitalWrite(pin3, HIGH);
}


void closeGrip(int p){  //close gripper
  while(getPSI() < p){
    digitalWrite(pin2, LOW);
    digitalWrite(pin3, LOW);
  }
  digitalWrite(pin2, HIGH);
  digitalWrite(pin3, HIGH);
}


void relaxGrip(){  //return to neutral grip
  while(abs(getPSI()) > 2){  //TODO test if we can lower this pressure margin
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    digitalWrite(pin3, HIGH);
  }
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, HIGH);
}

 
ros::NodeHandle nh;
ros::Subscriber<mgrip_controller::pneu_gripper> sub("/pneu_gripper", &updateGripper); //set up ros and listen to /pneu_gripper topic


void setup(){  //set up all sorts of arduino things
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
  pinMode(vRef, INPUT);
  pinMode(gripPress, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
 
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, HIGH);
  digitalWrite(pin3, HIGH);
  
  nh.initNode();
  nh.subscribe(sub);
  
  digitalWrite(LED_BUILTIN, LOW);
}


void loop(){  //keep ros alive
  nh.spinOnce();
  
  if((getPSI() > (maxP + 5)) || (getPSI() < (minP - 5))){  //safety against any strange over pressure event
    isError(3);
  }
  
  delay(100);
}
