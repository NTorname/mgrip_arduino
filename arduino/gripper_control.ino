/*
   Co-Drive Hardware Driver
   Controls pneumatic system for Co-Drive
   ROS publisher on system should be /codrive_driver
   State init to 3, flip to 4 to engage +24V power
   Then use state 0-2 as desired to control gripper
*/

#include <ros.h>
#include <std_msgs/UInt8.h>
//States
#define GRIP_NEUTRAL 0
#define GRIP_CLOSE 1
#define GRIP_OPEN 2
#define FLIP_OFF 3
#define FLIP_ON 4
//Control Lines
//Close gripper solenoid
#define OUT_A B00000100
//Open gripper solenoid
#define OUT_B B00001000
//Turns air pump on 
#define OUT_C B00010000
//Vref
#define VREF A0
//Co-Drive PSI
#define PSI A1
//Presure Limits in 0.1 PSI
#define MAX 100
#define MIN -50
//Port B bitmask - Used for relay control
#define MASK B00011111

//====================================================================================================================================================================
byte state = 3; //default to relay off

//ROS Arduino reference: http://wiki.ros.org/rosserial_arduino/
ros::NodeHandle nh;
void msgCb (const std_msgs::UInt8& msg) {
  state = msg.data;
}

ros::Subscriber<std_msgs::UInt8> sub("codrive_driver", &msgCb);

byte getPSI () {
  float comp = 3.3 / analogRead(VREF);
  return (byte) (7.288016 * analogRead(PSI) * comp - 21.65923);
}

//TODO: NEED TO CHECK IF ALL THESE FUNCTIONS ARE LOGICAL FOR THE ACTUAL SYSTEM - ASK NOAH & SASHA
//======================================================================================================

void relaxGrip() {
  // digitalWrite(OUT_A, HIGH);
  // digitalWrite(OUT_B, HIGH);
  // digitalWrite(OUT_C, LOW);
  PORTD = OUT_A | OUT_B;
  delay(2000);
  // digitalWrite(OUT_A, LOW);
  // digitalWrite(OUT_B, LOW);
  PORTD = B00000000;
}

void closeGrip() {
  while (getPSI() < MAX) {
    /*int i = 0;
      while(i < 30) {  //debug purposes only
      i++;*/
    // digitalWrite(OUT_B, HIGH);
    // digitalWrite(OUT_C, HIGH);
    PORTD = OUT_B | OUT_C;
  }
  // digitalWrite(OUT_B, LOW);
  // digitalWrite(OUT_C, LOW);
  PORTD = B00000000;
}

void openGrip() {
  while (getPSI() > MIN) {
    /*int j = 0;
      while(j < 30){  //debug purposes only
      j++;*/
    // digitalWrite(OUT_A, HIGH);
    // digitalWrite(OUT_C, HIGH);
    PORTD = OUT_A | OUT_C;
  }
}

void relayOn() {
  // This code utilizes a bitmask to flip pins #-# at the same exact time to ensure that they are not creating any shorts.
  // This provides the neccassary 80 mA +/- 10% to properly drive the onboard relay.
  PORTB = PORTB | MASK;
}

void relayOff() {
  // This code utilizes a bitmask to flip pins #-# at the same exact time to ensure that they are not creating any shorts.
  // This provides the neccassary 80 mA +/- 10% to properly drive the onboard relay.
  PORTB = PORTB & (~MASK);
}

void setup() {
  //+24V Digital Line Control via FETs
  pinMode(OUT_A, OUTPUT);
  pinMode(OUT_B, OUTPUT);
  pinMode(OUT_C, OUTPUT);
  //Pin 5 High Impedance - This ensures the output bank will function properly
  pinMode(5, INPUT);
  //The reason for this was due to a phsyical error in the boards design related
  //To how much current digital pins are capable of sourcing.
  //Do not alter this unless the physical issue has been repaired or you are
  //Implementing an alternate fix

  //Output Bank Pins - Used to provide 100 mA to the relay.
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  //This requires pin 8-12 to be jumped to pin 5, and for
  //Pin 5 to be high impedence (ie: input). The other option
  //is to cut the pin 5 trace and attach to pin 5 of K1 relay
  //ENSURE THESE ARE ONLY USED VIA PORT MANIPULATION    !!!
  //IF THESE PINS AREN'T THE SAME YOU'RE SHORTING THEM  !!!
  //ROS Node Handle Initialization

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  //curr_psi = getPSI();
  switch (state) {
    case GRIP_NEUTRAL:
      relaxGrip();
      nh.logdebug("State: GRIP_NEUTRAL");
      break;
    case GRIP_CLOSE:
      relaxGrip();
      closeGrip();
      nh.logdebug("State: GRIP_CLOSE");
      break;
    case GRIP_OPEN:
      nh.logdebug("State: GRIP_OPEN_Start");
      relaxGrip();
      openGrip();
      nh.logdebug("State: GRIP_OPEN");
      break;
    case FLIP_OFF:
      relayOff();
      nh.logdebug("State: FLIP_OFF");
      break;
    case FLIP_ON:
      relayOn();
      nh.logdebug("State: FLIP_ON");
      break;
  }
  nh.spinOnce();
}
