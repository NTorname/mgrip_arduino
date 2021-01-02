<h1>mGrip Arduino Controller</h1>

Using an arduino and a custom driver board, ROS is able to send messages to a custom controller for a Soft Robotics gripper.


<h2>Message Explained</h2>

Send GRIP variable to the Arduino to operate gripper.

   0 = CLOSED

   1 = NEUTRAL

   2 = OPEN

<h2>Arduino Error Blink Codes</h2>

Displays on built in LED on arduino

   1 = message pressure limits outside of acceptable range
   
   2 = grip request out of range
   
   3 = gripper pressure too high

<h2>Installing</h2>

gripper_control.ino must be installed on the arduino

pneu_gripper.h must be installed in `[arduino location]/libraries/ros_lib`
