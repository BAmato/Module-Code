/*
 * Module 2 -- Move it!
 */ 

//TODO, Add your group information to the top of your code.

//Wheel diameter = 72mm = 7.2cm
//Track distance = 125mm = 12.5 cm

#include <Arduino.h>
#include <wpi-32u4-lib.h>

#include <IRdecoder.h>
#include <ir_codes.h>

#include <Chassis.h>
#include <screen.h>

// TODO, Section, 4.2: Add line to include Chassis.h


// Sets up the IR receiver/decoder object
const uint8_t IR_DETECTOR_PIN = 1;
IRDecoder decoder(IR_DETECTOR_PIN);
volatile int baseSpeed = 10;

volatile uint16_t leftLine;
volatile uint16_t rightLine;


Chassis chassis(7.2,1440,13.2);
// TODO, Section 4.2: Declare the chassis object (with default values)
// TODO, Section 6.2: Adjust parameters to better match actual motion

// A helper function for debugging
#define LED_PIN 13
void setLED(bool value)
{
  Serial.println("setLED()");
  digitalWrite(LED_PIN, value);
}

// Defines the robot states
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE_FOR, ROBOT_LINING};
ROBOT_STATE robotState = ROBOT_IDLE;

// idle() stops the motors
void idle(void)
{
  Serial.println("idle()");
  setLED(LOW);

  // TODO, Section 4.2: Uncomment call to chassis.idle() to stop the motors
  chassis.idle();

  //set state to idle
  robotState = ROBOT_IDLE;
  Serial.println("/idle()");
}

/*
 * This is the standard setup function that is called when the board is rebooted
 * It is used to initialize anything that needs to be done once.
 */
void setup() 
{

  Serial.println("/setup()");

  // This will initialize the Serial at a baud rate of 115200 for prints
  // Be sure to set your Serial Monitor appropriately
  Serial.begin(115200);


  // TODO, Section 4.2: Initialize the chassis (which also initializes the motors)
  chassis.init();

  // TODO, Section 5.1: Adjust the PID coefficients
  chassis.setMotorPIDcoeffs(7,2);

  // Initializes the IR decoder
  decoder.init();

  //Setup the Line Sensors
  pinMode(LEFT_LINE_SENSE, INPUT);  //A3
  pinMode(RIGHT_LINE_SENSE, INPUT); //A4

  leftLine = analogRead(LEFT_LINE_SENSE);
  rightLine = analogRead(RIGHT_LINE_SENSE);

  screen.setup();

  idle();

  
}

// A helper command to drive a set distance
// At the start, it will take no arguments and we'll hardcode a motion
// TODO, Section 6.1 (but not before!): Edit the function definition to accept a distance and speed
void drive(int distance, int speed)
{
  setLED(HIGH);
  robotState = ROBOT_DRIVE_FOR;

  // TODO: In Section 4.2 and 5.1, add a call to chassis.setWheelSpeeds() to set the wheel speeds
  
  //chassis.setWheelSpeeds(30, 30);
  
  // TODO: In Section 6.1, remove the call to setWheelSpeeds() and add a call to chassis.driveFor()
  chassis.driveFor(distance, speed);
  
}

// A helper function to turn a set angle
void turn(float ang, float speed)
{
  setLED(HIGH);
  robotState = ROBOT_DRIVE_FOR;
  
  // TODO, Section 6.1: Make a call to chassis.turnFor()

  chassis.turnFor(ang, speed);
}

// TODO, Section 6.1: Declare function handleMotionComplete(), which calls idle()
void handleMotionComplete(){
  idle();
}

//Function that handles Line Following
void beginLineFollowing(){
  //Setup
  Serial.println("beginLineFollowing()");
  setLED(HIGH);
  robotState = ROBOT_LINING;
}

//Function to handle the Line Following
void handleLineFollow(int speed){
  //Serial.println("handleLineFollow()");
  
  int whiteMin = 120;
  int whiteMax = 220;
  int blackMin = 500;   //Sensor min was 680 but depends on proximity to black tape so overshooting
  int blackMax = 900;   //Sensor max was ~850 but depends on proimity so overshooting

  // Read the sensor values inside the loop
  leftLine = analogRead(LEFT_LINE_SENSE);
  rightLine = analogRead(RIGHT_LINE_SENSE);
 
  //Define error between sensors
  int error = leftLine - rightLine;
  int turnEffort = error * 1.1; //Error * K_p
  
    //        LIGHT                   BLACK
    //INNER LEFT 60   INNER RIGHT 60cm     LEFT 420    RIGHT 500
    //error -30 to 200 black line
    //

    //FAR LEFT   FAR RIGHT    FAR LEFT   FAR RIGHT
    //      BLACK                    WHITE
    //115          120          850        870

    //FAR LEFT   FAR RIGHT    FAR LEFT   FAR RIGHT
    //      BLACK                    WHITE
    //806          850              102            99

  //If it is in the black zone, dont adjust turning or speed

  // if(error > -30 && error < 200){
  //   chassis.setTwist(speed, 0); //Keep straight
  // }
  // else if(error > 210){
  //   chassis.setTwist(speed, turnEffort -210); //Turn Right
  // }
  // else if(error < -35){  
  //   chassis.setTwist(speed, turnEffort + 35); //Turn Left
  // }
  // else {
  //     //        (forwardSpeed, turningSpeed)
  // chassis.setTwist(speed, turnEffort);
  // }

  // Serial.print("Left Line: ");
  // Serial.print(leftLine);
  // Serial.print("\t");
  // Serial.print("Right Line: ");
  // Serial.print(rightLine);
  // Serial.print("\t");
  // Serial.print("\t");
  // Serial.print("Error: ");
  // Serial.print(error);
  // Serial.print("\t");
  // Serial.print("turnEffort: ");
  // Serial.println(turnEffort);

  // //Calculate effort based on error
  // // int16_t effort;
  // // if(error > 0){
  // //   //Turn Left???
  // //   effort = -error;
  // // } else {
  // //   //Turn Right???
  // //   effort = error;
  // // }
    
  Serial.print("LEFT S:\t");
  //Serial.print(analogRead(LEFT_LINE_SENSE));
  Serial.print(leftLine);
  Serial.print("\tRIGHT S:\t");
  Serial.print(rightLine);
  //Serial.println(analogRead(RIGHT_LINE_SENSE));
  
  while (rightLine > whiteMin && rightLine < whiteMax && (leftLine > whiteMin && leftLine < whiteMax)) {
    // Straight movement when both sensors are on white
    chassis.setWheelSpeeds(30, 30);

    if (leftLine > blackMin && leftLine < blackMax) {
        // Adjust speed and turn left when the left sensor detects black
        chassis.setTwist(speed, 10);   // Turn Left
        // chassis.turnFor(-30, 40);
    } else if (rightLine > whiteMin && rightLine < whiteMax) {
        // Adjust speed and turn right when the right sensor detects white
        chassis.setTwist(speed, -10);   // Turn Right
    }

    // Update sensor readings
    leftLine = analogRead(LEFT_LINE_SENSE);
    rightLine = analogRead(RIGHT_LINE_SENSE);
  }

  // If either sensor is on black, adjust the direction accordingly
  if (leftLine < whiteMin || leftLine > whiteMax) {
      chassis.setTwist(speed, 10); // Turn Left
      // chassis.turnFor(30, 200);
  } else if (rightLine < whiteMin || rightLine >= whiteMax) {
      chassis.setTwist(speed, -10);   // Turn Right
  }
  
  // while (rightLine >whiteMin && rightLine < whiteMax && (leftLine > whiteMin && leftLine < whiteMax)){
  //   chassis.setWheelSpeeds(30,30);
  //   if((leftLine > blackMin && leftLine < blackMax) && (rightLine >whiteMin && rightLine < whiteMax))
  //     chassis.setTwist(speed, 80);   //Turn Left
  //     //chassis.turnFor(-30, 40);
  //   if((leftLine < blackMin && leftLine > blackMax) && (rightLine <whiteMin && rightLine > whiteMax))
  //     chassis.setTwist(speed, -80);   //Turn Right
  // }
  
  // if(leftLine < whiteMin || leftLine > whiteMax)
  //   chassis.setTwist(speed, 80); //Turn Left
  //   //chassis.turnFor(30, 200);
  // else if (rightLine < whiteMin ||  rightLine >= whiteMax)
  //   chassis.setTwist(speed, -80);   //Turn Right
    //chassis.turnFor(30, 200);
  
  // else chassis.setWheelSpeeds(50, 50);
  // else 
  //   chassis.setWheelSpeeds(200, 200);
  //(leftLine<=250 and rightLine <= 250)
  // chassis.lineFollow(speed,LEFT_LINE_SENSE,RIGHT_LINE_SENSE,handleMotionComplete);
  // idle();
}

// Handles a key press on the IR remote
void handleKeyPress(int16_t keyPress)
{
  Serial.println("Key: " + String(keyPress));

  // TODO, Section 3.2: add "emergency stop"

  switch(robotState)
  {
      case ROBOT_IDLE:
        // TODO, Section 3.2: Handle up arrow button
        if(keyPress == UP_ARROW) drive(100, 30);
        if(keyPress == DOWN_ARROW) drive(-20, 30);
        if(keyPress == LEFT_ARROW) turn(90, 200);
        if(keyPress == RIGHT_ARROW) turn(-90, 200);
        if(keyPress == SETUP_BTN) beginLineFollowing();
        else Serial.println("Robot Idle");
        // TODO, Section 6.1: Handle remaining arrows
        break;

      case ROBOT_DRIVE_FOR:
        if(keyPress == ENTER_SAVE) {
          setLED(LOW);
          chassis.setWheelSpeeds(0,0);
          robotState = ROBOT_IDLE;
        }
        else Serial.println("Robot Drive");
        break;

      default:
        Serial.println("Unknown State");
        break;
  }
}


/*
 * The main loop for the program. The loop function is repeatedly called
 * after setup() is complete.
 */
void loop()
{
  // Checks for a key press on the remote
  // TODO, Section 3.1: Temporarily edit to pass true to getKeyCode()
  int16_t keyPress = decoder.getKeyCode();
  if(keyPress >= 0) handleKeyPress(keyPress);
  // A basic state machine
  switch(robotState)
  {
    case ROBOT_DRIVE_FOR: 
      chassis.printEncoderCounts();
      chassis.printSpeeds();
      delay(500);
      // TODO, Section 6.1: Uncomment to handle completed motion
      if(chassis.checkMotionComplete()) handleMotionComplete(); 
      Serial.print("Encoder Count: ");
      chassis.printEncoderCounts();
      break;

    case ROBOT_LINING:
      handleLineFollow(baseSpeed);
      if(keyPress == ENTER_SAVE){
        idle();
        Serial.print("Idle key pressed");
      }
      if(keyPress == VOLplus){
        baseSpeed += 5;
      }
      if(keyPress == VOLminus){
        baseSpeed -= 5;
      }
      break;

    case ROBOT_IDLE:

      break;

    default:
      
      Serial.println("Unknown State");
      break;
  }
  

}
