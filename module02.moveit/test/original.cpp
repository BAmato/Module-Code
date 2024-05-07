/*
 * Module 2 -- Move it!
 */ 

//TODO, Add your group information to the top of your code.

//Wheel diameter = 72mm = 7.2cm
//Track distance = 125mm = 12.5 cm

#include <Arduino.h>
#include <wpi-32u4-lib.h>
#include <Adafruit_GFX.h>
#include <IRdecoder.h>
#include <ir_codes.h>
#include <Chassis.h>


#include <limits.h>
#include <vector>

#include <QTRSensors.h>   //Library for reflective sensor array

#define LED_PIN 1
#define LED_IR_CALIBRATE 0
#define BUZZER_PIN 6

// Define the maximum number of vertices as per your arena size (# of destinations)
#define MAX_VERTICES 17

struct Edge {
    int to;
    int weight;
};

struct Node {
    int id;
    std::vector<Edge> edges;
};
// Global array of nodes
Node nodes[MAX_VERTICES];


QTRSensors qtr;   //Makes global object for reflective sensor array

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
uint16_t darkThreshold = 500;

int16_t keyPress = decoder.getKeyCode();    //Making keypres global so it can be read from other functions while running

// TODO, Section, 4.2: Add line to include Chassis.h


// Sets up the IR receiver/decoder object
const uint8_t IR_DETECTOR_PIN = 1;
IRDecoder decoder(IR_DETECTOR_PIN);
volatile int baseSpeed = 10;

volatile uint16_t leftLine;
volatile uint16_t rightLine;

int lastError = 0;
float Kp = 0.07;
float Ki = 0.0000;
float Kd = 0.6;

int P = 0;
int I = 0;
int D = 0;

const uint8_t maxspeedA = 20;
const uint8_t maxspeedB = 20;
const uint8_t basespeedA = 10;
const uint8_t basespeedB = 10;


Chassis chassis(7.2,1440,13.2);
// TODO, Section 4.2: Declare the chassis object (with default values)
// TODO, Section 6.2: Adjust parameters to better match actual motion


// A helper function for debugging

void setLED(bool value)
{
  Serial.println("setLED()");
  digitalWrite(LED_PIN, value);
}

// Defines the robot states
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE_FOR, ROBOT_LINING, ROBOT_DEBUG};
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

void QTR_init(void);

void dijkstraInit(void);

/*
 * This is the standard setup function that is called when the board is rebooted
 * It is used to initialize anything that needs to be done once.
 */
void setup() 
{
  
  // This will initialize the Serial at a baud rate of 115200 for prints
  // Be sure to set your Serial Monitor appropriately
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\n\n-------BEGINNING SETUP-------");
  
  // TODO, Section 4.2: Initialize the chassis (which also initializes the motors)
  chassis.init();

  // TODO, Section 5.1: Adjust the PID coefficients
  chassis.setMotorPIDcoeffs(7,2);

  // Initializes the IR decoder
  decoder.init();

  dijkstraInit();

  pinMode(BUZZER_PIN, OUTPUT);

  Serial.println("\n-----SETUP COMPLETE-------");
  idle();
  
}

// The setup phase of this example calibrates the sensors for ten seconds and
// turns on the Arduino's LED (usually on pin 13) while calibration is going
// on. During this phase, you should expose each reflectance sensor to the
// lightest and darkest readings they will encounter. For example, if you are
// making a line follower, you should slide the sensors across the line during
// the calibration phase so that each sensor can get a reading of how dark the
// line is and how light the ground is.  Improper calibration will result in
// poor readings.
//
// The main loop of the example reads the calibrated sensor values and uses
// them to estimate the position of a line. You can test this by taping a piece
// of 3/4" black electrical tape to a piece of white paper and sliding the
// sensor across it. It prints the sensor values to the serial monitor as
// numbers from 0 (maximum reflectance) to 1000 (minimum reflectance) followed
// by the estimated location of the line as a number from 0 to 5000. 1000 means
// the line is directly under sensor 1, 2000 means directly under sensor 2,
// etc. 0 means the line is directly under sensor 0 or was last seen by sensor
// 0 before being lost. 5000 means the line is directly under sensor 5 or was
// last seen by sensor 5 before being lost.

void QTR_init(){

  Serial.println("\n\n-------BEGINNING CALIBRATION-------");
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4}, SensorCount);    //A1 only receive 2.5V, unsure why
  //qtr.setEmitterPin(A2);

  delay(500);

  
  pinMode(LED_IR_CALIBRATE, OUTPUT);
  digitalWrite(LED_IR_CALIBRATE, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.

  for (uint16_t i = 0; i < 400; i++)
  {
    if(i % 40 == 0) chassis.setWheelSpeeds(10, 10);
    else if(i % 40 == 20) chassis.setWheelSpeeds (-10, -10);

    qtr.calibrate();
  }
  chassis.setWheelSpeeds(0, 0);

  digitalWrite(LED_IR_CALIBRATE, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on

  int sensorCurr = 0;
  int sensorMinMax = 0; //Max value from all min value readings
  int sensorMaxMin = 1000; //Min value from all max value readings

  for (uint8_t i = 0; i < SensorCount; i++)
  {

    sensorCurr = qtr.calibrationOn.minimum[i];
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');

    if(sensorCurr > sensorMinMax) sensorMinMax = sensorCurr;

  }

  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    sensorCurr = qtr.calibrationOn.maximum[i];
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');

    if(sensorCurr < sensorMaxMin) sensorMaxMin = sensorCurr;
  }


  Serial.println();
  Serial.println();
  delay(500);
  
  darkThreshold = sensorMaxMin - sensorMinMax;

  Serial.print("Maximum Min Val: "); Serial.println(sensorMinMax);
  Serial.print("Minimum Max Val: "); Serial.println(sensorMaxMin);
  Serial.print("Sensor darkThreshold: "); Serial.println(darkThreshold);

  //set state to idle
  robotState = ROBOT_IDLE;
  Serial.println("\n-----CALIBRATION COMPLETE-------\n");
}

//----------------------------------DIJKSTRA ALGORITHM-----------------------------------------------------

enum locations {
  warehouse, utep, airport, ftbliss, alamo, lascruces, outlets,
  countryclub, franklin, sunland, executive
};

enum streets {
  us375 = 5, us54 = 2, spur = 2, transmountain = 3, doniphan = 2, mesa = 2, i10 = 3, deadend = 1
};

enum intersections {
  sect1, sect2, sect3, sect4, sect5, sect6
};

const char* vertexNames[] = {
    "Warehouse", "UTEP", "Airport", "Ft Bliss", "Alamo", "Las Cruces", "Outlets",
    "Country Club", "Franklin", "Sunland Park", "Executive",
    "US-375", "US-54", "Spur", "Transmountain", "Doniphan", "Mesa", "I-10", "Dead End",
    "Intersection 1", "Intersection 2", "Intersection 3", "Intersection 4", "Intersection 5", "Intersection 6"
};

// Function to add edges to nodes
void addEdge(int from, int to, int weight) {
    nodes[from].edges.push_back({to, weight});
}

// Function to perform Dijkstra's algorithm
std::vector<int> dijkstra(int src) {
    std::vector<int> dist(MAX_VERTICES, INT_MAX);
    std::vector<bool> visited(MAX_VERTICES, false);
    dist[src] = 0;

    for (int i = 0; i < MAX_VERTICES - 1; i++) {
        int u = -1;
        for (int j = 0; j < MAX_VERTICES; j++) {
            if (!visited[j] && (u == -1 || dist[j] < dist[u]))
                u = j;
        }

        visited[u] = true;

        for (const auto& edge : nodes[u].edges) {
            int v = edge.to;
            int weight = edge.weight;
            if (!visited[v] && dist[u] != INT_MAX && dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
            }
        }
    }

    return dist;
}

void dijkstraInit(){
  Serial.println("DIJSTRA INIT BEGIN");
  // Initialize nodes
    for (int i = 0; i < MAX_VERTICES; i++) {
        nodes[i].id = i;
    }

    // Define your edges here (edges represent streets connecting two POI (vertex))
    // Streets that lead to POI are 1'
    // Transmountain and I-10 are longest @ 3.5'
    // All other are ~2'
    // US-375 ~ 5'
    addEdge(sect1, warehouse, us375);
    addEdge(sect1, utep, deadend);
    addEdge(sect1, sect2, us54);
    addEdge(sect1, sect6, i10);
    addEdge(sect2, airport, deadend);
    addEdge(sect2, sect3, spur);
    addEdge(sect3, alamo, deadend);
    
  
  // Running Dijkstra's algorithm from vertex 0
  // USED FOR DEBUGGING
    
    auto distances = dijkstra(0);
    for (int i = 0; i < MAX_VERTICES; i++) {
        if (distances[i] == INT_MAX) {
            Serial.print("Vertex ");
            Serial.print(vertexNames[i]);
            Serial.println(" is unreachable");
        } else {
            Serial.print("Shortest distance to vertex ");
            Serial.print(vertexNames[i]);
            Serial.print(" is ");
            Serial.println(distances[i]);
        }
    }
    Serial.println("DIJKSTRA INIT COMPLETE");
}



//--------------------------------------------------------------------------------------------
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

// void driveForward(int speed)
// {
//   chassis.setWheelSpeeds(speed,speed);
// }

// void driveReverse(int speed)
// {
//   chassis.setWheelSpeeds(-speed, -speed);
// }

void forward_movement(int speedA, int speedB){
  if (speedA < 0){
    speedA = 0 - speedA;
    chassis.setWheelSpeeds(-speedA,-speedB);
  }
  else{
    chassis.setWheelSpeeds(speedA,speedB);
  }
  if (speedB < 0){
    speedB = 0 - speedB;
    chassis.setWheelSpeeds(-speedA,-speedB);
  }
  else{
    chassis.setWheelSpeeds(speedA,speedB);
  }
}

void PID_control(){

  uint16_t positionLine = qtr.readLineBlack(sensorValues);
  // int position = (sensorValues[0] * 0 + sensorValues[1] * 1000 + sensorValues[2] * 2000 + sensorValues[3] * 3000 
  // + sensorValues[4] * 4000 + sensorValues[5] * 5000 ) / (sensorValues[0] + sensorValues[1] + sensorValues[2] +
  // sensorValues[3] + sensorValues[4] + sensorValues[5]);
  int error = (2000 - positionLine);

  // error = 0     NO NEED FOR CORRECTION, POS AT MIDDLE 
  // error < 0     POSITION NEAR RIGHT OF ARRAY (sensorValue[7])
  // error > 0     POSITION NEAR LEFT OF ARRAY (sensorValue[0])

  P = error;
  I = error + I;
  D = error - lastError;
  lastError = error;

  int motorSpeedChange = P * Kp + I * Ki + D * Kd; //Need values between [0,255]

  int motorSpeedA = baseSpeed - (motorSpeedChange/10);
  int motorSpeedB = baseSpeed + (motorSpeedChange/10);

  if(motorSpeedA > maxspeedA){
    motorSpeedA = maxspeedA;
  }
  if(motorSpeedB > maxspeedB){
    motorSpeedB = maxspeedB;
  }
  if(motorSpeedA < 0){
    motorSpeedA = 0;
  }
  if(motorSpeedB < 0){
    motorSpeedB = 0;
  }
  Serial.print("MotASpd: ");
  Serial.print(motorSpeedA);
  Serial.print("\tMotBSpd: ");
  Serial.print(motorSpeedB);

  Serial.print("A0: ");
  Serial.print(sensorValues[0]);
  Serial.print("  A1: ");
  Serial.print(sensorValues[1]);
  Serial.print("  A2: ");
  Serial.print(sensorValues[2]);
  Serial.print("  A3: ");
  Serial.print(sensorValues[3]);
  Serial.print("  A4: ");
  Serial.print(sensorValues[4]);
  // Serial.print("  A5: ");
  // Serial.print(sensorValues[5]);
  Serial.print("  Pos: ");
  Serial.print(qtr.readLineBlack(sensorValues));

  Serial.print(" Err: ");
  Serial.print(error);
  Serial.print(" Spd_Chng: ");
  Serial.println(motorSpeedChange);

  forward_movement(motorSpeedA, motorSpeedB);
}

void debug(){

  /*DIRECT VALUES PULLED FROM SENSOR*/
  // Serial.print("A0: ");
  // Serial.print(analogRead(A0));
  // Serial.print("  A1: ");
  // Serial.print(analogRead(A1));
  // Serial.print("  A2: ");
  // Serial.print(analogRead(A2));
  // Serial.print("  A3: ");
  // Serial.print(analogRead(A3));
  // Serial.print("  A4: ");
  // Serial.print(analogRead(A4));
  // Serial.print("  A5: ");
  // Serial.println(analogRead(A5));


  /*VALUES PULLED FROM QTR LIBRARY FUNCTION FOR ANALOG READ*/
  Serial.print("A0: ");
  Serial.print(sensorValues[0]);
  Serial.print("  A1: ");
  Serial.print(sensorValues[1]);
  Serial.print("  A2: ");
  Serial.print(sensorValues[2]);
  Serial.print("  A3: ");
  Serial.print(sensorValues[3]);
  Serial.print("  A4: ");
  Serial.print(sensorValues[4]);
  Serial.print("  A5: ");
  Serial.print(sensorValues[5]);
  Serial.print("Position: ");
  Serial.println(qtr.readLineBlack(sensorValues));

  

  robotState = ROBOT_DEBUG;
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
  Serial.println("ENTERING UNUSED FUNCTION: handleLineFollow()");
  
  // int whiteMin = 120;
  // int whiteMax = 220;
  // int blackMin = 500;   //Sensor min was 680 but depends on proximity to black tape so overshooting
  // int blackMax = 900;   //Sensor max was ~850 but depends on proimity so overshooting

  // // Read the sensor values inside the loop
  // leftLine = analogRead(LEFT_LINE_SENSE);
  // rightLine = analogRead(RIGHT_LINE_SENSE);
 
  // //Define error between sensors
  // int error = leftLine - rightLine;
  // int turnEffort = error * 1.1; //Error * K_p
  
  //   //        LIGHT                   BLACK
  //   //INNER LEFT 60   INNER RIGHT 60cm     LEFT 420    RIGHT 500
  //   //error -30 to 200 black line
  //   //

  //   //FAR LEFT   FAR RIGHT    FAR LEFT   FAR RIGHT
  //   //      BLACK                    WHITE
  //   //115          120          850        870

  //   //FAR LEFT   FAR RIGHT    FAR LEFT   FAR RIGHT
  //   //      BLACK                    WHITE
  //   //806          850              102            99
    
  // Serial.print("LEFT S:\t");
  // //Serial.print(analogRead(LEFT_LINE_SENSE));
  // Serial.print(leftLine);
  // Serial.print("\tRIGHT S:\t");
  // Serial.print(rightLine);
  // //Serial.println(analogRead(RIGHT_LINE_SENSE));
  
  // while ((rightLine > whiteMin && rightLine < whiteMax) && (leftLine > whiteMin && leftLine < whiteMax)) {
  //   // Straight movement when both sensors are on white
  //   chassis.setWheelSpeeds(30, 30);

  //   if (leftLine > blackMin && leftLine < blackMax) {
  //       // Adjust speed and turn left when the left sensor detects black
  //       chassis.setTwist(speed, 10);   // Turn Left
  //       // chassis.turnFor(-30, 40);
  //   } else if (rightLine > whiteMin && rightLine < whiteMax) {
  //       // Adjust speed and turn right when the right sensor detects white
  //       chassis.setTwist(speed, -10);   // Turn Right
  //   }

  //   // Update sensor readings
  //   leftLine = analogRead(LEFT_LINE_SENSE);
  //   rightLine = analogRead(RIGHT_LINE_SENSE);
  // }

  // // If either sensor is on black, adjust the direction accordingly
  // if (leftLine < whiteMin || leftLine > whiteMax) {
  //     chassis.setTwist(speed, 10); // Turn Left
  //     // chassis.turnFor(30, 200);
  // } else if (rightLine < whiteMin || rightLine >= whiteMax) {
  //     chassis.setTwist(speed, -10);   // Turn Right
  // }

}

void beep(boolean input) {
  // Play a tone on the buzzer pin
  tone(BUZZER_PIN, 500, 1000); // 440 Hz for 200 ms
  if (input) {
    Serial.println("BEEP");
  }
}

// //here's a nice opportunity to introduce boolean logic
bool checkIntersectionEvent(int16_t darkThreshold)
{
  static bool prevIntersection = false;

  bool retVal = false;

  bool leftDetect = (sensorValues[0]) > darkThreshold ? true : false;
  bool rightDetect = (sensorValues[SensorCount-1]) > darkThreshold ? true : false;

  bool intersection = leftDetect && rightDetect;
  if(intersection && !prevIntersection) retVal = true;
  prevIntersection = intersection;

  return retVal;
}

void handleIntersection(void)
{
  Serial.println("Intersection!");
  beep(true);

  //drive forward by dead reckoning to center the robot
  //chassis.driveFor(8, 5);
  chassis.setWheelSpeeds(0,0);

  robotState = ROBOT_IDLE;
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
        if(keyPress == REWIND) debug();
        if(keyPress == PLAY_PAUSE) QTR_init();

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

// void PID_control(){
//   uint16_t positionLine = 
// }

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
      //handleLineFollow(baseSpeed);
      PID_control();

      if(checkIntersectionEvent(darkThreshold)) handleIntersection();
      
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

    case ROBOT_DEBUG:
      if(keyPress == ENTER_SAVE){
        idle();
        Serial.print("Idle key pressed");
      }
      else debug();

    case ROBOT_IDLE:

      break;

    default:
      
      Serial.println("Unknown State");
      break;
  }
  

}