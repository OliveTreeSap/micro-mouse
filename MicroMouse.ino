/******************************************************************
Micromouse Right Wall Following Algorithm

Robot Specs:
- Dimensions: 14cm (width) x 11.5cm (length)
- Wheel diameter: 4.3cm
- Wheelbase: 12.2cm
- Encoder: 350 pulses per revolution
- Sensor offset: 3cm inward from wheels
*******************************************************************/

#include <SparkFun_TB6612.h>
#include <NewPing.h>
#include <math.h>

// ======================== PIN DEFINITIONS ========================
// Motor Driver Pins (TB6612FNG)
#define AIN1 7
#define AIN2 6
#define PWMA 5
#define BIN1 9
#define BIN2 10
#define PWMB 11
#define STBY 8

// Encoder Pins (Using available interrupt pins)
#define ENCA_L 3
#define ENCB_L A0
#define ENCA_R 2
#define ENCB_R A1  

// Ultrasonic Sensor Pins
#define TRIG_L A3
#define ECHO_L A2
#define TRIG_R 13
#define ECHO_R 12
#define TRIG_M A4
#define ECHO_M A5

#define MAX_DISTANCE 100 // Maximum distance for sensors (cm)

// ======================== CONTROL PARAMETERS ========================
// PID Gains
const float KP_WALL = 0.5;              // Proportional gain
const float KI_WALL = 0.03;              // Integral gain  
const float KD_WALL = 0.5;             // Derivative gain

// Target distances
const float TARGET_WALL_DIST = 6.0;      // Target distance from right wall (cm)

// Wall Detection Thresholds
const float WALL_THRESHOLD = 9.0;       // Max distance to consider wall present (cm)
const float MIN_WALL_DIST = 2.5;         // Minimum safe distance (cm)
const float FRONT_THRESHOLD = 6.5;        // Distance to trigger front wall turn (cm)

// Motor Control
const int BASE_SPEED = 80;             // Base speed (PWM 20-255)

// Turn Parameters
const int TURN_SPEED = 60;
const int TURN_90_PULSES = 230;        // Number of pulses for 90° turn

// ======================== GLOBAL VARIABLES ========================
// Motor objects
const int offsetA = -1;
const int offsetB = -1;
Motor motorR = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motorL = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// Encoder counters
volatile long encoderL = 0;
volatile long encoderR = 0;

// NewPing sensor objects
NewPing sonarLeft(TRIG_L, ECHO_L, MAX_DISTANCE);
NewPing sonarRight(TRIG_R, ECHO_R, MAX_DISTANCE);
NewPing sonarFront(TRIG_M, ECHO_M, MAX_DISTANCE);

// Sensor readings (with smoothing)
float oldLeftSensor = 0, oldRightSensor = 0, oldFrontSensor = 0;
float frontSensor = 0, leftSensor = 0, rightSensor = 0;
float lSensor, rSensor, fSensor;

// PID variables
float wallError = 0;
float wallLastError = 0;
float wallIntegral = 0;

// Wall detection flags
boolean frontwall = false;
boolean leftwall = false;
boolean rightwall = false;

// ======================== SETUP ========================
void setup() {
  Serial.begin(115200);
  
  // Setup encoders with interrupts
  pinMode(ENCA_L, INPUT);
  pinMode(ENCB_L, INPUT);
  pinMode(ENCA_R, INPUT);
  pinMode(ENCB_R, INPUT);
  
  // Encoders using hardware interrupt (pins 2, 3)
  attachInterrupt(digitalPinToInterrupt(ENCA_L), readEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_R), readEncoderR, RISING);
  
  Serial.println("========================================");
  Serial.println("    Micromouse RIGHT Wall Follower");
  Serial.println("========================================");
  Serial.println();
  delay(3000);
}

// ======================== MAIN LOOP ========================
void loop() {
  // Read all sensors
  ReadSensors();
  
  // Detect walls
  detectWalls();
  
  // PRIORITY 1: Check for front wall
  if (frontwall == true) {
    motorL.brake();
    motorR.brake();
    delay(300);
    if (rightwall == false) {
      // Turn right if there is no right wall
      if (leftSensor > 37) {
        turnLeft90();
      }
      else {
        turnRight90(); // Hardcoded turn, maze specific :( ts so ah ngl
      }

      //Clear PID errors after turn
      wallError = 0;
      wallLastError = 0;
      wallIntegral = 0;

      delay(200);
      return; // Exit loop and restart
    }
    // Turn left if there is a wall on the right
    turnLeft90();
    
    // Clear PID errors after turn
    wallError = 0;
    wallLastError = 0;
    wallIntegral = 0;
    
    delay(200);
    return; // Exit loop and restart
  }
  if (rightwall == false){ // if there is no right wall (even there is no front wall) turn right
    delay(200); // adjust for the robot to stop in the right place, ideally in the middle of the square
    motorL.brake();
    motorR.brake();
    delay(300);

    turnRight90();

    //Clear PID errors after turn
    wallError = 0;
    wallLastError = 0;
    wallIntegral = 0;

    delay(200);
    return; // Exit loop and restart
  } 
  
  // If there is no front wall, go straight using PID to balance the two sides
  balancedWallFollowingPID();
  
  // Safety: stop if all sensors show error readings
  if ((leftSensor == 0 || leftSensor > 100) && 
      (rightSensor == 0 || rightSensor > 100) && 
      (frontSensor == 0 || frontSensor > 100)) {
    motorL.brake();
    motorR.brake();
  }
  
  // Debug output every 500ms
  if (millis() % 500 < 50) {
    printDebugInfo();
  }
}

// ======================== BALANCED WALL FOLLOWING PID ========================
void balancedWallFollowingPID() {
  float errorP;
  
  // Check which walls are present and calculate appropriate error
  if (leftwall && rightwall) {
    // Both walls present - balance between them using modulo for cell spacing
    float leftDist = fmodf(leftSensor, 20.0);
    float rightDist = fmodf(rightSensor, 20.0);
    errorP = leftDist - rightDist;
  }
  else if (leftwall && !rightwall) {
    // Only left wall - maintain target distance from it
    float leftDist = fmodf(leftSensor, 20.0);
    errorP = leftDist - TARGET_WALL_DIST;
  }
  else if (!leftwall && rightwall) {
    // Only right wall - maintain target distance from it
    float rightDist = fmodf(rightSensor, 20.0);
    errorP = TARGET_WALL_DIST - rightDist;
  }
  else {
    // No walls detected - go straight, no correction
    errorP = 0;
  }
  
  // Integral term with anti-windup
  // wallIntegral += errorP;
  // wallIntegral = constrain(wallIntegral, -100, 100);
  
  // Derivative term
  float wallDerivative = errorP - wallLastError;
  
  // PID output
  float pidOutput = (KP_WALL * errorP) + 
                    // (KI_WALL * wallIntegral) + 
                    (KD_WALL * wallDerivative);
  
  // Constrain PID output
  pidOutput = constrain(pidOutput, -BASE_SPEED * 0.6, BASE_SPEED * 0.6);
  
  // Calculate motor speeds
  int leftSpeed = BASE_SPEED - pidOutput;
  int rightSpeed = BASE_SPEED + pidOutput;
  
  // Apply speed limits
  leftSpeed = constrain(leftSpeed, 20, 255);
  rightSpeed = constrain(rightSpeed, 20, 255);
  
  // Drive motors
  motorL.drive(leftSpeed);
  motorR.drive(rightSpeed);
  
  // Update for next iteration
  wallLastError = errorP;
}

// ======================== SENSOR READING ========================
void ReadSensors() {
  // Read raw sensor values
  lSensor = sonarLeft.ping_cm();
  rSensor = sonarRight.ping_cm();
  fSensor = sonarFront.ping_cm();
  
  // Smooth readings by averaging with previous values
  leftSensor = (lSensor + oldLeftSensor) / 2.0;
  rightSensor = (rSensor + oldRightSensor) / 2.0;
  frontSensor = (fSensor + oldFrontSensor) / 2.0;
  
  // Save current readings for next iteration
  oldLeftSensor = leftSensor;
  oldRightSensor = rightSensor;
  oldFrontSensor = frontSensor;
  
  delay(15); // Small delay between sensor readings
}

// ======================== WALL DETECTION ========================
void detectWalls() {
  // Right wall detection
  rightwall = (rightSensor > MIN_WALL_DIST && rightSensor < WALL_THRESHOLD);
  
  // Left wall detection
  leftwall = (leftSensor > MIN_WALL_DIST && leftSensor < WALL_THRESHOLD);
  
  // Front wall detection
  frontwall = ((frontSensor > 0 && frontSensor < FRONT_THRESHOLD) || 
               (frontSensor == 0)); // Treat 0 as "too close"
}

// ======================== TURNING FUNCTIONS ========================
void turnLeft90() {
  motorL.brake();
  motorR.brake();
  delay(200);
  
  // Reset encoders
  encoderL = 0;
  encoderR = 0;
  
  // Turn left: left motor backward, right motor forward
  motorL.drive(-TURN_SPEED);
  motorR.drive(TURN_SPEED);
  
  // Wait until turned 90 degrees
  while (abs(encoderR) < TURN_90_PULSES) {
    delay(10);
  }
  
  motorL.brake();
  motorR.brake();
  
  Serial.println(encoderR);
  delay(300);
}

void turnRight90() {
  motorL.brake();
  motorR.brake();
  delay(200);
  
  // Reset encoders
  encoderL = 0;
  encoderR = 0;
  
  // Turn right: right motor backward, left motor forward
  motorL.drive(TURN_SPEED);
  motorR.drive(-TURN_SPEED);
  
  // Wait until turned 90 degrees
  while (abs(encoderL) < TURN_90_PULSES) {
    delay(10);
  }
  
  motorL.brake();
  motorR.brake();
  
  Serial.println(encoderL);
  delay(300);
}

void turnAround() {
  motorL.brake();
  motorR.brake();
  delay(200);
  
  // Reset encoders
  encoderL = 0;
  encoderR = 0;
  
  // Turn left 180 degrees
  motorL.drive(-TURN_SPEED);
  motorR.drive(TURN_SPEED);
  
  // Wait until turned 180 degrees
  while (abs(encoderR) < (TURN_90_PULSES * 2)) {
    delay(10);
  }
  
  motorL.brake();
  motorR.brake();
  
  Serial.println(encoderR);
  delay(300);
}

// ======================== ENCODER INTERRUPTS ========================
// Left encoder interrupt
void readEncoderL() {
  int b = digitalRead(ENCB_L);
  if (b > 0) {
    encoderL++;
  } else {
    encoderL--;
  }
}

// Right encoder interrupt
void readEncoderR() {
  int b = digitalRead(ENCB_R);
  if (b > 0) {
    encoderR++;
  } else {
    encoderR--;
  }
}

// ======================== DEBUG OUTPUT ========================
void printDebugInfo() {
  Serial.print("L:");
  Serial.print(leftSensor, 1);
  Serial.print("cm  R:");
  Serial.print(rightSensor, 1);
  Serial.print("cm  F:");
  Serial.print(frontSensor, 1);
  Serial.print("cm | Err:");
  Serial.print(wallError, 2);
  Serial.print(" | Walls: ");
  Serial.print(leftwall ? "L" : "-");
  Serial.print(frontwall ? "F" : "-");
  Serial.print(rightwall ? "R" : "-");
  Serial.print(" | EncL:");
  Serial.print(encoderL);
  Serial.print(" EncR:");
  Serial.println(encoderR);
}
