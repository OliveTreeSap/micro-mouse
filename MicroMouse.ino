#include <SparkFun_TB6612.h>


// Set pins for the TB6612FNG
#define AIN1 10
#define BIN1 46
#define AIN2 11
#define BIN2 3
#define PWMA 12
#define PWMB 8
#define STBY 9

// Direction switch
const int offsetA = 1;
const int offsetB = 1;

// Initialize motors
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY); // Left motor
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY); // Right motor

// Set the echo and trigger pins for the HC-SR04 sensors
const int trigPinL = 38;
const int echoPinL = 39;
const int trigPinR = 2;
const int echoPinR = 42;
const int trigPinM = 41;
const int echoPinM = 40;

// Define sound speed in cm/uS
#define SOUND_SPEED 0.034

// Declare the variables that hold the readings from the sensors
float distanceCmL, distanceCmR, distanceCmM;

// Forward declarations
float readSensor(int trigPin, int echoPin);

void setup() {
  // Starts the serial communication
  Serial.begin(115200);

  // Set the trigger pins as output and echo pins as input
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  pinMode(trigPinM, OUTPUT);
  pinMode(echoPinM, INPUT);
}

void loop() {
  // Read LEFT sensor
  distanceL = readSensor(trigPinL, echoPinL);
  
  delay(60); // Wait for ultrasonic pulse to dissipate
  
  // Read MIDDLE sensor
  distanceM = readSensor(trigPinM, echoPinM);
  
  delay(60);
  
  // Read RIGHT sensor
  distanceR = readSensor(trigPinR, echoPinR);
  
  // Print results
  Serial.print("Distance L(cm): ");
  Serial.println(distanceCmL);
  Serial.print("Distance M(cm): ");
  Serial.println(distanceCmM);
  Serial.print("Distance R(cm): ");
  Serial.println(distanceCmR);
  Serial.println("---");
  
  delay(100);
}


float readSensor(int trigPin, int echoPin){
  // A function that reads the distance output in centimeters from an HC-SR04 sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * SOUND_SPEED/2;
}
