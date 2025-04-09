//  *
//  * Build-5  
//  * Build Date: 02-22-2025  
//  * Build Time: 10:00 AM  
//  *
//  * Code Summary 🛠️
//  * - This program uses an ESP32 to control a servo, three ultrasonic sensors (front, left, and right), and an L298N motor driver for controlling wheels.  
//  * - The robot can operate in **two modes**:  
//  *     1️⃣ **Manual Mode (Bluetooth)**: Controlled via an Android app connected via ESP32’s built-in Bluetooth.  
//  *     2️⃣ **Autonomous Mode**: The robot moves forward while scanning for obstacles using ultrasonic sensors.  
//  * - If an object is detected in front, the robot reverses, performs a servo left-right scan, and decides the best path to take.  
//  * - Smooth motor speed control is implemented using PWM ramp-up for better movement transitions.  
//  * - **Seamless Mode Switching**: If Bluetooth commands are received, the robot follows manual control. Otherwise, it remains in autonomous mode.  
//  *
//  * Libraries Used 📚
//  * - **ESP32Servo**: For controlling the servo motor.  
//  * - **NewPing**: For ultrasonic sensor readings.  
//  * - **BluetoothSerial**: For Bluetooth communication with a mobile app.  
//  *
//  * Adjustable Parameters ⚙️
//  * - **OBJECT_THRESHOLD_FRONT, OBJECT_THRESHOLD_LEFT, OBJECT_THRESHOLD_RIGHT**: Distance threshold (in cm) for detecting objects.  
//  * - **OBJECT_DETECTION_DELAY**: Delay (in seconds) after object detection or scans.  
//  * - **LOOK_ANGLE**: Maximum angle (up to 90°) for servo movement to the left and right.  
//  * - **SPEED_INCREMENT**: Amount by which motor speed increases for smoother ramp-up.  
//  *
//  * Key Enhancements from Build-4 🚀
//  * - **Added Bluetooth control** using ESP32’s built-in Bluetooth (no need for HC-05/HC-06).  
//  * - **Implemented manual mode**: Commands (`F`, `B`, `L`, `R`, `S`) from the Android app control movement.  
//  * - **Seamless switching between manual & autonomous mode** based on Bluetooth connectivity.  
//  * - **Improved distance handling** for better obstacle avoidance.  
//  * - **Code optimizations** for better performance and stability.  
//  *

#include <Arduino.h>
#include <ESP32Servo.h>
#include <NewPing.h>
#include <BluetoothSerial.h>
BluetoothSerial SerialBT; // Create Bluetooth Serial object



bool extinguishActive = false;  // Whether we're extinguishing
unsigned long lastSweepTime = 0;
bool sweepDirection = false;
const int SERVO_SWEEP_INTERVAL = 300;  // Time between sweeps





#define AUTO_MODE 0
#define MANUAL_MODE 1
#define FIRE_MODE 2

int mode = AUTO_MODE;  // Default mode

// Pin assignments for ultrasonic sensors
#define TRIG_PIN_FRONT 18
#define ECHO_PIN_FRONT 19
#define TRIG_PIN_LEFT 15
#define ECHO_PIN_LEFT 4
#define TRIG_PIN_RIGHT 23
#define ECHO_PIN_RIGHT 22

// Pin assignments for ultrasonic sensors
#define FIRE_SENSOR_LEFT 25
#define FIRE_SENSOR_RIGHT 33
#define FIRE_FRONT_PIN 32

// Water Pump Pin 
#define PUMP_PIN 2

// Servo pin
#define SERVO_PIN 13

// Motor driver pins
#define LEFT_MOTOR_IN1 12
#define LEFT_MOTOR_IN2 14
#define RIGHT_MOTOR_IN3 27
#define RIGHT_MOTOR_IN4 26

// Maximum distance (in cm) for ultrasonic sensors
#define MAX_DISTANCE 200

// Fire Detection
#define WAIT_AFTER_EXTINGUISH 5000 // Wait time after fire is out (in ms)
#define FIRE_THRESHOLD 600         // Value below this = fire detected
#define FIRE_NEAR_THRESHOLD  400   // Fire is close

// Distance threshold for triggering movement (adjustable)
int OBJECT_THRESHOLD_FRONT = 30;  // in cm
int OBJECT_THRESHOLD_SIDE = 18;   // in cm

// Delay after object detection (adjustable)
float OBJECT_DETECTION_DELAY = 0.05;  // Delay for stability (in seconds)

// Customizable angle for looking left and right
int LOOK_ANGLE = 65;  // Maximum angle for servo rotation

// Motor speed control variables
int motorSpeed = 0;    // Start with low speed (PWM) 
int defaultspeed = 200; //
int maxSpeed = defaultspeed;    // Max PWM value for motor speed (0 to 255
int speedIncrement = 5; // Speed increment per loop
int speedStart = 50;   // PWM value to start from (0 to 255)
int speedStepDelay = 20; // Delay for increasing speed (adjustable)

// Add a global flag to track motor speed ramp-up status
bool isMovingForward = false;



// Initialize ultrasonic sensors
NewPing sonarFront(TRIG_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);
NewPing sonarLeft(TRIG_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);
NewPing sonarRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);

// Initialize servo
Servo myservo;

// Function prototypes
// void checkBluetoothConnection();
void stopMovement();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
bool fireDetected();
void extinguishFire(int state);
void scanLeftRight(int &combinedLeft, int &combinedRight);
void handleFireMode(int distanceFront, int distanceLeft, int distanceRight);

void setup() {
  Serial.begin(115200);          // Start serial communication
  SerialBT.begin("ESP32_Car");   // Bluetooth name
  myservo.attach(SERVO_PIN);     // Attach servo to pin
  myservo.write(90);             // Set servo to center position

  // Set motor driver pins as outputs
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN3, OUTPUT);
  pinMode(RIGHT_MOTOR_IN4, OUTPUT);

  pinMode(PUMP_PIN, OUTPUT);
  //Fire sensor 
  pinMode(FIRE_SENSOR_LEFT, INPUT);
  pinMode(FIRE_FRONT_PIN, INPUT);
  pinMode(FIRE_SENSOR_RIGHT, INPUT);
}

void loop() {
  // checkBluetoothConnection();  // Check if Bluetooth is connected

  // Read distances from all three ultrasonic sensors
  int distanceFront = sonarFront.ping_cm();
  int distanceLeft = sonarLeft.ping_cm();
  int distanceRight = sonarRight.ping_cm();

  // Handle cases where no echo is detected (distance will be 0)
  if (distanceFront == 0) distanceFront = MAX_DISTANCE;
  if (distanceLeft == 0) distanceLeft = MAX_DISTANCE;
  if (distanceRight == 0) distanceRight = MAX_DISTANCE;

// Read fire sensor values
int fireLeftReading = analogRead(25);  // D25
int fireRightReading = analogRead(33); // D33
int fireFrontReading = analogRead(32); // D33

// Combine all data into one string
String data = String(distanceLeft) + "," +
              String(distanceFront) + "," +
              String(distanceRight) + "," +
              String(fireLeftReading) + "," +
              String(fireRightReading) + "," +
              0;

// Send data to Bluetooth
SerialBT.println(data);

  // Display sensor readings for debugging
  // Serial.println("======================");
  // Serial.printf("Front Distance: %d cm\n", distanceFront);
  // Serial.printf("Left Distance: %d cm\n", distanceLeft);
  // Serial.printf("Right Distance: %d cm\n", distanceRight);
  // Serial.println("======================");
  char command;

  if (extinguishActive) {
    unsigned long currentTime = millis();
    if (currentTime - lastSweepTime >= SERVO_SWEEP_INTERVAL) {
      lastSweepTime = currentTime;
  
      if (sweepDirection) {
        myservo.write(90 - LOOK_ANGLE);
      } else {
        myservo.write(90 + LOOK_ANGLE);
      }
  
      sweepDirection = !sweepDirection;
    }
  }
  

  // Check mode
  if (mode == MANUAL_MODE) {
    // Process Bluetooth commands for manual control
    if (SerialBT.available()) {
      command = SerialBT.read();
      Serial.print("Received: ");
      Serial.println(command);

      if (command == 'F') {
        Serial.println("Moving Forward");
        moveForward();
      } 
      else if (command == 'B') {
        Serial.println("Moving Backward");
        moveBackward();
      } 
      else if (command == 'L') {
        Serial.println("Turning Left");
        turnLeft();
      } 
      else if (command == 'R') {
        Serial.println("Turning Right");
        turnRight();
      } 
      else if (command == 'A') {
        mode = AUTO_MODE;
        Serial.println("Switched to Auto Mode");
      }   
      else if(command == 'S') {
        stopMovement();
        Serial.println("Stopping");
      }  
      else if (command == 'M') {
        Serial.println("Set To Max Speed");
        maxSpeed = 255;
      } 
      else if (command == 'D') {
        Serial.println("Set To Default Speed");
        maxSpeed = defaultspeed;
      } 
      else if (command == 'W') {
        extinguishActive = !extinguishActive; // Toggle the state
        extinguishFire(extinguishActive ? 1 : 0); // Call with 1 or 0 based on toggle
        Serial.println(extinguishActive ? "💧 Extinguish ON (W)" : "🛑 Extinguish OFF (W)");
      }// Add more commands as needed for manual control
      else {
        Serial.println("Unknown Command");

      }
    }
    // if (!SerialBT.available()){
    //   mode = AUTO_MODE;
    //   Serial.println("Conection lost");
    //   Serial.println("Switched to Auto Mode");
    // }
  } 
  else if (mode == AUTO_MODE) {
    if (SerialBT.available()) {
      char command = SerialBT.read();
      Serial.print("Received: ");
      Serial.println(command);
      if (command == 'M') {
        command = 'S';
        mode = MANUAL_MODE;
        Serial.println("Switched to Manual Mode");
      }
      else {
        Serial.println("Unknown Command");
      }
    }
    
    int fireLeft = analogRead(FIRE_SENSOR_LEFT);
    int fireRight = analogRead(FIRE_SENSOR_RIGHT);

    Serial.print("Fire Left: ");
    Serial.print(fireLeft);
    Serial.print(" | Fire Right: ");
    Serial.println(fireRight);

    if (fireLeft < FIRE_THRESHOLD || fireRight < FIRE_THRESHOLD) {
      Serial.println("🔥 Fire detected! Switching to FIRE MODE...");
      mode = FIRE_MODE;
      return;  // exit this cycle, FIRE_MODE will run in next loop
    }

    // Add stabilization delay after detecting objects
    delay(OBJECT_DETECTION_DELAY * 1000);

    // Handle object detection logic
    if (distanceFront <= OBJECT_THRESHOLD_FRONT || 
        (distanceLeft <= OBJECT_THRESHOLD_SIDE && distanceRight <= OBJECT_THRESHOLD_SIDE)) {
      Serial.println("Object detected in front or on both sides. Reversing...");
      stopMovement();
      moveBackward();
      delay(1000);  // Reverse for 1 second
      stopMovement();

      // Perform left-right scan
      int combinedLeft = 0, combinedRight = 0;
      scanLeftRight(combinedLeft, combinedRight);

      // Display combined data for debugging
      Serial.printf("Combined Left Data: %d cm\n", combinedLeft);
      Serial.printf("Combined Right Data: %d cm\n", combinedRight);

      // Decide direction based on combined data
      if (combinedLeft > OBJECT_THRESHOLD_SIDE || combinedRight > OBJECT_THRESHOLD_SIDE) {
        if (combinedLeft > combinedRight) {
          Serial.println("Turning left...");
          turnLeft();
        } else {
          Serial.println("Turning right...");
          turnRight();
        }
      } else {
        Serial.println("Both sides blocked or too short. Reversing again...");
        moveBackward();
        delay(1000);  // Reverse again for 1 second
      }
    } 
    else if (distanceLeft <= OBJECT_THRESHOLD_SIDE) {
      Serial.println("Object detected on the left. Turning right...");
      turnRight();
    } 
    else if (distanceRight <= OBJECT_THRESHOLD_SIDE) {
      Serial.println("Object detected on the right. Turning left...");
      turnLeft();
    } 
    else {
      Serial.println("No objects detected. Moving forward...");
      moveForward();
    }
    
  }

  // Mode handler
  else if (mode == FIRE_MODE) {
    handleFireMode(distanceFront, distanceLeft, distanceRight);
  }

  // Short delay to avoid flooding the serial output
  delay(50);
}

// Function to check Bluetooth connection
// void checkBluetoothConnection() {
//   if (!SerialBT.available()) {
//     Serial.println("Bluetooth disconnected! Switching to AUTO mode.");
//     mode = AUTO_MODE;
//   }
//   else if(SerialBT.available()) {
//     mode = MANUAL_MODE;
//     Serial.println("Bluetooth connected! Switching to MANUAL mode.");
//   }
// }



void scanLeftRight(int &combinedLeft, int &combinedRight) {
  int leftSensor, rightSensor, frontSensorLeft, frontSensorRight;

  // Look left
  myservo.write(90 + LOOK_ANGLE);
  delay(500);  // Wait for servo to reach position
  leftSensor = sonarLeft.ping_cm();
  frontSensorLeft = sonarFront.ping_cm();
  if (leftSensor == 0) leftSensor = MAX_DISTANCE;
  if (frontSensorLeft == 0) frontSensorLeft = MAX_DISTANCE;

  // Look right
  myservo.write(90 - LOOK_ANGLE);
  delay(500);  // Wait for servo to reach position
  rightSensor = sonarRight.ping_cm();
  frontSensorRight = sonarFront.ping_cm();
  if (rightSensor == 0) rightSensor = MAX_DISTANCE;
  if (frontSensorRight == 0) frontSensorRight = MAX_DISTANCE;

  // Return to center
  myservo.write(90);
  delay(500);  // Wait for servo to return to cen
  // Combine data from front and side sensors
  combinedLeft = min(leftSensor, frontSensorLeft);
  combinedRight = min(rightSensor, frontSensorRight);
}

void stopMovement() {
  analogWrite(LEFT_MOTOR_IN1, 0);
  analogWrite(LEFT_MOTOR_IN2, 0);
  analogWrite(RIGHT_MOTOR_IN3, 0);
  analogWrite(RIGHT_MOTOR_IN4, 0);
  isMovingForward = false;  // Reset the flag
  Serial.println("Motors stopped");
}


void moveForward() {
  if (!isMovingForward) {
    // Ramp up speed from low to max PWM duty cycle
    for (motorSpeed = speedStart; motorSpeed <= maxSpeed; motorSpeed += speedIncrement) {
      analogWrite(LEFT_MOTOR_IN1, motorSpeed);
      analogWrite(LEFT_MOTOR_IN2, 0);   // Forward direction for left motor
      analogWrite(RIGHT_MOTOR_IN3, motorSpeed);
      analogWrite(RIGHT_MOTOR_IN4, 0);  // Forward direction for right motor
      delay(speedStepDelay);  // Delay for smooth acceleration
    }
    isMovingForward = true;  // Mark that the motor has reached max speed
    Serial.println("Moving forward at max speed");
  } else {
    // Keep running at max speed
    analogWrite(LEFT_MOTOR_IN1, maxSpeed);
    analogWrite(LEFT_MOTOR_IN2, 0);   // Forward direction for left motor
    analogWrite(RIGHT_MOTOR_IN3, maxSpeed);
    analogWrite(RIGHT_MOTOR_IN4, 0);  // Forward direction for right motor
    Serial.println("Continuing forward at max speed");
  }
}

void moveBackward() {
  // Ramp up speed from low to max PWM duty cycle
  for (motorSpeed = speedStart; motorSpeed <= maxSpeed; motorSpeed += speedIncrement) {
    analogWrite(LEFT_MOTOR_IN1, 0);   // Backward direction for left motor
    analogWrite(LEFT_MOTOR_IN2, motorSpeed);
    analogWrite(RIGHT_MOTOR_IN3, 0);  // Backward direction for right motor
    analogWrite(RIGHT_MOTOR_IN4, motorSpeed);
    // delay(speedStepDelay);  // Delay for smooth acceleration
  }
  Serial.println("Moving backward at max speed"); 
}

void turnLeft() {
  Serial.println("Turning left at full power...");
  
  // Apply full power to turn left
  analogWrite(LEFT_MOTOR_IN1, 0);   
  analogWrite(LEFT_MOTOR_IN2, maxSpeed);  // Reverse left motor
  analogWrite(RIGHT_MOTOR_IN3, maxSpeed); // Forward right motor
  analogWrite(RIGHT_MOTOR_IN4, 0);

  delay(600);  // Adjust delay based on turning speed
  stopMovement();
}

void turnRight() {
  Serial.println("Turning right at full power...");

  // Apply full power to turn right
  analogWrite(LEFT_MOTOR_IN1, maxSpeed);  // Forward left motor
  analogWrite(LEFT_MOTOR_IN2, 0);
  analogWrite(RIGHT_MOTOR_IN3, 0);
  analogWrite(RIGHT_MOTOR_IN4, maxSpeed); // Reverse right motor

  delay(600);  // Adjust delay based on turning speed
  stopMovement();
}

bool fireDetected() {
  int fireLeft = analogRead(25);   // D25 = left fire sensor
  int fireRight = analogRead(33);  // D33 = right fire sensor
  int fireFront = analogRead(32);  // D32 = front fire sensor

  return (fireLeft < FIRE_THRESHOLD || fireRight < FIRE_THRESHOLD || fireFront < FIRE_THRESHOLD);
}

void extinguishFire(int state) {
  if (state == 1) {
    extinguishActive = true;
    digitalWrite(PUMP_PIN, HIGH);  // Turn on pump
  } else {
    extinguishActive = false;
    digitalWrite(PUMP_PIN, LOW);   // Turn off pump
    myservo.write(90);             // Center servo
  }
}


void handleFireMode(int distanceFront, int distanceLeft, int distanceRight) {
  int fireLeft = analogRead(25);
  int fireRight = analogRead(33);
  int fireFront = analogRead(32); // New fire sensor

  Serial.print("🔥 FIRE MODE => Left: ");
  Serial.print(fireLeft);
  Serial.print(" | Right: ");
  Serial.print(fireRight);
  Serial.print(" | Front: ");
  Serial.print(fireFront);
  Serial.print(" || Distance F: ");
  Serial.print(distanceFront);
  Serial.print(" | L: ");
  Serial.print(distanceLeft);
  Serial.print(" | R: ");
  Serial.println(distanceRight);

  // Obstacle avoidance
  if (distanceFront < 15) {
    stopMovement();
    delay(300);
    moveBackward();
    delay(300);
    if (distanceLeft > distanceRight) {
      turnLeft();
    } else {
      turnRight();
    }
    delay(500);
  } else if (distanceLeft < 10) {
    turnRight();
    delay(300);
  } else if (distanceRight < 10) {
    turnLeft();
    delay(300);
  }

  // Follow fire direction
  if (fireLeft < FIRE_THRESHOLD && fireRight >= FIRE_THRESHOLD && fireFront >= FIRE_THRESHOLD) {
    turnLeft();
  } else if (fireRight < FIRE_THRESHOLD && fireLeft >= FIRE_THRESHOLD && fireFront >= FIRE_THRESHOLD) {
    turnRight();
  } else {
    moveForward(); // Fire is likely centered
  }

  // When close to fire
  if ((fireLeft < FIRE_NEAR_THRESHOLD || fireRight < FIRE_NEAR_THRESHOLD || fireFront < FIRE_NEAR_THRESHOLD)
      && distanceFront < 10) {

    stopMovement();  // Important: stop before extinguishing

    while (fireDetected()) {
      extinguishFire(1); // Keep extinguishing
    }

    extinguishFire(0);  // Stop extinguisher and reset
    delay(WAIT_AFTER_EXTINGUISH);

    if (analogRead(25) > FIRE_THRESHOLD && analogRead(33) > FIRE_THRESHOLD && analogRead(32) > FIRE_THRESHOLD) {
      mode = AUTO_MODE;
      Serial.println("✅ Fire extinguished! Switching to AUTO_MODE");
    }    
  }
}
