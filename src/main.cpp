/*
 * Build-2
 * Build Date: 2025-01-02
 * Build Time: 02:30 PM
 *
 * Code Summary:
 * - Added virtual 2-wheel simulation displayed on the serial terminal.
 * - Enhanced object detection logic:
 *   - Moves forward if no object is within the threshold distance.
 *   - Stops and decides a direction based on the object's position (left, right, or all sides).
 *   - Simulates turning left or right and reversing when needed.
 */

#include <ESP32Servo.h>   // Correct library name
#include <NewPing.h>

// Pin assignments for ultrasonic sensors
#define TRIG_PIN_FRONT 15
#define ECHO_PIN_FRONT 4
#define TRIG_PIN_LEFT 18
#define ECHO_PIN_LEFT 19
#define TRIG_PIN_RIGHT 23
#define ECHO_PIN_RIGHT 22

// Servo pin
#define SERVO_PIN 13

// Maximum distance (in cm) for ultrasonic sensors
#define MAX_DISTANCE 200

// Servo positions
#define SERVO_MIDDLE 90
#define SERVO_LEFT 140
#define SERVO_RIGHT 40

// Distance threshold for triggering servo movement (adjustable)
int OBJECT_THRESHOLD_FRONT = 25; // in cm
int OBJECT_THRESHOLD_LEFT = 25;  // in cm
int OBJECT_THRESHOLD_RIGHT = 25; // in cm

// Delay after object detection (adjustable)
float OBJECT_DETECTION_DELAY = 10.0; // in seconds

// Timing variables for servo movements
unsigned long lastServoMoveTime = 0;
unsigned long servoMoveInterval = 1000; // Move servo every 1 second if needed

// Initialize ultrasonic sensors
NewPing sonarFront(TRIG_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);
NewPing sonarLeft(TRIG_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);
NewPing sonarRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);

// Initialize servo
Servo myservo;

bool objectDetected = false; // Flag to track if object is detected

// Wheel status variables for simulation
bool leftWheelForward = false;
bool rightWheelForward = false;

void setup() {
  Serial.begin(115200); // Start serial communication
  myservo.attach(SERVO_PIN); // Attach servo to pin
  myservo.write(SERVO_MIDDLE); // Move servo to middle position
}

void loop() {
  // Get current time
  unsigned long currentMillis = millis();

  // Read distances from all three ultrasonic sensors
  int distanceFront = sonarFront.ping_cm();
  int distanceLeft = sonarLeft.ping_cm();
  int distanceRight = sonarRight.ping_cm();

  // Handle cases where no echo is detected (distance will be 0)
  if (distanceFront == 0) distanceFront = MAX_DISTANCE;
  if (distanceLeft == 0) distanceLeft = MAX_DISTANCE;
  if (distanceRight == 0) distanceRight = MAX_DISTANCE;

  // Print sensor readings (optional)
  Serial.println("======================");
  Serial.print("Front Distance (Center): ");
  Serial.print(distanceFront);
  Serial.println(" cm");
  Serial.print("Left Distance: ");
  Serial.print(distanceLeft);
  Serial.println(" cm");
  Serial.print("Right Distance: ");
  Serial.print(distanceRight);
  Serial.println(" cm");
  Serial.println("======================");

  // If no object is detected, move forward
  if (!objectDetected) {
    if (distanceFront > OBJECT_THRESHOLD_FRONT && distanceLeft > OBJECT_THRESHOLD_LEFT && distanceRight > OBJECT_THRESHOLD_RIGHT) {
      // Move forward: Both wheels forward
      leftWheelForward = true;
      rightWheelForward = true;
      Serial.println("Left Wheel: Forward, Right Wheel: Forward");
    }
  }

  // If object detected, stop and start scanning
  if (distanceFront <= OBJECT_THRESHOLD_FRONT || distanceLeft <= OBJECT_THRESHOLD_LEFT || distanceRight <= OBJECT_THRESHOLD_RIGHT) {
    objectDetected = true;

    // Stop moving forward
    leftWheelForward = false;
    rightWheelForward = false;
    Serial.println("Object detected! Stopping forward movement.");

    // Scan using the servo motor
    // Move servo to the left to scan the left area
    myservo.write(SERVO_LEFT);
    delay(500);  // Wait for the servo to reach the position
    int leftScanDistance = sonarFront.ping_cm();  // Measure front distance from left position
    Serial.print("Left Scan Distance: ");
    Serial.println(leftScanDistance);

    // Move servo to the right to scan the right area
    myservo.write(SERVO_RIGHT);
    delay(500);  // Wait for the servo to reach the position
    int rightScanDistance = sonarFront.ping_cm();  // Measure front distance from right position
    Serial.print("Right Scan Distance: ");
    Serial.println(rightScanDistance);

    // Return servo to the middle position
    myservo.write(SERVO_MIDDLE);
    delay(500);  // Wait for the servo to return

    // Scan for object detection at front, left, and right sensors
    if (leftScanDistance <= OBJECT_THRESHOLD_LEFT) {
      // Object detected on the left
      Serial.println("Object detected on the left. Turning right.");
      leftWheelForward = true;  // Spin left wheel forward
      rightWheelForward = false;  // Spin right wheel backward
      Serial.println("Left Wheel: Forward, Right Wheel: Backward");
    }
    else if (rightScanDistance <= OBJECT_THRESHOLD_RIGHT) {
      // Object detected on the right
      Serial.println("Object detected on the right. Turning left.");
      leftWheelForward = false;  // Spin left wheel backward
      rightWheelForward = true;  // Spin right wheel forward
      Serial.println("Left Wheel: Backward, Right Wheel: Forward");
    }
    else {
      // No objects detected on the left or right, but object detected at the front
      // Check if the front sensors detect anything as well
      if (distanceFront <= OBJECT_THRESHOLD_FRONT) {
        Serial.println("Object detected directly in front. Reversing and changing direction.");
        leftWheelForward = false;
        rightWheelForward = false;
        Serial.println("Left Wheel: Backward, Right Wheel: Backward");
        delay(1000);  // Reverse for 1 second

        // Turn 180 degrees
        myservo.write(SERVO_LEFT);
        delay(500);  // Wait for the servo to reach the left position
        myservo.write(SERVO_MIDDLE);  // Return to middle position
        Serial.println("Turning 180 degrees.");
        delay(500);

        // After turning, continue moving forward
        leftWheelForward = true;
        rightWheelForward = true;
        Serial.println("Left Wheel: Forward, Right Wheel: Forward");
      }
    }

    // Delay for 2 seconds before scanning again
    delay(OBJECT_DETECTION_DELAY * 1000);
    objectDetected = false;  // Reset object detection flag
  }

  // Print the wheel status to simulate movement
  if (leftWheelForward && rightWheelForward) {
    Serial.println("Left Wheel: Forward, Right Wheel: Forward");
  }
  else if (!leftWheelForward && !rightWheelForward) {
    Serial.println("Left Wheel: Backward, Right Wheel: Backward");
  }
  else if (leftWheelForward && !rightWheelForward) {
    Serial.println("Left Wheel: Forward, Right Wheel: Backward");
  }
  else if (!leftWheelForward && rightWheelForward) {
    Serial.println("Left Wheel: Backward, Right Wheel: Forward");
  }
}
