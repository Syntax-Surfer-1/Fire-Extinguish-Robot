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

  // Check if object is detected based on any sensor
  if (!objectDetected) {
    objectDetected = (distanceFront <= OBJECT_THRESHOLD_FRONT || distanceLeft <= OBJECT_THRESHOLD_LEFT || distanceRight <= OBJECT_THRESHOLD_RIGHT);
  }

  if (objectDetected) {
    // Stop moving forward
    Serial.println("Object detected! Stopping forward movement.");

    // Scan for objects on the left, right, and center using the servo
    myservo.write(SERVO_LEFT);
    delay(500);  // Wait for servo to stabilize
    int leftScanDistance = sonarLeft.ping_cm();
    if (leftScanDistance == 0) leftScanDistance = MAX_DISTANCE;
    Serial.print("Left Scan Distance: ");
    Serial.println(leftScanDistance);

    myservo.write(SERVO_RIGHT);
    delay(500);  // Wait for servo to stabilize
    int rightScanDistance = sonarRight.ping_cm();
    if (rightScanDistance == 0) rightScanDistance = MAX_DISTANCE;
    Serial.print("Right Scan Distance: ");
    Serial.println(rightScanDistance);

    myservo.write(SERVO_MIDDLE);
    delay(500);  // Wait for servo to stabilize

    // Check if all sensors detect an object (reverse condition)
    if (distanceFront <= OBJECT_THRESHOLD_FRONT && distanceLeft <= OBJECT_THRESHOLD_LEFT && distanceRight <= OBJECT_THRESHOLD_RIGHT) {
      Serial.println("Object detected on all sides. Reversing and changing direction.");
      // Simulate reversing
      Serial.println("Left Wheel: Backward, Right Wheel: Backward");

      // Change direction randomly or 180 degrees (for simulation purposes, print message)
      Serial.println("Changing direction randomly or turning 180 degrees...");
    }
    else if (distanceRight <= OBJECT_THRESHOLD_RIGHT) {
      Serial.println("Object detected on the right. Turning left.");
      // Simulate turning left
      Serial.println("Left Wheel: Forward, Right Wheel: Backward");
    }
    else if (distanceLeft <= OBJECT_THRESHOLD_LEFT) {
      Serial.println("Object detected on the left. Turning right.");
      // Simulate turning right
      Serial.println("Left Wheel: Backward, Right Wheel: Forward");
    }
    else if (leftScanDistance <= OBJECT_THRESHOLD_LEFT && rightScanDistance <= OBJECT_THRESHOLD_RIGHT) {
      // If object detected on both sides (left and right)
      Serial.println("Object detected on both left and right. Reversing and changing direction.");
      // Simulate reversing
      Serial.println("Left Wheel: Backward, Right Wheel: Backward");

      // Change direction randomly or 180 degrees (for simulation purposes, print message)
      Serial.println("Changing direction randomly or turning 180 degrees...");
    }

    // Reset the object detection flag
    objectDetected = false;
    delay(OBJECT_DETECTION_DELAY * 1000); // Wait for the next scan
  }
}
