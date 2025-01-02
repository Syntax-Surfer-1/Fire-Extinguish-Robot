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
#include <ESP32Servo.h>
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

// Distance threshold for triggering servo movement (adjustable)
int OBJECT_THRESHOLD_FRONT = 25; // in cm
int OBJECT_THRESHOLD_LEFT = 25;  // in cm
int OBJECT_THRESHOLD_RIGHT = 25; // in cm

// Delay after object detection (adjustable)
float OBJECT_DETECTION_DELAY = 2.0; // Delay for stability

// Initialize ultrasonic sensors
NewPing sonarFront(TRIG_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);
NewPing sonarLeft(TRIG_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);
NewPing sonarRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);

// Initialize servo
Servo myservo;

// Function prototype for scanLeftRight()
void scanLeftRight();

void setup() {
  Serial.begin(115200); // Start serial communication
  myservo.attach(SERVO_PIN); // Attach servo to pin
  myservo.write(90); // Set servo to middle position (initially facing front)
}

void loop() {
  // Continually read distances from all three ultrasonic sensors
  int distanceFront = sonarFront.ping_cm();
  int distanceLeft = sonarLeft.ping_cm();
  int distanceRight = sonarRight.ping_cm();

  // Handle cases where no echo is detected (distance will be 0)
  if (distanceFront == 0) distanceFront = MAX_DISTANCE;
  if (distanceLeft == 0) distanceLeft = MAX_DISTANCE;
  if (distanceRight == 0) distanceRight = MAX_DISTANCE;

  // Print sensor readings (optional for debugging)
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

  // Handle obstacle detection in front
  if (distanceFront <= OBJECT_THRESHOLD_FRONT) {
    // Object detected in front: Stop and scan
    Serial.println("Object detected in front. Stopping...");
    stopMovement();

    // Perform left-right scan
    scanLeftRight();
    delay(OBJECT_DETECTION_DELAY * 1000); // Add delay for stability
  }
  else {
    // No object in front, check for objects on left or right
    if (distanceLeft <= OBJECT_THRESHOLD_LEFT && distanceRight <= OBJECT_THRESHOLD_RIGHT) {
      // Objects detected on both sides: Stop and scan
      Serial.println("Object detected on both sides. Stopping...");
      stopMovement();

      // Perform left-right scan
      scanLeftRight();
      delay(OBJECT_DETECTION_DELAY * 1000); // Add delay for stability
    }
    else if (distanceLeft <= OBJECT_THRESHOLD_LEFT) {
      // Object detected on left: Turn right
      Serial.println("Object detected on left. Turning right...");
      turnRight();
    }
    else if (distanceRight <= OBJECT_THRESHOLD_RIGHT) {
      // Object detected on right: Turn left
      Serial.println("Object detected on right. Turning left...");
      turnLeft();
    }
    else {
      // No objects detected: Move forward
      Serial.println("No objects detected. Moving forward...");
      moveForward();
    }
  }

  // Short delay to avoid flooding the serial output
  delay(50);
}

void scanLeftRight() {
  // Move the servo to scan left
  myservo.write(45); // Turn the servo left (45 degrees)
  delay(250); // Shorter delay for quicker response
  int leftScan = sonarLeft.ping_cm();
  if (leftScan == 0) leftScan = MAX_DISTANCE; // Handle no echo

  // Move the servo to scan right
  myservo.write(135); // Turn the servo right (135 degrees)
  delay(250); // Shorter delay for quicker response
  int rightScan = sonarRight.ping_cm();
  if (rightScan == 0) rightScan = MAX_DISTANCE; // Handle no echo

  // Decision-making based on scanned distances
  if (leftScan > rightScan) {
    // If the left scan distance is greater, go right
    Serial.println("Left side is clear, turning right...");
    turnRight();
  } else if (rightScan > leftScan) {
    // If the right scan distance is greater, go left
    Serial.println("Right side is clear, turning left...");
    turnLeft();
  } else {
    // If both sides are equally clear, choose one side to turn
    Serial.println("Both sides are equally clear. Turning left by default.");
    turnLeft();
  }

  // Revert to the center after scanning
  myservo.write(90); // Move the servo back to the middle position
  delay(250); // Shorter delay for quicker response
}

void stopMovement() {
  // Stop both wheels (simulated for now)
  Serial.println("Left Wheel: Stop, Right Wheel: Stop");
}

void moveForward() {
  // Move both wheels forward (simulated for now)
  Serial.println("Left Wheel: Forward, Right Wheel: Forward");
}

void turnLeft() {
  // Turn left (right wheel forward, left wheel backward)
  Serial.println("Left Wheel: Backward, Right Wheel: Forward");
  delay(500); // Turn left for 0.5 second
}

void turnRight() {
  // Turn right (left wheel forward, right wheel backward)
  Serial.println("Left Wheel: Forward, Right Wheel: Backward");
  delay(500); // Turn right for 0.5 second
}
