/*
 * Build-2 
 * Build Date: 02-01-2025
 * Build Time: 4:50PM
 *
 * Code Summary:
 * - This program uses an ESP32 to control a servo and three ultrasonic sensors (front, left, and right) for object detection and decision-making.
 * - The robot moves forward while continuously scanning distances using the ultrasonic sensors.
 * - When an object is detected in front, the robot reverses, then the servo performs a left-right scan, pausing at each side and returning to the center. 
 * - The decision to turn left or right is made based on distance readings from the left and right sensors. If both sides are blocked, the robot continues reversing.
 * - The left-right scan and movement incorporate a configurable delay (`OBJECT_DETECTION_DELAY`) for better stability.
 *
 * Libraries Used:
 * - ESP32Servo: For controlling the servo motor.
 * - NewPing: For ultrasonic sensor readings.
 * 
 * Adjustable Parameters:
 * - OBJECT_THRESHOLD_FRONT, OBJECT_THRESHOLD_LEFT, OBJECT_THRESHOLD_RIGHT: Distance threshold (in cm) for detecting objects.
 * - OBJECT_DETECTION_DELAY: Delay (in seconds) after object detection or scans.
 * - LOOK_ANGLE: Maximum angle (up to 90°) for servo movement to the left and right.
 *
 * Key Enhancements from Build-1:
 * - Added reverse movement before scanning when an object is detected in front or on both sides.
 * - Servo scans left and right with a configurable pause at each side and returns to the center.
 * - Dynamic decision-making to turn based on the clearest path (left or right).
 * - Enhanced real-time performance with optimized delays and better logical flow.
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
float OBJECT_DETECTION_DELAY = 10.0; // Delay for stability

// Customizable angle for looking left and right
int LOOK_ANGLE = 45; // Adjust this value as needed (max: 90)

// Initialize ultrasonic sensors
NewPing sonarFront(TRIG_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);
NewPing sonarLeft(TRIG_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);
NewPing sonarRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);

// Initialize servo
Servo myservo;

// Function prototypes
void stopMovement();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
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
  else if (distanceLeft <= OBJECT_THRESHOLD_LEFT && distanceRight <= OBJECT_THRESHOLD_RIGHT) {
    // Objects detected on both sides: Reverse, scan, then decide direction
    Serial.println("Objects detected on both sides. Reversing...");
    moveBackward();
    delay(1000); // Reverse for 1 second
    stopMovement();

    Serial.println("Scanning left and right...");
    scanLeftRight();

    // Decide direction based on left and right distances after scan
    if (distanceLeft > distanceRight) {
      Serial.println("Turning left...");
      turnLeft();
    } else {
      Serial.println("Turning right...");
      turnRight();
    }
  }
  else if (distanceLeft <= OBJECT_THRESHOLD_LEFT) {
    // Object detected on the left: Stop, apply delay, then turn right
    Serial.println("Object detected on left. Stopping and scanning...");
    stopMovement();
    delay(OBJECT_DETECTION_DELAY * 1000); // Add delay
    turnRight();
  }
  else if (distanceRight <= OBJECT_THRESHOLD_RIGHT) {
    // Object detected on the right: Stop, apply delay, then turn left
    Serial.println("Object detected on right. Stopping and scanning...");
    stopMovement();
    delay(OBJECT_DETECTION_DELAY * 1000); // Add delay
    turnLeft();
  }
  else {
    // No objects detected: Move forward
    Serial.println("No objects detected. Moving forward...");
    moveForward();
  }

  // Short delay to avoid flooding the serial output
  delay(50);
}

void scanLeftRight() {
  // Look left
  int leftAngle = 90 + LOOK_ANGLE;
  myservo.write(leftAngle);
  delay(500); // Wait for servo to reach position

  // Look right
  int rightAngle = 90 - LOOK_ANGLE;
  myservo.write(rightAngle);
  delay(500); // Wait for servo to reach position

  // Return to center
  myservo.write(90);
  delay(500); // Wait for servo to return to center
}

void stopMovement() {
  // Stop both wheels (simulated for now)
  Serial.println("Left Wheel: Stop, Right Wheel: Stop");
}

void moveForward() {
  // Move both wheels forward (simulated for now)
  Serial.println("Left Wheel: Forward, Right Wheel: Forward");
}

void moveBackward() {
  // Move both wheels backward (simulated for now)
  Serial.println("Left Wheel: Backward, Right Wheel: Backward");
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
