/*
 * Build-1 
 * Build Date: 2025-01-01
 * Build Time: 11:45 AM
 *
 * Code Summary:
 * - This program utilizes an ESP32 to control a servo and three ultrasonic sensors (front, left, and right).
 * - The center (front) sensor continuously scans for objects within a threshold distance.
 * - When an object is detected, the servo moves left, right, and back to the center while scanning each direction.
 * - The center sensor continues to monitor during the left and right scans, and the results are displayed on the serial monitor.
 * - After completing the left and right scans, the servo returns to the center position, waits for a configurable delay (2 seconds), and resets the detection flag.
 * 
 * Libraries Used:
 * - ESP32Servo: For controlling the servo motor.
 * - NewPing: For ultrasonic sensor readings.
 * 
 * Adjustable Parameters:
 * - OBJECT_THRESHOLD_FRONT, OBJECT_THRESHOLD_LEFT, OBJECT_THRESHOLD_RIGHT: Distance threshold (in cm) for detecting objects.
 * - OBJECT_DETECTION_DELAY: Delay (in seconds) after completing the scan and before restarting the detection.
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
float OBJECT_DETECTION_DELAY = 2.0; // in seconds

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

  // If object is detected, stop scanning and perform servo movements
  if (!objectDetected) {
    objectDetected = (distanceFront <= OBJECT_THRESHOLD_FRONT || distanceLeft <= OBJECT_THRESHOLD_LEFT || distanceRight <= OBJECT_THRESHOLD_RIGHT);
  }

  if (objectDetected) {
    // Stop scanning and perform movements (non-blocking)
    Serial.println("Object detected! Moving servo...");
    
    // Move the servo to the left position and scan
    myservo.write(SERVO_LEFT);
    delay(500);  // Wait for servo to stabilize
    int leftScanDistance = sonarLeft.ping_cm();
    if (leftScanDistance == 0) leftScanDistance = MAX_DISTANCE;
    Serial.print("Left Scan Distance: ");
    Serial.println(leftScanDistance);

    // During left scan, the center sensor (front sensor) is also reading:
    int frontScanDuringLeft = sonarFront.ping_cm();
    if (frontScanDuringLeft == 0) frontScanDuringLeft = MAX_DISTANCE;
    Serial.print("Center Sensor (Front) During Left Scan: ");
    Serial.println(frontScanDuringLeft);

    // Move the servo to the right position and scan
    myservo.write(SERVO_RIGHT);
    delay(500);  // Wait for servo to stabilize
    int rightScanDistance = sonarRight.ping_cm();
    if (rightScanDistance == 0) rightScanDistance = MAX_DISTANCE;
    Serial.print("Right Scan Distance: ");
    Serial.println(rightScanDistance);

    // During right scan, the center sensor (front sensor) is also reading:
    int frontScanDuringRight = sonarFront.ping_cm();
    if (frontScanDuringRight == 0) frontScanDuringRight = MAX_DISTANCE;
    Serial.print("Center Sensor (Front) During Right Scan: ");
    Serial.println(frontScanDuringRight);

    // Return the servo to the center
    myservo.write(SERVO_MIDDLE);
    delay(500);  // Wait for servo to stabilize

    // Delay for 2 seconds after returning to the center
    delay(OBJECT_DETECTION_DELAY * 1000); // Convert seconds to milliseconds

    // Reset object detection flag after scanning
    objectDetected = false;
  }
}
