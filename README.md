# ESP32 Object Detection and Servo Control Project 🎯🎯🎯

## Build Information 🎉🎉🎉

### Build-4 🎯
- **Build Date:** 01-18-2025
- **Build Time:** 7:30 PM

### Code Summary 🛠️
This program uses an ESP32 to control a servo, three ultrasonic sensors (front, left, and right), and an L298N motor driver for controlling wheels. Below are the key features:

- **Object Detection**: The robot moves forward while continuously scanning distances using the ultrasonic sensors.
- **Reverse and Scan**: When an object is detected in front, the robot reverses, then the servo performs a left-right scan, pausing at each side and returning to the center.
- **Decision-Making**: The decision to turn left or right is made based on distance readings from the left and right sensors. If both sides are blocked, the robot continues reversing.
- **Adjustable Parameters**:
  - **OBJECT_THRESHOLD_FRONT, OBJECT_THRESHOLD_LEFT, OBJECT_THRESHOLD_RIGHT**: Distance threshold (in cm) for detecting objects.
  - **OBJECT_DETECTION_DELAY**: Delay (in seconds) after object detection or scans.
  - **LOOK_ANGLE**: Maximum angle (up to 90°) for servo movement to the left and right.
  - **SPEED_INCREMENT**: Amount by which motor speed increases for smoother ramp-up.
- **Motor Speed Control**: Motor speed gradually ramps up from a starting value to a maximum speed, making the movement smoother and preventing sudden jerks.
- **Bug Fixes**: Smoother transitions between movements and improved distance reading handling.

### Libraries Used 📚
- **ESP32Servo**: For controlling the servo motor.
- **NewPing**: For ultrasonic sensor readings.

### Adjustable Parameters 🛠️
- **OBJECT_THRESHOLD_FRONT, OBJECT_THRESHOLD_LEFT, OBJECT_THRESHOLD_RIGHT**: Distance threshold for detecting objects in front, left, and right.
- **OBJECT_DETECTION_DELAY**: Delay time after object detection or scans to ensure stability before the next cycle.
- **LOOK_ANGLE**: Maximum servo angle for scanning left and right.
- **SPEED_INCREMENT**: Incremental increase in motor speed for smoother movement.

### Key Enhancements from Build-3 🔧
- **Motor Speed Control**: Introduced PWM-based ramp-up of motor speed for smoother transitions.
- **Improved Object Detection Handling**: Bug fixes for better scanning behavior and transitions between movements.
- **Servo Scanning Precision**: Updated the servo scanning function for better precision and response time.
- **Code Cleanup**: General improvements for readability and stability.

### Hardware Components 🧰
- **Ultrasonic Sensors (HC-SR04)**: Front, left, and right sensors for object detection.
- **Servo Motor**: Controls the scanning mechanism of the robot.
- **L298N Motor Driver**: Controls the movement of the robot's wheels.
- **ESP32 Development Board**: The brain of the system that processes sensor data and controls movement.

---

## Connection Details 🔌

### Components and Pin Connections 🧷

#### Ultrasonic Sensors (HC-SR04):
- **Front Sensor**:
  - **TRIG Pin**: GPIO 18
  - **ECHO Pin**: GPIO 19
- **Left Sensor**:
  - **TRIG Pin**: GPIO 15
  - **ECHO Pin**: GPIO 4
- **Right Sensor**:
  - **TRIG Pin**: GPIO 23
  - **ECHO Pin**: GPIO 22

#### Servo Motor:
- **Signal Pin**: GPIO 13
- **VCC Pin**: Connect to 5V (or external power source if required for high torque)
- **GND Pin**: Connect to ESP32 GND

#### Motor Driver (L298N):
- **OUT-1**: GPIO 12
- **OUT-2**: GPIO 14
- **OUT-3**: GPIO 27
- **OUT-4**: GPIO 26

#### ESP32 Power Supply:
- **USB Connection**: Use a micro-USB cable to power the ESP32 development board.
- **External Power Source**: Ensure proper voltage regulation if using an external power supply. 💡

---

## How It Works 🚀

1. **Object Detection**: The system continuously monitors the distances of objects in front, left, and right using ultrasonic sensors.
2. **Obstacle Handling**: When an object is detected:
   - The system reverses for a short duration to avoid collision.
   - The servo motor scans left and right to detect the position of the obstacle and then returns to the center.
3. **Decision-Making**: The system checks the distance of the left and right sensors, then decides the best direction to move based on the results. If both sides are blocked, the robot continues reversing.
4. **Motor Speed Control**: The motor speed gradually increases for smoother movement, avoiding sudden jerks.
5. **Continual Operation**: After scanning and deciding the direction, the system waits for the next object detection cycle.

---

## Contact Information ✉️

For further information or collaboration, please reach out via:

- **Email:** [yaxpatel6300@gmail.com](mailto:yaxpatel6300@gmail.com)
- **GitHub:** [Syntax-Surfer-1](https://github.com/Syntax-Surfer-1) 🌐

---

Feel free to fork the repository and contribute to the project. Contributions and feedback are highly appreciated! 🌟🌟🌟
