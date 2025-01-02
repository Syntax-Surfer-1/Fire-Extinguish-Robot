# ESP32 Object Detection and Servo Control Project 🎯🎯🎯

## Build Information 🎉🎉🎉

### Build-2 🎯
- **Build Date:** 02-01-2025
- **Build Time:** 4:50 PM

### Code Summary 🛠️
This project implements an ESP32-based system for object detection and servo control using ultrasonic sensors. Below are the key features:

- **Object Detection**: Three ultrasonic sensors (front, left, and right) detect objects within predefined threshold distances.
- **Reverse and Scan**: Upon detecting an object in front, the robot reverses, scans left and right with the servo motor, and returns to the center position.
- **Decision-Making**: After scanning, the robot decides which direction (left or right) to move based on the distance readings from the sensors.
- **Adjustable Parameters**: 
  - **LOOK_ANGLE**: The customizable angle for scanning left and right.
  - **OBJECT_THRESHOLD_* (Front, Left, Right)**: Distance thresholds for object detection.
  - **OBJECT_DETECTION_DELAY**: Configurable delay after scanning to ensure stability before the next cycle.
- **Servo Control**: The servo motor moves to predefined positions (left, right, center) during scanning, with a pause at each direction for accurate detection.

### Libraries Used 📚
- **ESP32Servo**: For controlling the servo motor.
- **NewPing**: For managing ultrasonic sensors and calculating distances.

### Hardware Components 🧰
- **Ultrasonic Sensors (HC-SR04)**: Front, left, and right sensors for object detection.
- **Servo Motor**: Controls the scanning mechanism of the robot.
- **ESP32 Development Board**: The brain of the system that processes sensor data and controls movement.

---

## Connection Details 🔌

### Components and Pin Connections 🧷

#### Ultrasonic Sensors (HC-SR04):
- **Front Sensor**:
  - *TRIG Pin*: GPIO 15
  - *ECHO Pin*: GPIO 4
- **Left Sensor**:
  - *TRIG Pin*: GPIO 18
  - *ECHO Pin*: GPIO 19
- **Right Sensor**:
  - *TRIG Pin*: GPIO 23
  - *ECHO Pin*: GPIO 22

#### Servo Motor:
- *Signal Pin*: GPIO 13
- *VCC Pin*: Connect to 5V (or external power source if required for high torque)
- *GND Pin*: Connect to ESP32 GND

#### ESP32 Power Supply:
- **USB Connection**: Use a micro-USB cable to power the ESP32 development board.
- **External Power Source**: Ensure proper voltage regulation if using an external power supply. 💡

---

## How It Works 🚀

1. **Object Detection**: The system continuously monitors the distances of objects in front, left, and right using ultrasonic sensors.
2. **Obstacle Handling**: When an object is detected:
   - The system reverses for a short duration to avoid collision.
   - The servo motor scans left and right to detect the position of the obstacle and then returns to the center.
3. **Decision-Making**: The system checks the distance of the left and right sensors, then decides the best direction to move based on the results.
4. **Continual Operation**: After scanning and deciding the direction, the system waits for the next object detection cycle.

---

## Contact Information ✉️

For further information or collaboration, please reach out via:

- **Email:** [yaxpatel6300@gmail.com](mailto:yaxpatel6300@gmail.com)
- **GitHub:** [Syntax-Surfer-1](https://github.com/Syntax-Surfer-1) 🌐

---

Feel free to fork the repository and contribute to the project. Contributions and feedback are highly appreciated! 🌟🌟🌟
