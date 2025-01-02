# ESP32 Object # Detection and Servo Control Project 🎯🎯🎯

## Build Information 🎉🎉🎉

### Build-1 🎯
- **Build Date:** 2025-01-01
- **Build Time:** 11:45 AM

### Code Summary 🛠️
This project implements an ESP32-based system for object detection and servo control using ultrasonic sensors. Below are the key features:

- Three ultrasonic sensors (front, left, and right) detect objects within predefined threshold distances.
- When an object is detected:
  - The servo motor rotates left, right, and returns to the center to scan different directions.
  - Sensor readings from each direction are logged for debugging purposes.
- A delay of 2 seconds is applied after each scanning sequence before restarting the detection process. 🚀

### Libraries Used 📚
- **ESP32Servo**: For controlling the servo motor.
- **NewPing**: For managing ultrasonic sensors and calculating distances.

### Hardware Components 🧰
- Ultrasonic Sensors (HC-SR04): Front, left, and right sensors for object detection.
- Servo Motor: Controls the scanning mechanism.

---

## Connection Details 🔌

### Components and Pin Connections 🧷

#### Ultrasonic Sensors (HC-SR04):
- **Front Sensor**:
  - TRIG Pin: GPIO 15
  - ECHO Pin: GPIO 4
- **Left Sensor**:
  - TRIG Pin: GPIO 18
  - ECHO Pin: GPIO 19
- **Right Sensor**:
  - TRIG Pin: GPIO 23
  - ECHO Pin: GPIO 22

#### Servo Motor:
- Signal Pin: GPIO 13
- VCC Pin: Connect to 5V (or external power source if required for high torque)
- GND Pin: Connect to ESP32 GND

#### ESP32 Power Supply:
- **USB Connection**: Use a micro-USB cable to power the ESP32 development board.
- **External Power Source**: Ensure proper voltage regulation if using an external power supply. 💡

---

## Contact Information ✉️

For further information or collaboration, please reach out via:

- **Email:** [yaxpatel6300@gmail.com](mailto:yaxpatel6300@gmail.com)
- **GitHub:** [Syntax-Surfer-1](https://github.com/Syntax-Surfer-1) 🌐

---

Feel free to fork the repository and contribute to the project. Contributions and feedback are highly appreciated! 🌟🌟🌟

