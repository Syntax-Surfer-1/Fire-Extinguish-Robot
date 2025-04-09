# 🔥 **Fire Extinguish Robot**  
### *(Autonomous + Manual Bluetooth PC Control)*  

---

## 📦 Version Information  
- **Version:** v1.0.0  
- **Version Last Updated:** April 10, 2025  
---

## 🧠 Project Overview  

The **Fire Extinguish Robot** is an advanced, dual-mode robotic system designed for autonomous firefighting and manual control via Bluetooth. Built on the **ESP32 platform**, the robot leverages directional fire sensing, ultrasonic obstacle detection, and real-time Bluetooth communication with a custom desktop interface. 

With its intelligent navigation, scanning, and extinguishing capabilities, this system serves as a powerful educational and experimental platform for robotics, IoT, and embedded systems.

---

## 🧰 Software Libraries  

- Arduino Core  
- ESP32Servo  
- NewPing  
- BluetoothSerial  

---

## 🛠️ Components  

- ESP32 Development Board  
- Ultrasonic Sensors (×3)  
- IR Flame Sensors (×3)  
- Submersible Water Pump  
- SG90 Servo Motor  
- L298N Motor Driver  
- 12V DC Motors with Wheels (×4)  
- 3.3V Lithium-ion Batteries (×4)  
- Jumper Wires and Breadboard  

---

## 📌 Pin Configuration  

**Ultrasonic Sensors:**  
- Front Sensor: TRIG → D18, ECHO → D19  
- Left Sensor: TRIG → D15, ECHO → D4  
- Right Sensor: TRIG → D23, ECHO → D22  

**Flame Sensors:**  
- Left Flame Sensor:  AO → D25  
- Right Flame Sensor: AO → D33  
- Front Flame Sensor: AO → D32  

**Water Pump:**  
- Control Pin → D2  

**Servo Motor:**  
- Control Pin → D13  

**Motor Driver (L298N):**  
- IN1 → D12  
- IN2 → D14  
- IN3 → D27  
- IN4 → D26  

---

## 🔧 Bluetooth Manual Control Setup  

To enable manual control via PC:  

1. Power on the ESP32 robot.  
2. Open Bluetooth settings on your Windows PC.  
3. Select “Add Bluetooth or other device” → Choose “Bluetooth”.  
4. Pair with `ESP32_Car`.  
5. Navigate to “More Bluetooth Options” → COM Ports tab.  
6. Note the **Outgoing COM port** for `ESP32_Car`.

📸 *App Screenshot Placeholder – Bluetooth COM Port Configuration*

---

## 💻 PC Application Connection  

1. Open the **Fire Extinguish Robot Controller App** on your PC.  
2. Navigate to Settings or “Scene”.  
3. Select the COM port assigned to `ESP32_Car`.  
4. Click **Connect** to begin manual control.

📸 *App Screenshot Placeholder – Controller UI and COM Port Selection*

---

## ⚙️ Key Features  

- Autonomous navigation and obstacle avoidance  
- Multi-directional fire detection with servo-based scanning  
- Automatic water pump activation  
- Manual override via Bluetooth-connected PC application  
- Intelligent decision-making for real-time fire suppression  
- Automatic fallback to autonomous mode when Bluetooth is idle  

---

## 🖼️ Circuit Design  

📸 *Circuit Diagram Placeholder – Include schematic showing full wiring layout*

---

## 🎬 Live Demonstration  

📸 *Demo Screenshot or Video Link Placeholder – Showcase robot in action*

---

## 🚀 Planned Enhancements  

- Integration of temperature-based fire verification  
- Android application for mobile control  
- Wi-Fi-based telemetry and remote access  
- Real-time battery voltage monitoring  

---

## 📬 Contact & Contributions  

For questions, collaboration, or contributions:  
- 📧 Email: [yaxpatel6300@gmail.com](mailto:yaxpatel6300@gmail.com)  
- 💻 GitHub: [Syntax-Surfer-1](https://github.com/Syntax-Surfer-1)

---

Feel free to fork the repository and contribute to the project. Contributions and feedback are highly appreciated! 🌟🌟🌟