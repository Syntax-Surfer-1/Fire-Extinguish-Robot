

















# ESP32 Obstacle Avoidance Robot with Bluetooth Manual Control 🎯🎯🎯

## Build Information 🎉🎉🎉

### Build-5 🚀
- **Build Date:** 01-18-2025
- **Build Time:** 7:30 PM

---

### **Code Summary 🛠️**  
This program uses an ESP32 to control a **servo**, three **ultrasonic sensors** (front, left, and right), and an **L298N motor driver** for wheel movement. Additionally, it introduces **Bluetooth control via the ESP32’s built-in Bluetooth**.  

### **Key Features 🚀**  
- **Manual Bluetooth Control** 📱: The ESP32 pairs directly with a mobile phone. After pairing, users can send movement commands (`F`, `B`, `L`, `R`, `S`) from an Android app.  
- **Autonomous Obstacle Avoidance** 🤖: The robot moves forward, using ultrasonic sensors to detect obstacles and make navigation decisions.  
- **Reverse and Scan** 🔄: If an object is detected, the robot reverses, scans left and right using the servo, and chooses the best direction to turn.  
- **Smart Decision-Making** 🧠: Turns left or right based on **distance readings** from the left and right sensors. If both sides are blocked, the robot continues reversing.  

### **Adjustable Parameters 🔧**  
- **OBJECT_THRESHOLD_FRONT, OBJECT_THRESHOLD_LEFT, OBJECT_THRESHOLD_RIGHT**: Distance thresholds (in cm) for detecting objects.  
- **OBJECT_DETECTION_DELAY**: Delay (in seconds) after object detection or scans.  
- **LOOK_ANGLE**: Maximum angle (up to **90°**) for servo movement to the left and right.  
- **SPEED_INCREMENT**: Gradual motor speed increase for **smooth acceleration** and jerk prevention.  

### **How This Works** 🛠️  
1. **Manual Mode (Bluetooth)**: The robot responds to Bluetooth commands (`F`, `B`, `L`, `R`, `S`) from a mobile app.  
2. **Autonomous Mode**: If no Bluetooth input is detected, the robot moves forward while scanning for obstacles.  
3. **Obstacle Handling**:  
   - If an object is detected, it reverses.  
   - The servo scans left and right.  
   - The robot turns toward the clearer path.
   
---

## **🔧 Libraries Used**  
- **ESP32Servo** → Controls the **servo motor**.  
- **NewPing** → For **ultrasonic distance measurement**.  
- **BluetoothSerial** → Enables **built-in ESP32 Bluetooth communication**.  

---

## **📡 Connecting via Bluetooth**  
**Steps to connect the ESP32 to your Android phone:**  
1. **Turn on ESP32** → It automatically enters **Bluetooth pairing mode**.  
2. **Go to your phone’s Bluetooth settings** → Find **"ESP32_ROBOT"** and pair.  
3. **Open the Android app** → Click **"Connect"** to establish a connection.  
4. **Start sending commands (F, B, L, R, S) to move the robot!**  

---

## **📲 Android App Controls**  
- **The app includes:**  
  ✅ **Buttons for Forward, Backward, Left, Right, and Stop**  
  ✅ **Real-time Bluetooth connection status**  
  ✅ **Auto-reconnect feature**  

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

## **🚀 Key Enhancements from Build-4**  
✅ **No external Bluetooth module needed (Uses ESP32 built-in Bluetooth)**  
✅ **Easier pairing with Android (Just select "ESP32_ROBOT")**  
✅ **Automatic mode switching between manual & autonomous**  
✅ **Refactored servo scanning for better precision**  
✅ **General code cleanup for better performance**  

---

Now, you just **turn on ESP32**, **pair via Bluetooth**, and **control the robot using the Android app**—**no extra setup required!** 🚀🔥

---

## Contact Information ✉️

For further information or collaboration, please reach out via:

- **Email:** [yaxpatel6300@gmail.com](mailto:yaxpatel6300@gmail.com)
- **GitHub:** [Syntax-Surfer-1](https://github.com/Syntax-Surfer-1) 🌐

---

Feel free to fork the repository and contribute to the project. Contributions and feedback are highly appreciated! 🌟🌟🌟
