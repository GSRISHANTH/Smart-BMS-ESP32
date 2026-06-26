# 🔋 Smart Battery Management System (BMS)

> **A Production-Grade Embedded IoT Battery Management System using ESP32, Wokwi Simulator, and Blynk Cloud**

---

## 📖 About the Project

The **Smart Battery Management System (BMS)** is an embedded systems project developed as part of my internship at **ElevanceSkills Technology Private Limited**. The goal of this project was to design and implement a production-style Battery Management System capable of monitoring a simulated **4-cell Lithium-Ion battery pack** while ensuring safety, reliability, and real-time cloud connectivity.

Unlike a basic voltage monitoring system, this project combines battery analytics, intelligent fault detection, runtime fault management, LCD-based diagnostics, cloud telemetry, and a professional monitoring dashboard into a single embedded application. Every subsystem was designed to operate using a **fully non-blocking architecture**, allowing multiple tasks to run simultaneously without affecting system responsiveness.

The complete firmware was developed using **Embedded C++** on the **ESP32 DevKit C v4**, tested using the **Wokwi Simulator**, and connected to the **Blynk IoT Cloud** for remote monitoring and visualization.

---

# 🎯 Internship Objectives

The primary objective of this internship project was to build an industrial-style Battery Management System by integrating six major embedded systems modules into one complete application.

The project focuses on:

* Real-time battery monitoring
* Battery health analysis
* Event-driven protection mechanisms
* Embedded Human Machine Interface
* Fault-tolerant runtime architecture
* Intelligent IoT telemetry
* Cloud-based battery analytics

---

# 🚀 Key Features

## 🔹 Adaptive Multi-Cell Battery Intelligence Engine

The system continuously monitors four simulated lithium-ion battery cells using the ESP32's ADC channels.

Features include:

* Continuous voltage monitoring
* 8-sample ADC averaging
* Cell voltage calculation
* Pack average voltage calculation
* Battery imbalance percentage
* Weakest cell identification
* Strongest cell identification
* Intelligent battery health classification

Battery Health States:

* Healthy
* Minor Imbalance
* Critical Imbalance
* Pack Failure

---

## 🔹 Event-Driven Safety Protection Kernel

The safety subsystem protects the battery pack by detecting abnormal operating conditions and taking immediate protective action.

Protection Events:

* Weak Cell Detection
* Over Voltage Detection
* Sensor Failure Detection
* Rapid Voltage Change Detection

Protection Actions:

* Relay Cutoff
* Buzzer Alert
* LCD Warning Messages
* Automatic Recovery
* Anti Relay Chatter Protection

The complete protection kernel is implemented using **millis()** without using **delay()**, ensuring smooth multitasking throughout the system.

---

## 🔹 Intelligent LCD Human Machine Interface

The project includes a professional LCD-based Human Machine Interface that continuously displays live battery information.

The LCD automatically rotates through multiple diagnostic screens including:

* Individual Cell Voltages
* Battery Pack Statistics
* Safety Status
* Fault Diagnostics

Additional features include:

* Automatic screen rotation
* Flicker-free LCD updates
* Differential display refresh
* Critical fault priority override
* Instant screen refresh whenever battery values change

---

## 🔹 Fault-Tolerant Runtime System

To improve system reliability, a runtime fault management subsystem was implemented.

The firmware continuously checks for abnormal hardware and software conditions and automatically changes the operating mode based on system health.

Operating Modes:

* NORMAL
* DEGRADED
* FAILSAFE
* SHUTDOWN

Fault detection includes:

* Sensor disconnection
* Invalid ADC readings
* Frozen ADC values
* Relay mismatch detection
* Pack failure detection

Faults are recorded using a timestamped ring-buffer log to assist with diagnostics and recovery.

---

## 🔹 Intelligent Cloud Telemetry

Instead of transmitting battery data at fixed intervals, the project uses an **event-driven cloud communication architecture**.

Cloud Features:

* Smart telemetry
* Event-based updates
* Offline event queue
* WiFi auto reconnection
* Queue synchronization
* Signal quality monitoring
* Asynchronous Blynk communication

This reduces unnecessary cloud traffic while ensuring important events are never missed.

---

## 🔹 Executive Battery Intelligence Dashboard

A professional dashboard was developed using the Blynk Cloud platform.

Dashboard features include:

* Live Cell Voltages
* Pack Voltage
* Battery Health Status
* Relay Status
* Protection Status
* Runtime Mode
* Fault Count
* Strongest Cell
* Weakest Cell
* Risk Level
* Historical Voltage Graphs
* WiFi Signal Quality
* Fault History
* Intelligent Operator Recommendations

---

# ⚙️ Hardware Components

* ESP32 DevKit C v4
* Four Slide Potentiometers
* 16×2 I2C LCD Display
* Active LOW Relay Module
* Active Buzzer
* LED Indicator

---

# 💻 Software & Development Tools

* Arduino IDE
* Embedded C++
* ESP32 Arduino Framework
* Wokwi Simulator
* Blynk IoT Cloud
* GitHub

---

# 🔌 GPIO Configuration

| Component     | GPIO   |
| ------------- | ------ |
| Cell 1        | GPIO34 |
| Cell 2        | GPIO35 |
| Cell 3        | GPIO32 |
| Cell 4        | GPIO33 |
| LCD SDA       | GPIO21 |
| LCD SCL       | GPIO22 |
| Relay Module  | GPIO25 |
| Active Buzzer | GPIO26 |

---

# 🔄 System Workflow

1. Read battery voltages from all four cells.
2. Average ADC samples to reduce noise.
3. Calculate battery statistics.
4. Detect faults and abnormal conditions.
5. Update battery health classification.
6. Execute protection logic if required.
7. Refresh the LCD interface.
8. Send important telemetry events to the cloud.
9. Update the Blynk dashboard with live data.

---

# 📂 Repository Contents

```
BMS/
│
├── MAIN_CODE/
│     ASSIGNMENT_MAIN_CODE.ino
│
├── IMAGES/
│     LCD Screens
│     Dashboard
│     Circuit Images
│
├── Srishanth_BMS_Report.pdf
├── WOKWI_LINK.txt
└── README.md
```

---

# 📚 Skills Demonstrated

Through this project, I gained practical experience in:

* Embedded Systems Programming
* ESP32 Development
* ADC Programming
* Event-Driven Firmware Design
* Finite State Machines
* Fault Detection
* Embedded Debugging
* IoT Development
* Blynk Cloud Integration
* LCD Interface Design
* Embedded Software Architecture
* GitHub Project Documentation

---

# 🚀 Future Improvements

Future versions of this project may include:

* Real Lithium-Ion Battery Integration
* Passive and Active Cell Balancing
* Temperature Monitoring
* CAN Bus Communication
* SD Card Data Logging
* OTA Firmware Updates
* Mobile Notifications
* Battery Remaining Useful Life Prediction using Machine Learning

---

# 👨‍💻 Author

**Srishanth G**

B.Tech – Electronics and Communication Engineering

Amrita School of Engineering, Bengaluru

Intern – ElevanceSkills Technology Private Limited

---

# 📄 License

This repository has been created for educational purposes and as part of the ElevanceSkills Internship Program. The source code and documentation are intended for learning, demonstration, and portfolio use.
