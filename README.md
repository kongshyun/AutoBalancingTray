# Balancing Tray System 🚀  

This project is an Arduino-based balancing tray system designed to maintain a stable horizontal position using stepper motors and an MPU-6050 sensor. It implements advanced control techniques like Kalman filtering and PID control for precise operation.

This project was a **team project** conducted during a **university course on Automation Systems** in the third year of college.  

- **Team Members**: 4  
  - **Design**: 2 members  
  - **Control**: 2 members  
  
## Core Explanation

The core technology of this project is **PID Control**, which combines **Proportional (P)**, **Integral (I)**, and **Derivative (D)** components to reach the target value efficiently and accurately.  

### Process Overview:

1. **Reading IMU Sensor Data**:
- 현재의 가속도와 각속도 데이터를 읽어옴.

2. **Applying Kalman Filter**:  
- IMU 데이터를 바탕으로 현재의 각도를 추정함.
칼만 필터는 예측 정확도를 높이기 위해 사용하는 알고리즘.

3. **PID Control**:  
- 칼만 필터로 계산된 각도를 기준으로 모터를 제어해 목표 각도에 도달하도록 피드백 제어를 수행.
이 과정에서 IMU와 칼만 필터를 통해 데이터를 처리하고, PID 제어를 사용해 시스템의 정확성과 안정성을 유지한다.



## 1. Key Features  

- **Kalman Filter Implementation**:  
  Reduces noise from the MPU-6050 sensor and provides accurate orientation data.  

- **PID Control**:  
  Ensures smooth and stable motor adjustments by calculating proportional, integral, and derivative errors.  

- **Dynamic Calibration**:  
  Automatically calibrates the tray's starting position to improve accuracy during operation.  

- **Stepper Motor Integration**:  
  Three stepper motors drive the tray's tilt based on real-time feedback.  

- **MPU-6050 Sensor Usage**:  
  Provides real-time accelerometer and gyroscope data for balancing computations.  

---

## 2. Hardware  

### Development Board  
- **Arduino Uno**

### Motors  
- **Stepper Motor: NK244-01AT-09 **  
  - **Step Angle**: 0.9°  
  - **Drive Mode**: 2-phase, 4-wire bipolar  
  - **Voltage**: 24V  
  - **Current**: 0.5A  
  - **Resistance**: 25Ω  
  - **Torque**: 0.27 Nm  
  - **Dimensions**: 42mm x 42mm (diameter)  
  - **Shaft Diameter**: Ø5mm  
### Motor Driver
- **A4988**
### Sensor  
- **MPU-6050**  
  - Provides accelerometer and gyroscope data.
---

## 3. Software Functionalities  

### **Initialization**  
- Sets up the MPU-6050 sensor and stepper motor pins.  
- Initializes the Kalman filter for the X, Y, and Z axes.  

### **Balancing Mode**  
- Processes real-time sensor data to compute tray angle errors.  
- Applies Kalman filtering to smooth accelerometer and gyroscope readings.  

### **PID Control**  
- Computes proportional, integral, and derivative errors to adjust motor speed and direction.  

### **Motor Control**  
- Converts angle errors into stepper motor signals to adjust tray tilt.  

### **Dynamic Calibration**  
- Calibrates the tray's initial position to ensure accurate balance.  

