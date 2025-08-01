# Balancing Tray System 🚀  

This project is an Arduino-based balancing tray system designed to maintain a stable horizontal position using stepper motors and an MPU-6050 sensor. It implements advanced control techniques like Kalman filtering and PID control for precise operation.

### Process Overview:

1. **Reading IMU Sensor Data**:
- 현재의 가속도와 각속도 데이터를 읽어옴.

2. **Applying Kalman Filter**:  
- IMU 데이터를 바탕으로 현재의 각도를 추정함.
칼만 필터는 예측 정확도를 높이기 위해 사용하는 알고리즘.

3. **PID Control**:  
- 칼만 필터로 계산된 각도를 기준으로 모터를 제어해 목표 각도에 도달하도록 피드백 제어를 수행.
이 과정에서 IMU와 칼만 필터를 통해 데이터를 처리하고, PID 제어를 사용해 시스템의 정확성과 안정성을 유지한다.
