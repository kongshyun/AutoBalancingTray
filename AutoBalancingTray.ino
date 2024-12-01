/**
 * Balancing Tray System using Arduino
 * This code controls a balancing tray using stepper motors and an MPU-6050 sensor.
 * Features include Kalman filter-based orientation estimation and PID control.
 */

#include <math.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

/* ==================== MPU-6050 Sensor Configuration ==================== */
#define MPU6050_ACCEL_XOUT_H 0x3B // R
#define MPU6050_PWR_MGMT_1 0x6B // R/W
#define MPU6050_PWR_MGMT_2 0x6C // R/W
#define MPU6050_WHO_AM_I 0x75 // R
#define MPU6050_I2C_ADDRESS 0x68

/* Kalman Filter Structure */
struct GyroKalman{
    /* These variables represent our state matrix x */
    float x_angle, x_bias;
    /* Our error covariance matrix */
    float P_00, P_01, P_10, P_11;
    float Q_angle, Q_gyro;
    float R_angle;
};

/* Kalman Filter Instances for X, Y, Z axes */
struct GyroKalman angX, angY, angZ;

// Kalman filter noise configuration
static const float Q_angle = 0.01;
static const float Q_gyro = 0.04;
static const float R_angle = 0.3;

/* ==================== Global Variables ==================== */
// MPU-6050 calibration
const int lowX = -2150, highX = 2210;
const int lowY = -2150, highY = 2210;
const int lowZ = -2150, highZ = 2550;

unsigned long prevSensoredTime = 0;
unsigned long curSensoredTime = 0;

int xInit[5] = {0}, yInit[5] = {0}, zInit[5] = {0};
int initIndex = 0, initSize = 5;
int xCal = 0, yCal = 0, zCal = 1800;

int initializeIter = 0;
int goalX = 0, goalY = 0;
int prevErrorX = 0, prevErrorY = 0;

/* PID Control Variables */
float P_controlX = 0, I_controlX = 0, D_controlX = 0;
float P_controlY = 0, I_controlY = 0, D_controlY = 0;
float prevPIDTime = 0;
float K_P = 0.8, K_I = 0, K_D = 0;
float scale = 5;

/* Stepper Motor Pins */
const int dirPin1 = 8, stepPin1 = 9;
const int dirPin2 = 4, stepPin2 = 5;
const int dirPin3 = 6, stepPin3 = 7;
const unsigned int Serial_communication_speed = 19200;

// State variables
bool isBalenceCalib = false;
bool isBalenceStart = false;

typedef union accel_t_gyro_union
{
    struct
    {
        uint8_t x_accel_h;
        uint8_t x_accel_l;
        uint8_t y_accel_h;
        uint8_t y_accel_l;
        uint8_t z_accel_h;
        uint8_t z_accel_l;
        uint8_t t_h;
        uint8_t t_l;
        uint8_t x_gyro_h;
        uint8_t x_gyro_l;
        uint8_t y_gyro_h;
        uint8_t y_gyro_l;
        uint8_t z_gyro_h;
        uint8_t z_gyro_l;
    } reg;
    struct
    {
        int x_accel;
        int y_accel;
        int z_accel;
        int temperature;
        int x_gyro;
        int y_gyro;
        int z_gyro;
    } value;
};


/* ==================== Setup and Main Loop ==================== */

// Setup function to initialize hardware
void setup()
{
    Serial.begin(Serial_communication_speed);
    pinMode(stepPin1, OUTPUT);
    pinMode(stepPin2, OUTPUT);
    pinMode(stepPin3, OUTPUT);
    pinMode(dirPin1, OUTPUT);
    pinMode(dirPin2, OUTPUT);
    pinMode(dirPin3, OUTPUT);
    Serial.println("Motor Initialize");
    Wire.begin();

    int error;
    uint8_t c;
    initGyroKalman(&angX, Q_angle, Q_gyro, R_angle);
    initGyroKalman(&angY, Q_angle, Q_gyro, R_angle);
    initGyroKalman(&angZ, Q_angle, Q_gyro, R_angle);
    delay(100);
    error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
    Serial.print(F("WHO_AM_I : "));
    Serial.print(c,HEX);
    Serial.print(F(", error = "));
    Serial.println(error,DEC);
    error = MPU6050_read (MPU6050_PWR_MGMT_2, &c, 1);
    Serial.print(F("PWR_MGMT_2 : "));
    Serial.print(c,HEX);
    Serial.print(F(", error = "));
    Serial.println(error,DEC);
    // Clear the 'sleep' bit to start the sensor. MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
    Serial.println("I M U Running");
    // ============== IMU ===============
}

//////////////////////////////////////////////////////

// Main loop controlling the balance mode or setting mode
void loop(){
    if(isBalenceStart){ //isBalenceStart=true이면 밸런싱모드
        BalancingMode(); //밸런싱모드
    }
    else{
        Setting();
    }
}
//////////////////////////////////////////////////////

/*원판이 수평상태가 됐을 때 시리얼모니터에 ‘o’를 입력하면 밸런싱모드로 진입하게하는 함수*/
void Setting(){
    char state;
    if(Serial.available()){
        state=Serial.read();
        if (state=='o'){//o를 입력하면
            Serial.println(state);
            isBalenceStart=true;//true 가 되면 BalencingMode()를 실행할것이다. Serial.println("Horizon!");
        }
    }
}

/*밸런싱모드*/
void BalancingMode(){
    int error;
    double dT;
    accel_t_gyro_union accel_t_gyro;
    curSensoredTime = millis();
    error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
    if(error != 0) {
        Serial.print(F("Read accel, temp and gyro, error = "));
        Serial.println(error,DEC);
    }
    uint8_t swap;
    #define SWAP(x,y) swap = x; x = y; y = swap
    SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
    SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
    SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
    SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
    SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
    SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
    SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);

     //IMU 칼만필터를 통해 원판의 X축, Y축 각 목표값과 현재값 차이(Error)계산후 모터제어
    if(prevSensoredTime > 0) {
        int gx1=0, gy1=0, gz1 = 0;
        float gx2=0, gy2=0, gz2 = 0;
        int loopTime = curSensoredTime - prevSensoredTime;
        gx2 = angleInDegrees(lowX, highX, accel_t_gyro.value.x_gyro);
        gy2 = angleInDegrees(lowY, highY, accel_t_gyro.value.y_gyro);
        gz2 = angleInDegrees(lowZ, highZ, accel_t_gyro.value.z_gyro);
        predict(&angX, gx2, loopTime);
        predict(&angY, gy2, loopTime);
        predict(&angZ, gz2, loopTime);
        gx1 = update(&angX, accel_t_gyro.value.x_accel) / 10;
        gy1 = update(&angY, accel_t_gyro.value.y_accel) / 10;
        gz1 = update(&angZ, accel_t_gyro.value.z_accel) / 10;
        if(initIndex < initSize) {
            xInit[initIndex] = gx1;
            yInit[initIndex] = gy1;
            zInit[initIndex] = gz1;
            if(initIndex == initSize - 1) {
                int sumX = 0; int sumY = 0; int sumZ = 0;
                for(int k=1; k <= initSize; k++) {
                    sumX += xInit[k];
                    sumY += yInit[k];
                    sumZ += zInit[k];
                }
                xCal -= sumX/(initSize -1);
                yCal -= sumY/(initSize -1);
                zCal = (sumZ/(initSize -1) - zCal);
            }
            initIndex++;
        }
        else {
            gx1 += xCal;
            gy1 += yCal;
        }
        if(initializeIter>20){
            if(!isBalenceCalib){
                Serial.println("Calibration Completed, Balancing Mode On");
                isBalenceCalib = true;
                goalX = gx1;
                goalY = gy1;
            }
            int errorX = goalX - gx1;
            int errorY = goalY - gy1;
            BalancingMotor(errorX,-errorY); //계산한 error를 통해 PID control for motors
        }
        else{
            initializeIter++;
            delay(200);
        }
    }
    prevSensoredTime = curSensoredTime;
}

//초기 X축, Y축 제어값
float P_controlX = 0;
float I_controlX = 0;
float D_controlX = 0;

float P_controlY = 0;
float I_controlY = 0;
float D_controlY = 0;

float prevPIDTime =0;

//PID값
float K_P = 0.8;
float K_I = 0;
float K_D = 0.1;

//scale값
float scale = 5;


/*밸런싱모터*/
void BalancingMotor(int _errorX, int _errorY){
    float nowPIDTime = millis();
    float loopPIDTime = nowPIDTime - prevPIDTime;

    P_controlX = K_P * _errorX;
    I_controlX += K_I * _errorX * loopPIDTime;
    D_controlX = K_D * (_errorX - prevErrorX) / loopPIDTime;

    P_controlY = K_P * _errorY;
    I_controlY += K_I * _errorY * loopPIDTime;
    D_controlY = K_D * (_errorY - prevErrorY) / loopPIDTime;

    float PID_ControlX = (P_controlX + I_controlX + D_controlX)/scale;
    float PID_ControlY = (P_controlY + I_controlY + D_controlY)/scale;
    
    /*각 모터에 부여할 각도값 계산 공식*/
    float Motor1 = +PID_ControlY;
    float Motor2 = +PID_ControlX-PID_ControlY;
    float Motor3 = -PID_ControlY-PID_ControlX;

    Serial.print("| errorX : ");
    Serial.print(_errorX);
    Serial.print("| errorY : ");
    Serial.print(_errorY);

    Serial.print(" M1: ");
    Serial.print(Motor1);
    Serial.print(" M2: ");
    Serial.print(Motor2);
    Serial.print(" M3: ");
    Serial.println(Motor3);
    Serial.print("\t \n");
    // MotorCode
    Stepmotor(dirPin1,stepPin1,(double)Motor1);
    Stepmotor(dirPin2,stepPin2,(double)Motor2);
    Stepmotor(dirPin3,stepPin3,(double)Motor3);

    prevErrorX = _errorX;
    prevErrorY = _errorY;
    prevPIDTime = nowPIDTime;
}

/*스텝모터구동*/
void Stepmotor(int dirpin_num, int steppin_num, double error_degree){
    const int one_degree= 20;//코드의 에러 각도의 범위가 0~90 => 0~2000이므로 0.9도 = 20으로 변환. int pulse_number = (abs(error_degree)/one_degree); // 에러각/20 = 펄스 수 -> 펄스*0.9 = 구동 각도
    if(error_degree>3*one_degree){// 오차가 3도 이상이면 모터 구동
        for(int i=0;i<(pulse_number);i++){
            digitalWrite(dirpin_num, HIGH);
            digitalWrite(steppin_num, HIGH);
            delayMicroseconds(2500);
            delayMicroseconds(2500);
            digitalWrite(steppin_num, LOW);
            delayMicroseconds(2500);
            delayMicroseconds(2500);
        }
    }//타겟 각도 0보다 클 때
    if(error_degree<(-3*one_degree)){// 오차가 -3도 이상이면 모터 구동)
        for(int i=0;i<(pulse_number);i++){
            digitalWrite(dirpin_num, LOW);
            digitalWrite(steppin_num, HIGH);
            delayMicroseconds(2500);
            delayMicroseconds(2500);
            digitalWrite(steppin_num, LOW);
            delayMicroseconds(2500);
            delayMicroseconds(2500);
        }
    }//타겟 각도 0보다 클 때
}
////////////////////////////////////////////////////////

// Reads data from MPU-6050
int MPU6050_read(int start, uint8_t *buffer, int size)
{
    int i, n, error;
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    n = Wire.write(start);
    if (n != 1)
        return (-10);
    n = Wire.endTransmission(false); // hold the I2C-bus
    if (n != 0)
        return (n);
    // Third parameter is true: relase I2C-bus after data is read. Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
    i = 0;
    while(Wire.available() && i<size)
    {
        buffer[i++]=Wire.read();
    }
    if ( i != size)
        return (-11);

    return (0); // return : no error 
}

int MPU6050_write(int start, const uint8_t *pData, int size)
{
    int n, error;
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    n = Wire.write(start); // write the start address
    if (n != 1)
        return (-20);
    n = Wire.write(pData, size); // write data bytes
    if (n != size)
        return (-21);
    error = Wire.endTransmission(true); // release the I2C-bus
    if (error != 0)
        return (error);
    return (0); // return : no error 
}

// Writes a value to MPU-6050
int MPU6050_write_reg(int reg, uint8_t data)
{
    int error;
    error = MPU6050_write(reg, &data, 1);
    return (error);
}
float angleInDegrees(int lo, int hi, int measured) {
    float x = (hi - lo)/180.0;
    return (float)measured/x;
}

// Initializes Kalman filter
void initGyroKalman(struct GyroKalman *kalman, const float Q_angle, const float Q_gyro, const float
R_angle) {
    kalman->Q_angle = Q_angle;
    kalman->Q_gyro = Q_gyro;
    kalman->R_angle = R_angle;
    kalman->P_00 = 0;
    kalman->P_01 = 0;
    kalman->P_10 = 0;
    kalman->P_11 = 0;
}

// Kalman predict step
void predict(struct GyroKalman *kalman, float dotAngle, float dt) {
    kalman->x_angle += dt * (dotAngle - kalman->x_bias);
    kalman->P_00 += -1 * dt * (kalman->P_10 + kalman->P_01) + dt*dt * kalman->P_11 +
    kalman->Q_angle;
    kalman->P_01 += -1 * dt * kalman->P_11;
    kalman->P_10 += -1 * dt * kalman->P_11;
    kalman->P_11 += kalman->Q_gyro;
}

// Kalman update step
float update(struct GyroKalman *kalman, float angle_m) {
    const float y = angle_m - kalman->x_angle;
    const float S = kalman->P_00 + kalman->R_angle;
    const float K_0 = kalman->P_00 / S;
    const float K_1 = kalman->P_10 / S;

    kalman->x_angle += K_0 * y;
    kalman->x_bias += K_1 * y;
    kalman->P_00 -= K_0 * kalman->P_00;
    kalman->P_01 -= K_0 * kalman->P_01;
    kalman->P_10 -= K_1 * kalman->P_00;
    kalman->P_11 -= K_1 * kalman->P_01;
    return kalman->x_angle;
}