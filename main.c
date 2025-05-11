#include <lpc214x.h>
#include <stdint.h>
#include <math.h>

// MPU6050 I2C Address
#define MPU6050_ADDR 0x68

// Sensitivity for cursor movement
#define SENSITIVITY 1.5f

// Kalman filter parameters
#define PROCESS_NOISE 0.01f
#define MEASUREMENT_NOISE 0.1f

// Button pin (P0.15 in this case)
#define BUTTON_PIN (1 << 15)

// Function prototypes
void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Write(uint8_t data);
uint8_t I2C_Read(uint8_t ack);
void MPU6050_WriteReg(uint8_t reg, uint8_t data);
uint8_t MPU6050_ReadReg(uint8_t reg);
void MPU6050_Init(void);
void MPU6050_GetAcceleration(int16_t *ax, int16_t *ay, int16_t *az);
void USB_Init(void);
void USB_SendMouseReport(int8_t dx, int8_t dy, uint8_t buttons);
void DelayMs(uint32_t ms);
float KalmanFilter(float measurement, float *estimate, float *error);

// Global variables
float ay_offset = 0.0f;
float roll_est = 0, roll_err = 1;
float pitch_est = 0, pitch_err = 1;
uint8_t lastButtonState = 1;

int main(void) {
    // Initialize systems
    I2C_Init();
    MPU6050_Init();
    USB_Init();
    
    // Configure button pin as input with pull-up
    IODIR0 &= ~BUTTON_PIN;  // Set as input
    IOSET0 = BUTTON_PIN;    // Enable pull-up
    
    // Calibration routine
    int16_t ax, ay, az;
    const int numSamples = 100;
    
    for(int i = 0; i < numSamples; i++) {
        MPU6050_GetAcceleration(&ax, &ay, &az);
        ay_offset += ay / 16384.0f;
        DelayMs(10);
    }
    ay_offset /= numSamples;
    
    while(1) {
        // Read accelerometer
        MPU6050_GetAcceleration(&ax, &ay, &az);
        
        float axx = ax / 16384.0f;
        float ayy = (ay / 16384.0f) - ay_offset;
        float azz = az / 16384.0f;
        
        // Calculate angles
        float roll_meas = atan2(ayy, sqrt(axx * axx + azz * azz)) * 180.0f / M_PI;
        float pitch_meas = atan2(-axx, sqrt(ayy * ayy + azz * azz)) * 180.0f / M_PI;
        
        // Apply Kalman filter
        float roll = KalmanFilter(roll_meas, &roll_est, &roll_err);
        float pitch = KalmanFilter(pitch_meas, &pitch_est, &pitch_err);
        
        // Calculate movement
        int8_t deltaY = (int8_t)(roll * SENSITIVITY);
        int8_t deltaX = (int8_t)(pitch * SENSITIVITY);
        
        // Thresholding
        if(abs(roll) < 5.0f) deltaY = 0;
        if(abs(pitch) < 5.0f) deltaX = 0;
        
        // Read button state
        uint8_t currentButtonState = (IOPIN0 & BUTTON_PIN) ? 1 : 0;
        uint8_t buttonClick = (lastButtonState == 1 && currentButtonState == 0) ? 1 : 0;
        lastButtonState = currentButtonState;
        
        // Send USB HID report
        USB_SendMouseReport(deltaX, -deltaY, buttonClick);
        
        DelayMs(20);
    }
    
    return 0;
}

// I2C Initialization
void I2C_Init(void) {
    PINSEL0 |= (1 << 4) | (1 << 6);  // Enable I2C on P0.2 (SCL) and P0.3 (SDA)
    I2C0CONSET = (1 << 6);           // Enable I2C interface
    I2C0SCLL = 0x32;                 // Set clock rate (adjust as needed)
    I2C0SCLH = 0x32;
}

// MPU6050 Initialization
void MPU6050_Init(void) {
    MPU6050_WriteReg(0x6B, 0x00);  // Wake up MPU6050
    MPU6050_WriteReg(0x1B, 0x00);  // Set gyro full scale range
    MPU6050_WriteReg(0x1C, 0x00);  // Set accelerometer full scale range
}

// Read acceleration data from MPU6050
void MPU6050_GetAcceleration(int16_t *ax, int16_t *ay, int16_t *az) {
    *ax = (int16_t)((MPU6050_ReadReg(0x3B) << 8 | MPU6050_ReadReg(0x3C));
    *ay = (int16_t)((MPU6050_ReadReg(0x3D) << 8 | MPU6050_ReadReg(0x3E));
    *az = (int16_t)((MPU6050_ReadReg(0x3F) << 8 | MPU6050_ReadReg(0x40));
}

// USB HID Mouse Implementation
void USB_Init(void) {
    // This is a simplified USB initialization
    // Actual implementation would require proper USB stack configuration
    // including endpoint setup, descriptor tables, etc.
    PINSEL1 |= (1 << 18) | (1 << 20);  // Configure USB pins
    // Additional USB controller initialization would go here
}

void USB_SendMouseReport(int8_t dx, int8_t dy, uint8_t buttons) {
    // This would send a USB HID mouse report
    // Implementation depends on your USB stack
    uint8_t report[4] = {buttons, dx, dy, 0};
    // Actual USB transmission code would go here
}

// Kalman Filter Implementation
float KalmanFilter(float measurement, float *estimate, float *error) {
    // Prediction update
    *error += PROCESS_NOISE;
    
    // Measurement update
    float kalman_gain = *error / (*error + MEASUREMENT_NOISE);
    *estimate += kalman_gain * (measurement - *estimate);
    *error *= (1 - kalman_gain);
    
    return *estimate;
}

// Helper functions for I2C
void I2C_Start(void) {
    I2C0CONSET = (1 << 5);  // Set STA bit
    while(!(I2C0CONSET & (1 << 3)));  // Wait for SI
    I2C0CONCLR = (1 << 5) | (1 << 3);  // Clear STA and SI
}

void I2C_Stop(void) {
    I2C0CONSET = (1 << 4);  // Set STO bit
    I2C0CONCLR = (1 << 3);  // Clear SI
}

void I2C_Write(uint8_t data) {
    I2C0DAT = data;
    I2C0CONSET = (1 << 3);  // Set SI
    while(!(I2C0CONSET & (1 << 3)));  // Wait for SI
    I2C0CONCLR = (1 << 3);  // Clear SI
}

uint8_t I2C_Read(uint8_t ack) {
    I2C0CONSET = (1 << 2) | (ack ? 0 : (1 << 2));  // Set AA
    I2C0CONSET = (1 << 3);  // Set SI
    while(!(I2C0CONSET & (1 << 3)));  // Wait for SI
    I2C0CONCLR = (1 << 3);  // Clear SI
    return I2C0DAT;
}

void MPU6050_WriteReg(uint8_t reg, uint8_t data) {
    I2C_Start();
    I2C_Write(MPU6050_ADDR << 1);
    I2C_Write(reg);
    I2C_Write(data);
    I2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t reg) {
    I2C_Start();
    I2C_Write(MPU6050_ADDR << 1);
    I2C_Write(reg);
    I2C_Start();
    I2C_Write((MPU6050_ADDR << 1) | 1);
    uint8_t data = I2C_Read(0);
    I2C_Stop();
    return data;
}

void DelayMs(uint32_t ms) {
    for(uint32_t i = 0; i < ms; i++)
        for(uint32_t j = 0; j < 10000; j++);
}
