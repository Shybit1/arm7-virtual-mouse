//This is the C code for LPC2148 microcontroller to read MPU6050 data and send mouse movements on COM5 (via Bluetooth (HC-05) OR Serial Port)
#include <lpc214x.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

// MPU6050 I2C Address
#define MPU6050_ADDR 0x68

// Sensitivity for cursor movement
#define SENSITIVITY 1.5f

// Kalman filter parameters
#define PROCESS_NOISE 0.01f
#define MEASUREMENT_NOISE 0.1f

// Button pin (P0.15)
#define BUTTON_PIN (1 << 15)

// UART Configuration
#define UART_BAUD 9600  // HC-05 default baud rate

// Function prototypes
void UART0_Init(void);
void UART0_SendChar(char c);
void UART0_SendString(const char *str);
void I2C_Init(void);
void MPU6050_Init(void);
void MPU6050_GetAcceleration(int16_t *ax, int16_t *ay, int16_t *az);
float KalmanFilter(float measurement, float *estimate, float *error);
void DelayMs(uint32_t ms);

// Global variables
float ay_offset = 0.0f;
float roll_est = 0, roll_err = 1;
float pitch_est = 0, pitch_err = 1;
uint8_t lastButtonState = 1;

int main(void) {
    // Initialize systems
    UART0_Init();
    I2C_Init();
    MPU6050_Init();
    
    // Configure button pin as input with pull-up
    IODIR0 &= ~BUTTON_PIN;  // Set as input
    IOSET0 = BUTTON_PIN;     // Enable pull-up
    
    // Send AT commands to configure HC-05 (optional)
    UART0_SendString("AT+NAME=LPC2148_MOUSE\r\n");
    DelayMs(100);
    UART0_SendString("AT+ROLE=0\r\n");  // Set as slave
    DelayMs(100);
    UART0_SendString("AT+CMODE=1\r\n"); // Connect any address
    DelayMs(100);
    
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
        
        // Send mouse data via Bluetooth
        // Format: "M,X,Y,B\r\n" where X,Y are deltas, B is button state
        UART0_SendString("M,");
        UART0_SendChar(deltaX + '0');  // Simple conversion for demo
        UART0_SendChar(',');
        UART0_SendChar(deltaY + '0');
        UART0_SendChar(',');
        UART0_SendChar(buttonClick + '0');
        UART0_SendString("\r\n");
        
        DelayMs(20);
    }
    
    return 0;
}

// UART Initialization
void UART0_Init(void) {
    PINSEL0 |= (1 << 0) | (1 << 2);  // Enable UART0 on P0.0 (TXD0) and P0.1 (RXD0)
    U0LCR = 0x83;  // 8 bits, no parity, 1 stop bit, DLAB enabled
    U0DLL = (15000000 / (16 * UART_BAUD)) & 0xFF;  // Set baud rate
    U0DLM = ((15000000 / (16 * UART_BAUD)) >> 8) & 0xFF;
    U0LCR = 0x03;  // DLAB disabled
}

void UART0_SendChar(char c) {
    while(!(U0LSR & (1 << 5)));  // Wait for THRE
    U0THR = c;
}

void UART0_SendString(const char *str) {
    while(*str) {
        UART0_SendChar(*str++);
    }
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
