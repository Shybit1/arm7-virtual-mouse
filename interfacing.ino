#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
float angleX = 0, angleY = 0;
float refAngleX = 0, refAngleY = 0;

long gx_bias = 0, gy_bias = 0;

float velocityX = 0, velocityY = 0;

const float alpha = 0.96;
const float sensitivity = 131.0;

unsigned long lastTime;

const int flexPin = A0;
const int irLeft = A1;
const int irRight = A2;
void calibrateGyro() {
  const int samples = 500;
  long sum_gx = 0, sum_gy = 0;
  for (int i = 0; i < samples; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    sum_gx += gx;
    sum_gy += gy;
    delay(2);
  }
  gx_bias = sum_gx / samples;
  gy_bias = sum_gy / samples;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  delay(1000);

  calibrateGyro();
  delay(500);

  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  refAngleX = atan2(ay, az) * 180 / PI;
  refAngleY = atan2(-ax, az) * 180 / PI;

  lastTime = millis();
}
void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
  mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

  // Remove gyro bias
  gx_raw -= gx_bias;
  gy_raw -= gy_bias;

  // Convert acceleration to g
  float ax = (float)ax_raw / 16384.0;
  float ay = (float)ay_raw / 16384.0;
  float az = (float)az_raw / 16384.0;

  // Angle from accel
  float accel_angle_x = atan2(ay, az) * 180 / PI;
  float accel_angle_y = atan2(-ax, az) * 180 / PI;

  // Complementary filter for tilt angle
  angleX = alpha * (angleX + gx_raw * dt / sensitivity) + (1 - alpha) * accel_angle_x;
  angleY = alpha * (angleY + gy_raw * dt / sensitivity) + (1 - alpha) * accel_angle_y;

  // Delta from reference
  float deltaAngleX = angleX - refAngleX;
  float deltaAngleY = angleY - refAngleY;

  // Deadzones to avoid jitter
  if (abs(deltaAngleX) < 0.5) deltaAngleX = 0;
  if (abs(deltaAngleY) < 0.5) deltaAngleY = 0;

  // High-pass filter acceleration to isolate motion (simple deadzone)
  float accelX = (abs(ax) > 0.05) ? ax : 0;
  float accelY = (abs(ay) > 0.05) ? ay : 0;
  // Optional: non-linear scaling
  if (deltaAngleX > 0) deltaAngleX = pow(deltaAngleX, 1.);
  else deltaAngleX = -pow(-deltaAngleX, 1.5);

  if (deltaAngleY > 0) deltaAngleY = pow(deltaAngleY, 1.5);
  else deltaAngleY = -pow(-deltaAngleY, 1.5);
  // Integrate acceleration -> velocity (with decay for drift control)
  velocityX += accelX * dt;
  velocityY += accelY * dt;

  velocityX *= 0.9;  // friction to reduce drift
  velocityY *= 0.9;

  // Gains — tune these to your preference
  float tiltGainX = 0.5;
  float tiltGainY = 0.5;
  float accelGainX = 100.0;
  float accelGainY = 100.0;

  int moveX = (int)(deltaAngleY * tiltGainX + velocityX * accelGainX);
  int moveY = (int)(deltaAngleX * tiltGainY + velocityY * accelGainY);

  // Read flex and IR sensors
  int flexValue = analogRead(flexPin);
  int leftClick = (analogRead(irLeft) < 500) ? 1 : 0;
  int rightClick = (analogRead(irRight) < 500) ? 1 : 0;

  int scroll = 0;
  if (flexValue < 800)
  scroll = 1;         // Scroll Down
else if (flexValue > 900)
  scroll = -1;        // Scroll Up
else
  scroll = 0;

  
  // Send data to PC
  Serial.print(moveX); Serial.print(",");
  Serial.print(moveY); Serial.print(",");
  Serial.print(leftClick); Serial.print(",");
  Serial.print(rightClick); Serial.print(",");
  Serial.println(scroll);

  delay(2);