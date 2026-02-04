// Check this link for Seeed XIAO ESP32C6 pinout
// https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/

// L293D with 3.3V logic link
// https://arduino.stackexchange.com/questions/88781/how-do-i-instruct-the-l293d-to-operate-a-motor-at-full-speed-when-using-3-3v-gpi

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h> // not used in this demo but required!
#include <xyz_type.h>
#include <math.h>

// screen
Adafruit_SSD1306 display = Adafruit_SSD1306();

// IMU i2c
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

// gyro calibration offsets
float rollOffset;  // around x axis
float pitchOffset; // around y axis
float yawOffset;   // around z axis

// magnetometer hard iron offsets
float xHardOffset = -12.88; 
float yHardOffset = -5.13;
float zHardOffset = 3.49;

// motor 1 on right, motor 2 on left
uint8_t motor1En = 0;
uint8_t motor1Pin1 = 1;
uint8_t motor1Pin2 = 2;
float motor1Correction = 1.00;

uint8_t motor2En = 21;
uint8_t motor2Pin1 = 16;
uint8_t motor2Pin2 = 17;
float motor2Correction = 1.15;
// goes left 1.1
// goes right 1.2

uint8_t default_speed = 150; // change if needed
uint8_t max_speed = 200;     // change if neededf
int stopTime = 500;

// TO BE IMPLEMENTED IF NEEDED: motor state variables, updated within every move function
// motor velocity: -255 to 255   direction: true for forward, false for backward
int motor1Velocity = 0;
int motor2Velocity = 0;
double carX = 0.0;
double carY = 0.0;
float carVelocity = 0.0;
float carTheta = 0.0;
xyz_t accelVector;
xyz_t magVector;
xyz_t gyroVector;

// -------------------------------------------
// May need to implement Kalman filter
// -------------------------------------------

uint8_t allPins[] = {motor1En, motor1Pin1, motor1Pin2,
                     motor2En, motor2Pin1, motor2Pin2};

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  // lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  // lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  // lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  // lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  // lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  // lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  // lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  // lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  // lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void readIMU()
{
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  accelVector.x = accel.acceleration.x;
  accelVector.y = accel.acceleration.y;
  accelVector.z = accel.acceleration.z;
  gyroVector.x = gyro.gyro.x - rollOffset;
  gyroVector.y = gyro.gyro.y - pitchOffset;
  gyroVector.z = gyro.gyro.z - yawOffset;
  magVector.x = mag.magnetic.x - xHardOffset;
  magVector.y = mag.magnetic.y - yHardOffset;
  magVector.z = mag.magnetic.z - zHardOffset;
}

float getCurrentHeading()
{
  sensors_event_t mag;
  lsm.getEvent(NULL, &mag, NULL, NULL);

  xyz_t downVec = {0, 0, 1};

  xyz_t magVec = {mag.magnetic.x, mag.magnetic.y, mag.magnetic.z};

  xyz_t magNorm = magVec.normalized();

  xyz_t eastNorm = downVec.cross(magNorm).normalized();
  xyz_t northNorm = eastNorm.cross(downVec).normalized();

  // set north to be 0 radians
  float heading = -atan2(northNorm.y, northNorm.x);

  return heading;
}

void displayMag()
{
  display.clearDisplay();
  readIMU();
  display.setCursor(0, 0);
  display.print("Mag x: ");
  display.print(magVector.x);
  display.setCursor(0, 8);
  display.print("Mag y: ");
  display.print(magVector.y);
  display.setCursor(0, 16);
  display.print("Mag z: ");
  display.print(magVector.z);
  display.setCursor(0, 24);
  float heading = getCurrentHeading();
  display.print("Heading: ");
  display.print(heading);
  display.display();
}

void displayGyro()
{
  display.clearDisplay();
  readIMU();
  display.setCursor(0, 0);
  display.print("Gyro x: ");
  display.print(gyroVector.x);
  display.setCursor(0, 8);
  display.print("Gyro y: ");
  display.print(gyroVector.y);
  display.setCursor(0, 16);
  display.print("Gyro z: ");
  display.print(gyroVector.z);
  display.display();
}

void displayAccel()
{
  display.clearDisplay();
  readIMU();
  display.setCursor(0, 0);
  display.print("Accel x: ");
  display.print(accelVector.x);
  display.setCursor(0, 8);
  display.print("Accel y: ");
  display.print(accelVector.y);
  display.setCursor(0, 16);
  display.print("Accel z: ");
  display.print(accelVector.z);
  display.display();
}

void imuCalibration()
{
  int sampleSize = 100;
  float avg_x_rot = 0;
  float avg_y_rot = 0;
  float avg_z_rot = 0;

  for (int i = 0; i < sampleSize; i++)
  {
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);
    avg_x_rot += gyro.gyro.x;
    avg_y_rot += gyro.gyro.y;
    avg_z_rot += gyro.gyro.z;
    delay(10);
  }

  rollOffset = avg_x_rot / sampleSize;
  pitchOffset = avg_y_rot / sampleSize;
  yawOffset = avg_z_rot / sampleSize;

  Serial.print("Roll Offset: ");
  Serial.println(rollOffset);
  Serial.print("Pitch Offset: ");
  Serial.println(pitchOffset);
  Serial.print("Yaw Offset: ");
  Serial.println(yawOffset);
}

void stop()
{
  for (int i = 0; i < 6; i++)
  {
    digitalWrite(allPins[i], LOW);
  }
  delay(stopTime);
}

// ---------------------------------------------------------------------
// INCOMPLETE! Need to incorporate distance tracking
// ---------------------------------------------------------------------
void straight(float distanceInCm)
{
  int timeStep = 10; // milliseconds
  int runTime = 3000;
  unsigned long startTime = millis();
  unsigned long currentTime = startTime;
  float currentTheta = carTheta;
  int kp = 250;
  float thetaError;
  // int ki = 30;
  // int kd = 0;

  // slow ramp up to avoid slipping
  for (int i = 0; i < 255; i = i + 5)
  {
    analogWrite(motor1En, i);
    analogWrite(motor2En, i);
    delay(2);
  }

  while (millis() < startTime + runTime)
  {
    if (millis() >= currentTime + timeStep)
    {
      currentTime = millis();
      readIMU();
      carTheta = getCurrentHeading();
      thetaError = carTheta - currentTheta;

      analogWrite(motor1En, default_speed - kp * thetaError);
      analogWrite(motor2En, default_speed + kp * thetaError);

      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW);
      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW);
    }
  }
  stop();
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Yaw velocity: ");
  display.println(gyroVector.z);
  display.print("Car Theta: ");
  display.println(carTheta);
  display.print("Theta Error: ");
  display.println(thetaError);
  display.display();
}

void rotate(float deltaTheta)
{
  // deltaTheta is desired angular displacement from current heading, in radians

  int timeStep = 10; // milliseconds
  int kp = 200;      // oscillates slightly at 600, IMU starts to drift
  int ki = 30;
  // int kd = 0;

  int displayTimer = 0;
  unsigned long currentTime = millis();
  float thetaSet = carTheta + deltaTheta;

  // wrap thetaSet to be within -pi to pi
  while (thetaSet > 3.1415)
  {
    thetaSet -= 2.0 * 3.1415;
  }

  while (thetaSet < -3.1415)
  {
    thetaSet += 2.0 * 3.1415;
  }

  float thetaError = thetaSet - carTheta;
  // Normalize thetaError to [-pi, pi] for shortest rotation
  while (thetaError > 3.1415)
  {
    thetaError -= 2.0 * 3.1415;
  }
  while (thetaError < -3.1415)
  {
    thetaError += 2.0 * 3.1415;
  }
  // static float thetaErrorOld = thetaSet - carTheta;
  float thetaErrorI = 0.0;
  // static float thetaErrorD;

  float initialThetaError = thetaError;

  bool ccw = (thetaError > 0);
  digitalWrite(motor1Pin1, ccw);
  digitalWrite(motor1Pin2, !ccw);
  digitalWrite(motor2Pin1, !ccw);
  digitalWrite(motor2Pin2, ccw);

  // slow ramp up to avoid slipping
  for (int i = 0; i < 255; i = i + 5)
  {
    analogWrite(motor1En, i);
    analogWrite(motor2En, i);
    delay(2);
  }

  // make sure car only stops at set point and is at rest
  while (abs(thetaError) > 0.01 * abs(initialThetaError) || abs(gyroVector.z) > 0.01)
  {

    if (millis() >= currentTime + timeStep)
    {
      currentTime = millis();
      readIMU();
      carTheta = getCurrentHeading();
      // implementing P control (no ID)
      thetaError = thetaSet - carTheta;
      // Normalize thetaError to [-pi, pi] for shortest rotation
      while (thetaError > 3.1415)
      {
        thetaError -= 2.0 * 3.1415;
      }
      while (thetaError < -3.1415)
      {
        thetaError += 2.0 * 3.1415;
      }
      thetaErrorI += thetaError * ((float)timeStep / 1000.0);
      // thetaErrorD = (thetaError - thetaErrorOld) / ((float)timeStep / 1000.0);
      // thetaErrorOld = thetaError;
      int motorVelocity = thetaError * kp + ki * thetaErrorI; // + kd * thetaErrorD;
      ccw = (thetaError > 0);
      int motorSpeed = abs(motorVelocity);
      if (motorSpeed > max_speed)
      {
        analogWrite(motor1En, max_speed);
        analogWrite(motor2En, max_speed);
      }
      else
      {
        analogWrite(motor1En, abs(motorVelocity));
        analogWrite(motor2En, abs(motorVelocity));
      }

      digitalWrite(motor1Pin1, ccw);
      digitalWrite(motor1Pin2, !ccw);
      digitalWrite(motor2Pin1, !ccw);
      digitalWrite(motor2Pin2, ccw);

      displayTimer++;
      if (displayTimer == 100)
      {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Yaw velocity: ");
        display.println(gyroVector.z);
        display.print("Car Theta: ");
        display.println(carTheta);
        display.print("Theta Error: ");
        display.println(thetaError);
        display.display();
        displayTimer = 0;
      }
    }
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Yaw velocity: ");
  display.println(gyroVector.z);
  display.print("Car Theta: ");
  display.println(carTheta);
  display.print("Theta Error: ");
  display.println(thetaError);
  display.display();
}

void setup()
{
  Serial.begin(9600);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.display();

  // sensor setup
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS0. Check your wiring!");
    while (1)
      ;
  }
  Serial.println("Found LSM9DS0 9DOF");
  Serial.println("");
  Serial.println("");
  Serial.println("Setting up LSM9DS0 9DOF");
  setupSensor();
  delay(1000);
  imuCalibration();

  // pin setup
  for (int i = 0; i < 6; i++)
  {
    pinMode(allPins[i], OUTPUT);
  }

  carTheta = getCurrentHeading();
  Serial.print("Initial Heading (radians): ");
  Serial.println(carTheta);
  displayGyro();
  delay(2000);

  // stop();
  // calibrateCar();
  // straight(100.0, true);
  // stop();
  // rotate(3.1415 * 2.000, true);
  // stop();
}

void loop()
{

  rotate(-3.1415 / 2.0);
  stop();
  delay(1000);
  rotate(3.1415 / 2.0);
  stop();
  delay(1000);
}
