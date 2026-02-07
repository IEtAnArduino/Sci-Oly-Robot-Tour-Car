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

// accel calibration offsets
float xAccelOffset;
float yAccelOffset;
float zAccelOffset;

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

uint8_t maxSpeed = 200;             // change if neededf
uint8_t defaultStraightSpeed = 160; // change if needed
uint8_t defaultTurnSpeed = 100;     // change if needed
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
xyz_t downVector;

// -------------------------------------------
// May need to implement Kalman filter
// -------------------------------------------

uint8_t allPins[] = {motor1En, motor1Pin1, motor1Pin2,
                     motor2En, motor2Pin1, motor2Pin2};

void setupSensor()
{
  // 1.) Set the accelerometer range
  // lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  // lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  // lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  // lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  // lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  // lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  // lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  // lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  // lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void imuCalibration()
{
  int sampleSize = 100;
  float avg_x_rot = 0;
  float avg_y_rot = 0;
  float avg_z_rot = 0;

  float avg_x_accel = 0;
  float avg_y_accel = 0;
  float avg_z_accel = 0;

  for (int i = 0; i < sampleSize; i++)
  {
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);
    avg_x_rot += gyro.gyro.x;
    avg_y_rot += gyro.gyro.y;
    avg_z_rot += gyro.gyro.z;
    avg_x_accel += accel.acceleration.x;
    avg_y_accel += accel.acceleration.y;
    avg_z_accel += accel.acceleration.z;
    delay(10);
  }

  rollOffset = avg_x_rot / sampleSize;
  pitchOffset = avg_y_rot / sampleSize;
  yawOffset = avg_z_rot / sampleSize;

  xAccelOffset = avg_x_accel / sampleSize;
  yAccelOffset = avg_y_accel / sampleSize;
  zAccelOffset = avg_z_accel / sampleSize;

  downVector = {xAccelOffset, yAccelOffset, zAccelOffset};

  Serial.print("Roll Offset: ");
  Serial.println(rollOffset);
  Serial.print("Pitch Offset: ");
  Serial.println(pitchOffset);
  Serial.print("Yaw Offset: ");
  Serial.println(yawOffset);
  Serial.print("X Accel Offset: ");
  Serial.println(xAccelOffset);
  Serial.print("Y Accel Offset: ");
  Serial.println(yAccelOffset);
  Serial.print("Z Accel Offset: ");
  Serial.println(zAccelOffset);
}

void readIMU()
{
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  accelVector.x = accel.acceleration.x;
  accelVector.y = accel.acceleration.y;
  accelVector.z = accel.acceleration.z;
  gyroVector.x = gyro.gyro.x;
  gyroVector.y = gyro.gyro.y;
  gyroVector.z = gyro.gyro.z;
  magVector.x = mag.magnetic.x;
  magVector.y = mag.magnetic.y;
  magVector.z = mag.magnetic.z;
}

void readIMUAdjusted()
{
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  accelVector.x = accel.acceleration.x - downVector.x;
  accelVector.y = accel.acceleration.y - downVector.y;
  accelVector.z = accel.acceleration.z - downVector.z;
  gyroVector.x = gyro.gyro.x - rollOffset;
  gyroVector.y = gyro.gyro.y - pitchOffset;
  gyroVector.z = gyro.gyro.z - yawOffset;
  magVector.x = mag.magnetic.x - xHardOffset;
  magVector.y = mag.magnetic.y - yHardOffset;
  magVector.z = mag.magnetic.z - zHardOffset;
}

// must be calibrated while stationary to get accurate down vector
float getCurrentHeading()
{
  sensors_event_t accel, mag;
  lsm.getEvent(&accel, &mag, NULL, NULL);

  // downVector = {accel.acceleration.x, accel.acceleration.y, accel.acceleration.z};
  xyz_t downVecNorm = downVector.normalized();

  xyz_t magVec = {mag.magnetic.x, mag.magnetic.y, mag.magnetic.z};

  xyz_t magNorm = magVec.normalized();

  xyz_t eastNorm = downVecNorm.cross(magNorm).normalized();
  xyz_t northNorm = eastNorm.cross(downVecNorm).normalized();

  // set north to be 0 radians
  float heading = -atan2(northNorm.y, northNorm.x);

  return heading;
}

void displayMag(bool raw = false)
{
  if (raw)
  {
    readIMU();
  }
  else
  {
    readIMUAdjusted();
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Mag x: ");
  display.println(magVector.x);
  display.print("Mag y: ");
  display.println(magVector.y);
  display.print("Mag z: ");
  display.println(magVector.z);
  float heading = getCurrentHeading();
  display.print("Heading: ");
  display.println(heading);
  display.display();

  Serial.print("Mag x: ");
  Serial.println(magVector.x);
  Serial.print("Mag y: ");
  Serial.println(magVector.y);
  Serial.print("Mag z: ");
  Serial.println(magVector.z);
  Serial.print("Heading: ");
  Serial.println(heading);
}

void displayGyro(bool raw = false)
{
  if (raw)
  {
    readIMU();
  }
  else
  {
    readIMUAdjusted();
  }
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Gyro x: ");
  display.println(gyroVector.x);
  display.print("Gyro y: ");
  display.println(gyroVector.y);
  display.print("Gyro z: ");
  display.println(gyroVector.z);
  display.display();

  Serial.print("Gyro x: ");
  Serial.println(gyroVector.x);
  Serial.print("Gyro y: ");
  Serial.println(gyroVector.y);
  Serial.print("Gyro z: ");
  Serial.println(gyroVector.z);
}

void displayAccel(bool raw = false)
{
  if (raw)
  {
    readIMU();
  }
  else
  {
    readIMUAdjusted();
  }
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Accel x: ");
  display.println(accelVector.x, 4);
  display.print("Accel y: ");
  display.println(accelVector.y, 4);
  display.print("Accel z: ");
  display.println(accelVector.z, 4);
  display.print("Accel mag: ");
  display.println(accelVector.magnitude(), 4);
  display.display();

  Serial.print("Accel x: ");
  Serial.println(accelVector.x, 4);
  Serial.print("Accel y: ");
  Serial.println(accelVector.y, 4);
  Serial.print("Accel z: ");
  Serial.println(accelVector.z, 4);
  Serial.print("Accel mag: ");
  Serial.println(accelVector.magnitude(), 4);
}

void stop()
{
  for (int i = 0; i < 6; i++)
  {
    digitalWrite(allPins[i], LOW);
  }
  delay(stopTime);
}

void straight(float displacement)
{

  float kp_theta = 250.0;
  float kp_velocity = 1.0;
  // int ki = 30;
  // int kd = 0;

  int timeStep = 5; // milliseconds
  int displayTimer = 0;
  unsigned long currentTime = micros();

  float position = 0.0;                          // in m
  float velocity = 0.0;                          // in m/s
  float positionError = displacement + position; // quirk of negative signs since forward is negative displacement (according to the accelerometer)
  bool forward = (positionError > 0);

  float currentTheta = carTheta;
  float thetaError;

  /*
  // slow ramp up to avoid slipping
  for (int i = 0; i < maxSpeed; i = i + 5)
  {
    analogWrite(motor1En, i);
    analogWrite(motor2En, i);
    delay(5);
  }
  */

  while (abs(position) < abs(displacement))
    if (micros() >= currentTime + timeStep * 1000)
    {
      currentTime = micros();
      readIMUAdjusted();
      carTheta += gyroVector.z * ((float)timeStep / 1000.0);
      thetaError = carTheta - currentTheta;

      // float accelAdjusted = (forward) ? accelVector.x + xAccelOffset : accelVector.x + 0.3;

      readIMU();
      velocity += accelVector.x * ((float)timeStep / 1000.0);
      position += velocity * ((float)timeStep / 1000.0);
      positionError = displacement + position;
      forward = (positionError > 0);
      if (!forward)
      {
        thetaError = -thetaError; // need to flip theta error when going forward since positive theta error means we need to turn right, which means we need more power on the left motor, which means we need a negative theta error for the motor control equation to work out
      }

      motor1Velocity = positionError * (defaultStraightSpeed - kp_theta * thetaError);
      motor2Velocity = positionError * (defaultStraightSpeed + kp_theta * thetaError);
      if (abs(motor1Velocity) > maxSpeed)
      {
        motor1Velocity = maxSpeed;
      }
      if (abs(motor2Velocity) > maxSpeed)
      {
        motor2Velocity = maxSpeed;
      }

      analogWrite(motor1En, abs(motor1Velocity));
      analogWrite(motor2En, abs(motor2Velocity));

      digitalWrite(motor1Pin1, forward);
      digitalWrite(motor1Pin2, !forward);
      digitalWrite(motor2Pin1, forward);
      digitalWrite(motor2Pin2, !forward);

      displayTimer++;
      if (displayTimer >= 50)
      {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Acceleration: ");
        display.println(accelVector.x);
        display.print("Velocity: ");
        display.println(velocity);
        display.print("Position: ");
        display.println(position);
        display.print("Pos Error: ");
        display.println(positionError);
        display.display();
        displayTimer = 0;
      }
    }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Acceleration: ");
  display.println(accelVector.x);
  display.print("Velocity: ");
  display.println(velocity);
  display.print("Position: ");
  display.println(position);
  display.print("Pos Error: ");
  display.println(positionError);
  display.display();
}

// relies on gyroscope for heading, integrating angular velocity
void rotateGyro(float deltaTheta)
{
  // deltaTheta is desired angular displacement from current heading, in radians

  int timeStep = 10; // milliseconds
  int kp = 200;      // oscillates slightly at 600, IMU starts to drift
  int ki = 30;
  // int kd = 0;
  float ANGULAR_CORRECTION = 90.0 / 106.0;

  int displayTimer = 0;
  unsigned long currentTime = millis();
  float thetaSet = carTheta + ANGULAR_CORRECTION * deltaTheta;

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
  /*
  digitalWrite(motor1Pin1, ccw);
  digitalWrite(motor1Pin2, !ccw);
  digitalWrite(motor2Pin1, !ccw);
  digitalWrite(motor2Pin2, ccw);

  // slow ramp up to avoid slipping
  for (int i = 0; i < 255; i = i + 5)
  {
    analogWrite(motor1En, i);
    analogWrite(motor2En, i);
    delay(5);
  }
*/

  // make sure car only stops at set point and is at rest
  while (abs(thetaError) > 0.02 * abs(initialThetaError) || abs(gyroVector.z) > 0.02)
  {

    if (millis() >= currentTime + timeStep)
    {
      currentTime = millis();
      readIMUAdjusted();
      carTheta += gyroVector.z * ((float)timeStep / 1000.0);
      // implementing PI control (no D)
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

      // PI control
      int motorVelocity = thetaError * kp + ki * thetaErrorI; // + kd * thetaErrorD;
      ccw = (thetaError > 0);
      int motorSpeed = abs(motorVelocity);
      if (motorSpeed > defaultTurnSpeed)
      {
        analogWrite(motor1En, defaultTurnSpeed);
        analogWrite(motor2En, defaultTurnSpeed);
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
      if (displayTimer == 20)
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
  displayTimer = 0;
}

// relies on magnetometer for heading
// currently unreliable, possibly due to noisy mag readings and/or hard/soft iron offsets
void rotateMagnet(float deltaTheta)
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
    delay(5);
  }

  // make sure car only stops at set point and is at rest
  while (abs(thetaError) > 0.02 * abs(initialThetaError) || abs(gyroVector.z) > 0.02)
  {

    if (millis() >= currentTime + timeStep)
    {
      currentTime = millis();
      readIMUAdjusted();
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
      if (motorSpeed > defaultTurnSpeed)
      {
        analogWrite(motor1En, defaultTurnSpeed);
        analogWrite(motor2En, defaultTurnSpeed);
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

void executePath()
{
  // example path: square
  straight(0.5);
  stop();
  delay(1000);
  rotateGyro(3.1415 / 2);
  stop();
  delay(1000);

  straight(0.5);
  stop();
  delay(1000);
  rotateGyro(3.1415 / 2);
  stop();
  delay(1000);

  straight(0.5);
  stop();
  delay(1000);
  rotateGyro(3.1415 / 2);
  stop();
  delay(1000);

  straight(0.5);
  stop();
  delay(1000);
  rotateGyro(3.1415 / 2);
  stop();
  delay(1000);
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
  delay(1);
  displayAccel(true);
  delay(1000);

  stop();
  // rotateGyro(3.1415 / 2);
  // stop();
  // delay(1000);

  // rotateGyro(-3.1415 / 2);
  // stop();
  // delay(1000);

  // straight(1.0);
  // stop();
  // delay(1000);

  // straight(0.5);
  // stop();
  // delay(1000);
  // straight(-0.5);
  // stop();

  executePath();
}

void loop()
{
  displayAccel(true);
  delay(500);
}
