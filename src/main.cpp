// Check this link for Seeed XIAO ESP32C6 pinout
//https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/

// L293D with 3.3V logic link
// https://arduino.stackexchange.com/questions/88781/how-do-i-instruct-the-l293d-to-operate-a-motor-at-full-speed-when-using-3-3v-gpi


#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <xyz_type.h>
#include <math.h>

// screen
Adafruit_SSD1306 display = Adafruit_SSD1306();


// IMU i2c
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

float rollOffset;   // around x axis
float pitchOffset;  // around y axis
float yawOffset;    // around z axis
float rollPitchYawVelocity[3];

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

uint8_t default_speed = 150;  //change if needed
int stopTime = 500;
/*
  CALIBRATION!!!
  To calculate the rough speed of the car, run calibration()
  Measure the distance traveled in cm and divide by 3000 ms

  angular speed (speedRadPerMilli) is calculated automatically with center to wheel distance of 5.5 cm
*/

float speedCmPerMilli = 90.0 / 3000.0;
float speedRadPerMilli = (speedCmPerMilli / 5.5);

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


// uint8_t currentSpeed = 0;

uint8_t allPins[] = {motor1En, motor1Pin1, motor1Pin2,
                     motor2En, motor2Pin1, motor2Pin2};



void setupSensor() {
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  // lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  // lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  // lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void readIMU() {
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  rollPitchYawVelocity[0] = (float)gyro.gyro.x - rollOffset;
  rollPitchYawVelocity[1] = (float)gyro.gyro.y - pitchOffset;
  rollPitchYawVelocity[2] = (float)gyro.gyro.z - yawOffset;
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

float getCurrentHeading() {
  sensors_event_t accel, mag;
  lsm.getEvent(&accel, &mag, NULL, NULL);
  xyz_t accelVec = {accel.acceleration.x, accel.acceleration.y, accel.acceleration.z};
  xyz_t magVec = {mag.magnetic.x, mag.magnetic.y, mag.magnetic.z};

  xyz_t accelNorm = accelVec.normalized();
  xyz_t magNorm = magVec.normalized();

  xyz_t eastNorm = accelNorm.cross(magNorm).normalized();
  xyz_t northNorm = eastNorm.cross(accelNorm).normalized();

  float heading = atan2(northNorm.y, northNorm.x);

  return heading;

}

void displayMag() {
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

void displayGyro() {
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

void displayAccel() {
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

void imuCalibration() {
  int counter = 0;
  int counter_limit = 150;
  float avg_x_rot = 0;
  float avg_y_rot = 0;
  float avg_z_rot = 0;


  while (counter < counter_limit) {
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);
    avg_x_rot += gyro.gyro.x;
    avg_y_rot += gyro.gyro.y;
    avg_z_rot += gyro.gyro.z;
    counter++;
    delay(25);
  }

  rollOffset = avg_x_rot / counter_limit;
  pitchOffset = avg_y_rot / counter_limit;
  yawOffset = avg_z_rot / counter_limit;

  Serial.print("Roll Offset: ");
  Serial.println(rollOffset);
  Serial.print("Pitch Offset: ");
  Serial.println(pitchOffset);
  Serial.print("Yaw Offset: ");
  Serial.println(yawOffset);
}



void stop() {
  for (int i = 0; i < 6; i++) {
    digitalWrite(allPins[i], LOW);
  }
  delay(stopTime);
}

// ---------------------------------------------------------------------
// INCOMPLETE! Need to incorporate distance tracking
// ---------------------------------------------------------------------
void straight(float distanceInCm) {
  int timeStep = 10;  //milliseconds
  int runTime = 3000;
  unsigned long startTime = millis();
  unsigned long currentTime = startTime;
  float currentTheta = carTheta;
  int kp = 250;
  float thetaError;
  // int ki = 30;
  // int kd = 0;

  //slow ramp up to avoid slipping
  for (int i = 0; i < 150; i = i + 5) {
    analogWrite(motor1En, i);
    analogWrite(motor2En, i);
    delay(2);
  }

  while (millis() < startTime + runTime) {
    if (millis() >= currentTime + timeStep) {
      currentTime = millis();
      readIMU();
      carTheta += rollPitchYawVelocity[2] * ((float)timeStep / 1000.0);
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
  display.println(rollPitchYawVelocity[2]);
  display.print("Car Theta: ");
  display.println(carTheta);
  display.print("Theta Error: ");
  display.println(thetaError);
  display.display();
}


void rotate(float thetaSet) {
  int timeStep = 10;  //milliseconds
  int kp = 100;       //oscillates slightly at 600, IMU starts to drift
  int ki = 30;
  // int kd = 0;

  int displayTimer = 0;
  unsigned long currentTime = millis();
  float thetaError = thetaSet - carTheta;
  // static float thetaErrorOld = thetaSet - carTheta;
  float thetaErrorI = 0.0;
  // static float thetaErrorD;

  float initialThetaError = thetaError;

  //make sure car only stops at set point and is at rest
  while (thetaError > 0.01 * (initialThetaError) || abs(rollPitchYawVelocity[2]) > 0.01) {

    if (millis() >= currentTime + timeStep) {
      currentTime = millis();
      readIMU();
      carTheta += rollPitchYawVelocity[2] * ((float)timeStep / 1000.0);
      // implementing P control (no ID)
      thetaError = thetaSet - carTheta;
      thetaErrorI += thetaError * ((float)timeStep / 1000.0);
      // thetaErrorD = (thetaError - thetaErrorOld) / ((float)timeStep / 1000.0);
      // thetaErrorOld = thetaError;
      int motorVelocity = thetaError * kp + ki * thetaErrorI;  // + kd * thetaErrorD;
      bool ccw = (motorVelocity > 0);
      int motorSpeed = abs(motorVelocity);
      if (motorSpeed > 255) {
        analogWrite(motor1En, 255);
        analogWrite(motor2En, 255);
      } else {
        analogWrite(motor1En, abs(motorVelocity));
        analogWrite(motor2En, abs(motorVelocity));
      }

      digitalWrite(motor1Pin1, ccw);
      digitalWrite(motor1Pin2, !ccw);
      digitalWrite(motor2Pin1, !ccw);
      digitalWrite(motor2Pin2, ccw);

      displayTimer++;
      if (displayTimer == 100) {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Yaw velocity: ");
        display.println(rollPitchYawVelocity[2]);
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
  display.println(rollPitchYawVelocity[2]);
  display.print("Car Theta: ");
  display.println(carTheta);
  display.print("Theta Error: ");
  display.println(thetaError);
  display.display();
}

void calibrateCar() {
  // Runs motors at 90% power for 3000 milliseconds
  // Measure distance traveled in cm and divide by 3000 to calculate speedCmPerMilli

  digitalWrite(motor1Pin1, true);
  digitalWrite(motor1Pin2, false);

  digitalWrite(motor2Pin1, true);
  digitalWrite(motor2Pin2, false);

  for (int i = 0; i < 150; i = i + 5) {
    analogWrite(motor1En, i);
    analogWrite(motor2En, i);
    delay(2);
  }

  analogWrite(motor1En, default_speed * motor1Correction);
  analogWrite(motor2En, default_speed * motor2Correction);

  delay(3000 - 80);
}




void setup() {
  Serial.begin(9600);


  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.display();


  // sensor setup
  if (!lsm.begin()) {
    Serial.println("Oops ... unable to initialize the LSM9DS0. Check your wiring!");
    while (1)
      ;
  }
  Serial.println("Found LSM9DS0 9DOF");
  Serial.println("");
  Serial.println("");
  Serial.println("Setting up LSM9DS0 9DOF");
  setupSensor();
  delay(1);
  // imuCalibration();

  // pin setup
  for(int i = 0; i < 6; i++) {
    pinMode(allPins[i], OUTPUT);
  }
  

  // stop();
  // calibrateCar();
  // straight(100.0, true);
  // stop();
  // rotate(3.1415 * 2.000, true);
  // stop();
}

void loop() {

  float heading = getCurrentHeading();
  Serial.print("Heading (radians): ");
  Serial.println(heading);
  displayMag();
  delay(100);
}
