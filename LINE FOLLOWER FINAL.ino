#include <QTRSensors.h>
const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;

int m1Speed = 0;
int m2Speed = 0;

float kp = 15;
float ki = 0;
float kd = 30;

int p = 0;
int i = 0;
int d = 0;

int error = 0;
int lastError = 0;

const int maxSpeed = 255;
const int minSpeed = -255;

const int baseSpeed = 240;

QTRSensors qtr;

const int sensorCount = 6;
int sensorValues[sensorCount];
int sensors[sensorCount] = { 0, 0, 0, 0, 0, 0 };

const int calibrationTime = 8000;

long calibrationInterval = 0;
void setup() {

  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, sensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Callibration
  while (millis() < calibrationTime) {
    qtr.calibrate();
    error = map(qtr.readLineBlack(sensorValues), 0, 5000, -255, 255);
    if (error < -130) {
      //  Going right in case of bigger error

      for (int j = 200; j < 240; j++) {
        setMotorSpeed(j, j);
        error = map(qtr.readLineBlack(sensorValues), 0, 5000, -255, 255);
        if (error > 250) {
          break;
        }
      }
    } else {
      //  Going left in case of smaller error

      for (int j = 200; j < 240; j++) {
        setMotorSpeed(-j, -j);
        error = map(qtr.readLineBlack(sensorValues), 0, 5000, -255, 255);
        if (error < -250) {
          break;
        }
      }
    }
  }

  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(9600);
}

void loop() {

  // Setting pid
  pidControl(kp, ki, kd);

  lineFollower();

  setMotorSpeed(-m1Speed, m2Speed);
}

void lineFollower() {
  //  Calculating the motor speed according to error

  int motorSpeed = kp * p + ki * i + kd * d;

  //  Setting the speed with the base speed
  m1Speed = baseSpeed;
  m2Speed = baseSpeed;

  //  Adding the calcultaed errors to the motor speed
  if (error < 0) {
    m1Speed += motorSpeed;
  } else if (error > 0) {
    m2Speed -= motorSpeed;
  }

  //  Setting the speed limits for the motors
  m1Speed = constrain(m1Speed, -150, maxSpeed);
  m2Speed = constrain(m2Speed, -150, maxSpeed);
}

//  Setting the variables for PID and the error
void pidControl(float kp, float ki, float kd) {
  error = map(qtr.readLineBlack(sensorValues), 0, 5000, -50, 50);

  p = error;
  i = i + error;
  d = error - lastError;
  lastError = error;
}

//  Function for setting motor speed
void setMotorSpeed(int motor1Speed, int motor2Speed) {
  if (motor1Speed == 0) {
    //  Stopping motor 1 in case it's speed is 0

    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, motor1Speed);
  } else {
    //  Motor 1 going back or forward according to speed

    if (motor1Speed > 0) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, motor1Speed);
    }
    if (motor1Speed < 0) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -motor1Speed);
    }
  }

  if (motor2Speed == 0) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, motor2Speed);
  } else {
    if (motor2Speed > 0) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, motor2Speed);
    }
    if (motor2Speed < 0) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -motor2Speed);
    }
  }
}