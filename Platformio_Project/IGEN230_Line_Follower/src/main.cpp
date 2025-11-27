#include <Arduino.h>

const int motorA1 = 12;
const int motorA2 = 13;
const int motorB1 = 14;
const int motorB2 = 27;

const int motorAPWM = 26;
const int motorBPWM = 25;

int pwm = 200;

// order: 13 12 14 27 26 25

const int sensor1 = 15;
const int sensor2 = 4;
const int sensor3 = 35;
const int sensor4 = 34;
const int sensor5 = 39;

const int sensorThreshold = 1000;

void forward();
void backward();
void stopMotors();
int readSensors();

void setup() {
  Serial.begin(115200);
  delay(10);

  // pinMode(motorA1, OUTPUT); //Declaring the pin modes
  // pinMode(motorA2, OUTPUT);
  // pinMode(motorB1, OUTPUT);
  // pinMode(motorB2, OUTPUT);

  // pinMode(motorAPWM, OUTPUT);
  // pinMode(motorBPWM, OUTPUT);

  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);
}

void loop() {
  int sensorValue = analogRead(sensor1);
  Serial.print("Sensor Value 1: ");
  Serial.println(sensorValue);
  sensorValue = analogRead(sensor2);
  Serial.print("Sensor Value 2: ");
  Serial.println(sensorValue);
  sensorValue = analogRead(sensor3);
  Serial.print("Sensor Value 3: ");
  Serial.println(sensorValue);
  delay(500);

  return;

  // while (true) {
  //   pwm += 25;
  //   if (pwm > 255) pwm = 0;

  //   analogWrite(motorAPWM, pwm);
  //   analogWrite(motorBPWM, pwm);

  //   forward();
  //   delay(1000);
  // }

  // Serial.println("Moving Forward");
  // forward();
  // delay(1000);

  // Serial.println("Stopping");
  // stopMotors();
  // delay(500);

  // Serial.println("Moving Backward");
  // backward();
  // delay(1000);

  // Serial.println("Stopping");
  // stopMotors();
  // delay(500);
}

void forward() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

void backward() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}

void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}

// int readSensors(){
//   int sensorValue = analogRead(sensor1);
//   // threshold code is easy
// }