#include <Arduino.h>

int motorA1 = 12;
int motorA2 = 13;
int motorB1 = 14;
int motorB2 = 27;

void forward();
void backward();
void stopMotors();

void setup() {
  Serial.begin(115200);
  delay(10);

  pinMode(motorA1, OUTPUT); //Declaring the pin modes
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
}

void loop() {
  Serial.println("Moving Forward");
  forward();
  delay(1000);

  Serial.println("Stopping");
  stopMotors();
  delay(500);

  Serial.println("Moving Backward");
  backward();
  delay(1000);

  Serial.println("Stopping");
  stopMotors();
  delay(500);
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