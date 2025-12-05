#include <Arduino.h>

const int motorAPWM = 34;
const int motorA1 = 35;
const int motorA2 = 32;
const int motorB1 = 33;
const int motorB2 = 25;
const int motorBPWM = 26;

// sensor:
// trig: D23
// echo: D27

int pwm = 255; // up to 255
float motor_speed_balance = 0.5; // balance between motors A and B (0.0 to 1.0)

const int sensorPins[] = {15, 4, 13, 12, 14};

const int sensor_thresholds[] = {1000, 1000, 1000, 1000, 1000};

int sensor_read_values[5] = {0, 0, 0, 0, 0};

int sensors_activated[5] = {0, 0, 0, 0, 0}; // 0 = black (on line), 1 = white (off line)

const int sensor_read_delay_ms = 1;
const int sensor_averaged_values_count = 1;

void forward();
void backward();
void stopMotors();
void readSensors();
int readSensor(int sensorPin);
void setMotorSpeed();

void characterize_sensors();

void setup() {
  Serial.begin(115200);
  delay(10);

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  pinMode(motorAPWM, OUTPUT);
  pinMode(motorBPWM, OUTPUT);

  pinMode(sensorPins[0], INPUT);
  pinMode(sensorPins[1], INPUT);
  pinMode(sensorPins[2], INPUT);
  pinMode(sensorPins[3], INPUT);
  pinMode(sensorPins[4], INPUT);
}

void loop() {
  readSensors();

  // If 0 active, turn left, if 4 active, turn right, else forward

  if (sensors_activated[0] == 0) {
    // leftmost sensor on line, turn left
    motor_speed_balance = 0.0; // favor motor B
    setMotorSpeed();
    forward();
  } else if (sensors_activated[4] == 0) {
    // rightmost sensor on line, turn right
    motor_speed_balance = 1.0; // favor motor A
    setMotorSpeed();
    forward();
  } else {
    motor_speed_balance = 0.5; // balanced
    setMotorSpeed();
    forward();
  }
}

void characterize_sensors() {
  Serial.println("Characterizing sensors...");
  Serial.println("Place all sensors on WHITE surface.");
  
  // wait 5 seconds
  for (int i = 5; i > 0; i--) {
    Serial.print(i);
    Serial.println("...");
    delay(1000);
  }

  readSensors();
  int white_surface_readings[] = {sensor_read_values[0], sensor_read_values[1], sensor_read_values[2], sensor_read_values[3], sensor_read_values[4]};

  Serial.println("White surface readings:");
  for (int i = 0; i < 5; i++) {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(white_surface_readings[i]);
  }

  Serial.println("Place all sensors on BLACK surface.");
  // wait 5 seconds
  for (int i = 5; i > 0; i--) {
    Serial.print(i);
    Serial.println("...");
    delay(1000);
  }

  readSensors();

  int black_surface_readings[] = {sensor_read_values[0], sensor_read_values[1], sensor_read_values[2], sensor_read_values[3], sensor_read_values[4]};

  Serial.println("Black surface readings:");
  for (int i = 0; i < 5; i++) {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(black_surface_readings[i]);
  }

  // Calculate and print thresholds
  Serial.println("Calculated thresholds:");
  for (int i = 0; i < 5; i++) {
    int threshold = (white_surface_readings[i] + black_surface_readings[i]) / 2;
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(" Threshold: ");
    Serial.println(threshold);
  }

  Serial.println("Characterization complete. Exit Please.");
  
  delay(100000);
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

void readSensors(){
  for (int i = 0; i < 5; i++) {
    sensor_read_values[i] = readSensor(i);

    sensors_activated[i] = 1; // white, default value if above or equal to threshold
    if (sensor_read_values[i] < sensor_thresholds[i]) {
      sensors_activated[i] = 0; // black, if below threshold
    }
  }
}

int readSensor(int sensorIndex) {
  long total = 0;
  int sensorPin = sensorPins[sensorIndex];
  for (int i = 0; i < sensor_averaged_values_count; i++) {
    total += analogRead(sensorPin);
    delay(sensor_read_delay_ms); // small delay between readings
  }
  return total / sensor_averaged_values_count;
}

void setMotorSpeed() {
  int motorAPWMValue = pwm * motor_speed_balance;
  int motorBPWMValue = pwm * (1.0 - motor_speed_balance);

  analogWrite(motorAPWM, motorAPWMValue);
  analogWrite(motorBPWM, motorBPWMValue);
}