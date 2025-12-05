#include <Arduino.h>

const int motorAPWM = 19; // Right
const int motorA1 = 32;
const int motorA2 = 21;
const int motorB1 = 33; // Left
const int motorB2 = 25;
const int motorBPWM = 26;

// sensor:
// trig: D23
// echo: D27

int pwm = 255; // up to 255

const int sensorPins[] = {15, 4, 13, 12, 14};

const int sensor_thresholds[] = {1000, 1000, 1000, 1000, 1000};

int sensor_read_values[5] = {0, 0, 0, 0, 0};

int sensors_activated[5] = {0, 0, 0, 0, 0}; // 0 = black (on line), 1 = white (off line)

const int sensor_read_delay_ms = 1;
const int sensor_averaged_values_count = 1;

void forward();
void backward();
void turn(int direction); // -1 = left, 1 = right
void stopMotors();
void readSensors();
int readSensor(int sensorPin);
void setMotorSpeed();

void characterize_sensors();

void setup() {
  Serial.begin(115200);
  delay(10);

  pinMode(motorAPWM, OUTPUT);
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorBPWM, OUTPUT);

  // Initialize motor speeds
  setMotorSpeed();

  pinMode(sensorPins[0], INPUT);
  pinMode(sensorPins[1], INPUT);
  pinMode(sensorPins[2], INPUT);
  pinMode(sensorPins[3], INPUT);
  pinMode(sensorPins[4], INPUT);
  //characterize_sensors();
}

void loop() {
  // forward 1 s, left 1 s, backward 1 s, right 1 s, stop 1 s

  // int delay_ms = 2000;
  // forward();
  // delay(delay_ms);
  // turn(-1);
  // delay(delay_ms);
  // backward();
  // delay(delay_ms);
  // turn(1);
  // delay(delay_ms);
  // stopMotors();
  // delay(delay_ms);

  // read sensors and print all of them in a line eg. "1200 1300 1400 1500 1600"
  
  readSensors();
  Serial.print("Sensors: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(sensor_read_values[i]);
    Serial.print(" ");
  }
  Serial.println();

  delay(500);

  //motor_speed_balance = 0.5; // balanced
  // setMotorSpeed();
  // forward();
  // sleep(1000);
  // return;

  // readSensors();

  // If 0 active, turn left, if 4 active, turn right, else forward

  // if (sensors_activated[0] == 0) {
  //   // leftmost sensor on line, turn left
  //   turn(-1);
  //   forward();
  // } else if (sensors_activated[4] == 0) {
  //   // rightmost sensor on line, turn right
  //   turn(1);
  //   forward();
  // } else {
  //   forward();
  // }
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

void turn(int direction) {
  if (direction < 0) {
    // turn left
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
  } else {
    // turn right
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
  }
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
  int motorAPWMValue = pwm; // ;pwm * motor_speed_balance;
  int motorBPWMValue = pwm; // pwm * (1.0 - motor_speed_balance);

  analogWrite(motorAPWM, motorAPWMValue);
  analogWrite(motorBPWM, motorBPWMValue);
}