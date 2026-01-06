#include <Arduino.h>

const int motorAPWM = 21; // Right
const int motorA1 = 32;
const int motorA2 = 19;
const int motorB1 = 33; // Left
const int motorB2 = 25;
const int motorBPWM = 26;

// sensor:
// trig: D23
// echo: D27

int pwm = 150; // up to 255

// Custom tuned to go in a straight line
float motor_A_multiplier = 0.90; // Left
float motor_B_multiplier = 1.0; // Right

const int sensorPins[] = {15, 4, 13, 14, 27};

// If pin 12 is occupied, flashing fails

const int sensor_thresholds[] = {1000, 1000, 1000, 1000, 1000};

int sensor_read_values[5] = {0, 0, 0, 0, 0};

int sensors_activated[5] = {0, 0, 0, 0, 0}; // 0 = black (on line), 1 = white (off line)

const int sensor_read_delay_ms = 1;
const int sensor_averaged_values_count = 1;

void forward();
void backward();
void turn(int direction); // -1 = left, 1 = right

void stopMotors();
void lerpPWM(int ms, int start_pwm, int target_pwm);

void setMotorSpeed();

void readSensors();
int readSensor(int sensorPin);

void printSensorsActivated();
void printSensorValues();

void characterize_sensors();

void algorithm1();
void algorithm2();
void algorithm3();

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
  // forward();
  // pwm = 100;
  // return;

  algorithm2();
}

void algorithm2() {
  lerpPWM(50, pwm, 0); // accelerate to 200 pwm in 200 ms

  readSensors();
  printSensorsActivated();

  if (sensors_activated[0] == 1) {
    // leftmost sensor on line, turn left
    turn(-1);
    //forward();
  } else if (sensors_activated[4] == 1) {
    // rightmost sensor on line, turn right
    turn(1);
    //forward();
  } else {
    forward();
  }

  delay(50);

  lerpPWM(50, pwm, 200); // accelerate to 200 pwm in 200 ms
}

void algorithm1() {
  readSensors();

  printSensorsActivated();

  // If 0 active, turn left, if 4 active, turn right, else forward

  if (sensors_activated[0] == 1) {
    // leftmost sensor on line, turn left
    turn(-1);
    //forward();
  } else if (sensors_activated[4] == 1) {
    // rightmost sensor on line, turn right
    turn(1);
    //forward();
  } else {
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

void printSensorsActivated() {
  // print all sensors_activated
  Serial.print("Sensors Activated: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(sensors_activated[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void printSensorValues() {
  // print all sensor values
  Serial.print("Sensor Values: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(sensor_read_values[i]);
    Serial.print(" ");
  }
  Serial.println();
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
  int motorAPWMValue = pwm * motor_A_multiplier; // ;pwm * motor_speed_balance;
  int motorBPWMValue = pwm * motor_B_multiplier; // pwm * (1.0 - motor_speed_balance);

  analogWrite(motorAPWM, motorAPWMValue);
  analogWrite(motorBPWM, motorBPWMValue);
}

void lerpPWM(int ms, int start_pwm, int target_pwm) {
  int steps = 10;
  int delay_per_step = ms / steps;
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / (float)steps;
    int current_pwm = start_pwm + (target_pwm - start_pwm) * t;
    pwm = current_pwm;
    setMotorSpeed();
    delay(delay_per_step);
  }
}