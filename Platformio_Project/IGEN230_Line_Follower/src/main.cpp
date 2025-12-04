#include <Arduino.h>

// ================== USER CONFIG ==================

// Number of line sensors
const int NUM_SENSORS = 5;

// ADC pins for QRD1114 sensors (ESP32 ADC-capable pins)
const int sensorPins[NUM_SENSORS] = {32, 33, 34, 35, 36};

// Sensor positions (for weighted average), centered at 0
// Example: 5 sensors -> -2, -1, 0, 1, 2
const float sensorPositions[NUM_SENSORS] = {-2, -1, 0, 1, 2};

// Motor driver pins (example for L298N/L293D)
const int ENA = 25;  // PWM for left motor
const int IN1 = 26;  // Left motor direction 1
const int IN2 = 27;  // Left motor direction 2

const int ENB = 14;  // PWM for right motor
const int IN3 = 12;  // Right motor direction 1
const int IN4 = 13;  // Right motor direction 2

// PWM configuration for ESP32 LEDC
const int PWM_FREQ       = 20000;    // 20 kHz
const int PWM_RESOLUTION = 8;        // 8-bit (0-255)
const int PWM_CHANNEL_A  = 0;        // Left motor channel
const int PWM_CHANNEL_B  = 1;        // Right motor channel

// Line-following parameters
const int   BASE_SPEED  = 170;    // Nominal PWM for both motors (0-255)
const int   MAX_SPEED   = 255;    // Max PWM
const float SETPOINT    = 0.0f;   // Desired line position (center)

// PID gains (start with this, then tune)
float Kp = 45.0f;
float Ki = 0.0f;
float Kd = 18.0f;

// Sensor reading behavior:
// If your line is darker than background, set this true to invert ADC readings
// (because dark => low ADC, but we want "stronger" value on the line)
const bool INVERT_SENSOR = true;

// Minimum total sensor signal required to consider line detected
const int MIN_TOTAL_SIGNAL = 1000;

// Control loop period (ms)
const int CONTROL_PERIOD_MS = 10;  // 100 Hz

// ================== SHARED STATE ==================

volatile int commandedLeftSpeed  = 0;  // 0-255
volatile int commandedRightSpeed = 0;  // 0-255

// Simple spinlock for shared data (optional; for this case int writes are atomic,
// but we include it for clarity)
portMUX_TYPE motorMux = portMUX_INITIALIZER_UNLOCKED;

// ================== SENSOR / PID HELPERS ==================

float readLinePosition()
{
    long weightedSum = 0;
    long total = 0;

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        int raw = analogRead(sensorPins[i]);  // 0-4095

        int value = INVERT_SENSOR ? (4095 - raw) : raw;

        if (value < 0)   value = 0;
        if (value > 4095) value = 4095;

        weightedSum += (long)(value) * (long)(sensorPositions[i] * 1000.0f);
        total       += value;
    }

    if (total < MIN_TOTAL_SIGNAL)
    {
        // Line lost; return a large error (you may change strategy here)
        return 0.0f;  // or last known position; here we just return center
    }

    float position = (float)weightedSum / (float)total;
    position /= 1000.0f;

    return position;
}

float computePID(float error, float dt)
{
    static float integral = 0.0f;
    static float lastError = 0.0f;
    static bool firstRun = true;

    if (firstRun)
    {
        lastError = error;
        firstRun = false;
    }

    integral += error * dt;

    float derivative = 0.0f;
    if (dt > 0.0f)
    {
        derivative = (error - lastError) / dt;
    }

    float output = Kp * error + Ki * integral + Kd * derivative;

    lastError = error;
    return output;
}

int clampSpeed(int val)
{
    if (val < 0)   val = 0;
    if (val > MAX_SPEED) val = MAX_SPEED;
    return val;
}

// ================== MOTOR DRIVER ==================

void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
    // Direction: here we assume forward motion only, with speed >= 0.
    // If you want reverse, sign of speed can encode direction.
    bool leftForward  = true;
    bool rightForward = true;

    if (leftSpeed < 0)
    {
        leftForward = false;
        leftSpeed   = -leftSpeed;
    }
    if (rightSpeed < 0)
    {
        rightForward = false;
        rightSpeed   = -rightSpeed;
    }

    leftSpeed  = clampSpeed(leftSpeed);
    rightSpeed = clampSpeed(rightSpeed);

    // Left motor direction
    digitalWrite(IN1, leftForward ? HIGH : LOW);
    digitalWrite(IN2, leftForward ? LOW  : HIGH);

    // Right motor direction
    digitalWrite(IN3, rightForward ? HIGH : LOW);
    digitalWrite(IN4, rightForward ? LOW  : HIGH);

    // PWM
    ledcWrite(PWM_CHANNEL_A, leftSpeed);
    ledcWrite(PWM_CHANNEL_B, rightSpeed);
}

// ================== TASKS (MULTICORE) ==================

// Task 1: Sensor + PID, pinned to core 0
void controlTask(void *param)
{
    uint32_t lastTime = millis();

    for (;;)
    {
        uint32_t now = millis();
        uint32_t dt_ms = now - lastTime;
        if (dt_ms == 0) dt_ms = CONTROL_PERIOD_MS;
        lastTime = now;

        float dt = dt_ms / 1000.0f;

        // 1) Read line position
        float position = readLinePosition();

        // 2) Compute error (desired - measured)
        float error = SETPOINT - position;

        // 3) PID output (steering correction)
        float correction = computePID(error, dt);

        // Map correction to motor speeds
        int left  = (int)(BASE_SPEED - correction);
        int right = (int)(BASE_SPEED + correction);

        left  = clampSpeed(left);
        right = clampSpeed(right);

        // 4) Update shared speeds
        portENTER_CRITICAL(&motorMux);
        commandedLeftSpeed  = left;
        commandedRightSpeed = right;
        portEXIT_CRITICAL(&motorMux);

        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

// Task 2: Motor update + optional debugging, pinned to core 1
void motorTask(void *param)
{
    int localLeft = 0;
    int localRight = 0;

    for (;;)
    {
        portENTER_CRITICAL(&motorMux);
        localLeft  = commandedLeftSpeed;
        localRight = commandedRightSpeed;
        portEXIT_CRITICAL(&motorMux);

        setMotorSpeeds(localLeft, localRight);

        // Debug output if needed
        // Serial.print("L: "); Serial.print(localLeft);
        // Serial.print("  R: "); Serial.println(localRight);

        vTaskDelay(pdMS_TO_TICKS(5));  // Fast motor refresh
    }
}

// ================== SETUP / LOOP ==================

void setup()
{
    Serial.begin(115200);

    // Sensor pins
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        pinMode(sensorPins[i], INPUT);
    }

    // Motor pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // PWM setup
    ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(ENA, PWM_CHANNEL_A);
    ledcAttachPin(ENB, PWM_CHANNEL_B);

    // Start with motors stopped
    setMotorSpeeds(0, 0);

    // Create tasks on different cores
    xTaskCreatePinnedToCore(
        controlTask,        // Task function
        "ControlTask",      // Name
        4096,               // Stack size
        NULL,               // Parameters
        2,                  // Priority
        NULL,               // Task handle
        0                   // Core 0
    );

    xTaskCreatePinnedToCore(
        motorTask,
        "MotorTask",
        2048,
        NULL,
        1,
        NULL,
        1                   // Core 1
    );
}

void loop()
{
    // Nothing here; all work done in FreeRTOS tasks
}