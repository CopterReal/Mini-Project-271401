
#include <Arduino.h>

// =====================================================
// ESP32-S3 | 4WD Smoke Tracking Robot
// Raspberry Pi 5 + ESP32-S3 + BTS7960
// =====================================================

// =====================================================
// 1. CONFIGURATION
// =====================================================

#define BOARD_LED_PIN 8

// Motor 1
#define M1_RPWM 4
#define M1_LPWM 5

// Motor 2
#define M2_RPWM 6
#define M2_LPWM 7

// Motor 3
#define M3_RPWM 15
#define M3_LPWM 16

// Motor 4
#define M4_RPWM 17
#define M4_LPWM 18

// =====================================================
// 2. ENCODER PINS
// =====================================================

// Motor 1
#define M1_ENA 1
#define M1_ENB 2

// Motor 2
#define M2_ENA 48
#define M2_ENB 47

// Motor 3
#define M3_ENA 9
#define M3_ENB 10

// Motor 4
#define M4_ENA 11
#define M4_ENB 12

// =====================================================
// 3. SYSTEM PARAMETERS
// =====================================================

const unsigned long BAUD_RATE = 115200;

// PWM 0-255
const int BASE_SPEED = 100;

// Check actual encoder counts per wheel revolution.
const float PPR = 1859.0f;

// Telemetry at 10 Hz
const unsigned long RPM_INTERVAL_MS = 100;

// Motor watchdog
const unsigned long COMMAND_TIMEOUT_MS = 500;

// Current motor command
char current_cmd = 'X';

// Watchdog timestamp
unsigned long lastCommandTime = 0;

// RPM timestamp
unsigned long prev_time = 0;

// =====================================================
// 4. ENCODER VARIABLES
// =====================================================

volatile long encoder_count[4] = {
  0, 0, 0, 0
};

long prev_encoder_count[4] = {
  0, 0, 0, 0
};

float rpm[4] = {
  0.0f, 0.0f, 0.0f, 0.0f
};

// =====================================================
// 5. ENCODER INTERRUPTS
// =====================================================

void IRAM_ATTR isr_m1() {
  if (digitalRead(M1_ENB)) {
    encoder_count[0]++;
  } else {
    encoder_count[0]--;
  }
}

void IRAM_ATTR isr_m2() {
  if (digitalRead(M2_ENB)) {
    encoder_count[1]--;
  } else {
    encoder_count[1]++;
  }
}

void IRAM_ATTR isr_m3() {
  if (digitalRead(M3_ENB)) {
    encoder_count[2]++;
  } else {
    encoder_count[2]--;
  }
}

void IRAM_ATTR isr_m4() {
  if (digitalRead(M4_ENB)) {
    encoder_count[3]--;
  } else {
    encoder_count[3]++;
  }
}

// =====================================================
// 6. BTS7960 MOTOR CONTROL
// =====================================================

void setMotorBTS7960(
  int rpwm_pin,
  int lpwm_pin,
  int speed
) {

  speed = constrain(speed, -255, 255);

  if (speed > 0) {

    // Clear opposite direction first
    analogWrite(lpwm_pin, 0);
    analogWrite(rpwm_pin, speed);

  } else if (speed < 0) {

    analogWrite(rpwm_pin, 0);
    analogWrite(lpwm_pin, -speed);

  } else {

    analogWrite(rpwm_pin, 0);
    analogWrite(lpwm_pin, 0);
  }
}

// =====================================================
// 7. SET ALL FOUR MOTORS
// =====================================================

void moveRobot(
  int speed_m1,
  int speed_m2,
  int speed_m3,
  int speed_m4
) {

  setMotorBTS7960(
    M1_RPWM,
    M1_LPWM,
    speed_m1
  );

  setMotorBTS7960(
    M2_RPWM,
    M2_LPWM,
    speed_m2
  );

  setMotorBTS7960(
    M3_RPWM,
    M3_LPWM,
    speed_m3
  );

  setMotorBTS7960(
    M4_RPWM,
    M4_LPWM,
    speed_m4
  );
}

// =====================================================
// 8. STOP ALL MOTORS
// =====================================================

void stopMotors() {

  moveRobot(0, 0, 0, 0);

  current_cmd = 'X';

  digitalWrite(
    BOARD_LED_PIN,
    LOW
  );
}

// =====================================================
// 9. PROCESS SERIAL COMMAND
// =====================================================

void processCommand(char cmd) {

  // Ping / Handshake
  if (cmd == 'P') {

    Serial.println("PONG");

    // P does not refresh motor watchdog.
    return;
  }

  // Valid motor command received
  lastCommandTime = millis();

  current_cmd = cmd;

  switch (cmd) {

    // ---------------------------------
    // W = FORWARD
    // ---------------------------------
    case 'W':

      moveRobot(
        -BASE_SPEED,
         BASE_SPEED,
        -BASE_SPEED,
         BASE_SPEED
      );

      break;

    // ---------------------------------
    // S = BACKWARD
    // ---------------------------------
    case 'S':

      moveRobot(
         BASE_SPEED,
        -BASE_SPEED,
         BASE_SPEED,
        -BASE_SPEED
      );

      break;

    // ---------------------------------
    // A = TURN LEFT
    // ---------------------------------
    case 'A':

      moveRobot(
        BASE_SPEED,
        BASE_SPEED,
        BASE_SPEED,
        BASE_SPEED
      );

      break;

    // ---------------------------------
    // D = TURN RIGHT
    // ---------------------------------
    case 'D':

      moveRobot(
        -BASE_SPEED,
        -BASE_SPEED,
        -BASE_SPEED,
        -BASE_SPEED
      );

      break;

    // ---------------------------------
    // X = STOP
    // ---------------------------------
    case 'X':

      stopMotors();

      break;

    default:

      stopMotors();

      break;
  }

  // LED ON when motor command active
  if (current_cmd != 'X') {

    digitalWrite(
      BOARD_LED_PIN,
      HIGH
    );

  } else {

    digitalWrite(
      BOARD_LED_PIN,
      LOW
    );
  }
}

// =====================================================
// 10. MOTOR WATCHDOG
// =====================================================

void checkMotorWatchdog() {

  unsigned long now = millis();

  if (current_cmd != 'X') {

    if (
      now - lastCommandTime >= COMMAND_TIMEOUT_MS
    ) {

      stopMotors();

      // No extra serial log:
      // preserves numeric RPM telemetry format.
    }
  }
}

// =====================================================
// 11. READ ENCODERS + CALCULATE RPM
// =====================================================

void sendRPMData() {

  unsigned long current_time = millis();

  unsigned long elapsed =
    current_time - prev_time;

  if (elapsed < RPM_INTERVAL_MS) {
    return;
  }

  float dt = elapsed / 1000.0f;

  if (dt <= 0.0f) {
    return;
  }

  long counts[4];

  // Atomic copy of encoder counts
  noInterrupts();

  for (int i = 0; i < 4; i++) {

    counts[i] = encoder_count[i];
  }

  interrupts();

  // Calculate signed RPM
  for (int i = 0; i < 4; i++) {

    long pulses =
      counts[i] - prev_encoder_count[i];

    rpm[i] =
      (pulses * 60.0f) / (PPR * dt);

    prev_encoder_count[i] = counts[i];
  }

  prev_time = current_time;

  // Protocol:
  // M1,M2,M3,M4,CMD
  Serial.printf(
    "%.2f,%.2f,%.2f,%.2f,%c\n",
    rpm[0],
    rpm[1],
    rpm[2],
    rpm[3],
    current_cmd
  );
}

// =====================================================
// 12. SETUP
// =====================================================

void setup() {

  Serial.begin(BAUD_RATE);

  // ---------------------------
  // Board LED
  // ---------------------------

  pinMode(
    BOARD_LED_PIN,
    OUTPUT
  );

  digitalWrite(
    BOARD_LED_PIN,
    LOW
  );

  // ---------------------------
  // Motor PWM Pins
  // ---------------------------

  const int motorPins[] = {

    M1_RPWM,
    M1_LPWM,

    M2_RPWM,
    M2_LPWM,

    M3_RPWM,
    M3_LPWM,

    M4_RPWM,
    M4_LPWM
  };

  const int pinCount =
    sizeof(motorPins) / sizeof(motorPins[0]);

  for (int i = 0; i < pinCount; i++) {

    pinMode(
      motorPins[i],
      OUTPUT
    );

    digitalWrite(
      motorPins[i],
      LOW
    );

    analogWrite(
      motorPins[i],
      0
    );
  }

  // Ensure motors stopped
  stopMotors();

  // ---------------------------
  // Encoder Inputs
  // ---------------------------

  pinMode(M1_ENA, INPUT_PULLUP);
  pinMode(M1_ENB, INPUT_PULLUP);

  pinMode(M2_ENA, INPUT_PULLUP);
  pinMode(M2_ENB, INPUT_PULLUP);

  pinMode(M3_ENA, INPUT_PULLUP);
  pinMode(M3_ENB, INPUT_PULLUP);

  pinMode(M4_ENA, INPUT_PULLUP);
  pinMode(M4_ENB, INPUT_PULLUP);

  // ---------------------------
  // Encoder Interrupts
  // ---------------------------

  attachInterrupt(
    digitalPinToInterrupt(M1_ENA),
    isr_m1,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(M2_ENA),
    isr_m2,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(M3_ENA),
    isr_m3,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(M4_ENA),
    isr_m4,
    RISING
  );

  // ---------------------------
  // Reset Encoder Counts
  // ---------------------------

  noInterrupts();

  for (int i = 0; i < 4; i++) {

    encoder_count[i] = 0;
  }

  interrupts();

  for (int i = 0; i < 4; i++) {

    prev_encoder_count[i] = 0;
    rpm[i] = 0.0f;
  }

  // ---------------------------
  // Initialize Timers
  // ---------------------------

  prev_time = millis();

  lastCommandTime = millis();

  current_cmd = 'X';
}

// =====================================================
// 13. MAIN LOOP
// =====================================================

void loop() {

  // Read all available commands
  while (Serial.available() > 0) {

    char cmd = Serial.read();

    if (
      cmd == 'W' ||
      cmd == 'S' ||
      cmd == 'A' ||
      cmd == 'D' ||
      cmd == 'X' ||
      cmd == 'P'
    ) {

      processCommand(cmd);
    }
  }

  // Stop if command stream is lost
  checkMotorWatchdog();

  // Send encoder telemetry
  sendRPMData();
}
