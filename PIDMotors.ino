// Lightweight Arduino Bridge: READ_ENCODERS ('e'), MOTOR_RAW_PWM ('o'), MOTOR_SPEEDS ('m'), 
//RESET_ENCODERS ('r') with PID and original serial parser

#include <Arduino.h>
#include <avr/interrupt.h>
#include <string.h>

// ---- Pin Definitions ----
#define LEFT_ENC_PIN_A  PK0  // Left encoder A (PCINT16)
#define LEFT_ENC_PIN_B  PK1  // Left encoder B (PCINT17)
#define RIGHT_ENC_PIN_A PB0  // Right encoder A (PCINT0)
#define RIGHT_ENC_PIN_B PB1  // Right encoder B (PCINT1)
const int pwmLeftPin  = 3;   // PWM pin for left motor
const int pwmRightPin = 11;  // PWM pin for right motor

// ---- Encoder state & lookup table ----
volatile long left_enc_pos  = 0;
volatile long right_enc_pos = 0;
static const int8_t ENC_STATES[] = {0, -1,  1,  0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// ---- PID parameters ----
struct PID { float Kp, Ki, Kd; long setpoint; float integral; long lastError; };
PID leftPID  = {0.6, 0.05, 0.2, 0, 0, 0};
PID rightPID = {0.9, 0.15, 0.1, 0, 0, 0};

// ---- PID timing ----
const unsigned long PID_RATE     = 40;              // Hz
const unsigned long PID_INTERVAL = 1000 / PID_RATE; // ms
unsigned long nextPID;
unsigned long startTime = millis();
float elapsedTime = 0.0;

// ---- Serial parsing state ----
int arg = 0;
int index = 0;
char cmd = 0;
char argv1[16];
char argv2[16];
long arg1 = 0;
long arg2 = 0;

// ---- PID enable flag ----
bool pidEnabled = false;

// ---- Function prototypes ----
void resetCommand();
void runCommand();
long readEncoder(int side);
void setRawPWM(long left, long right);
void updatePIDLoop();

// ---- PCINT ISRs for quadrature decoding ----
ISR(PCINT2_vect) {
  static uint8_t enc_last = 0;
  enc_last <<= 2;
  uint8_t cur = (PINK & ((1 << LEFT_ENC_PIN_A) | (1 << LEFT_ENC_PIN_B))) >> LEFT_ENC_PIN_A;
  enc_last |= cur;
  left_enc_pos += ENC_STATES[enc_last & 0x0F];
}
ISR(PCINT0_vect) {
  static uint8_t enc_last = 0;
  enc_last <<= 2;
  uint8_t cur = (PINB & ((1 << RIGHT_ENC_PIN_A) | (1 << RIGHT_ENC_PIN_B))) >> RIGHT_ENC_PIN_A;
  enc_last |= cur;
  right_enc_pos += ENC_STATES[enc_last & 0x0F];
}

void setup() {
  // Encoder pins
  pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);
  // Enable PCINT on encoder pins
  PCMSK2 |= (1 << LEFT_ENC_PIN_A) | (1 << LEFT_ENC_PIN_B);
  PCMSK0 |= (1 << RIGHT_ENC_PIN_A) | (1 << RIGHT_ENC_PIN_B);
  PCICR  |= (1 << PCIE2) | (1 << PCIE0);
  sei();  // Global interrupt enable

  // Relay pin behavior retained
  pinMode(41, OUTPUT);
  digitalWrite(41, LOW);
  digitalWrite(41, HIGH);

  // Motor PWM pins
  pinMode(pwmLeftPin, OUTPUT);
  pinMode(pwmRightPin, OUTPUT);

  // Neutral PWM until first command
  setRawPWM(0, 0);

  // Serial interface
  Serial.begin(57600);
  resetCommand();
  nextPID = millis() + PID_INTERVAL;
}

void loop() {
  // Serial parsing
  while (Serial.available() > 0) {
    char chr = Serial.read();
    if (chr == '\n') continue;      // Ignore LF
    if (chr == '\r') {              // CR terminates command
      runCommand();
      resetCommand();
      continue;
    }
    if (chr == ' ') {
      if (arg == 0) arg = 1;
      else if (arg == 1) { argv1[index] = '\0'; arg = 2; index = 0; }
      continue;
    }
    if (arg == 0) {
      cmd = chr;
    } else if (arg == 1) {
      if (index < sizeof(argv1) - 1) argv1[index++] = chr;
    } else if (arg == 2) {
      if (index < sizeof(argv2) - 1) argv2[index++] = chr;
    }
  }
  // PID loop at fixed rate
  if (millis() >= nextPID) {
    updatePIDLoop();
    nextPID += PID_INTERVAL;
  }
}

void resetCommand() {
  cmd = 0;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg = 0;
  index = 0;
  arg1 = 0;
  arg2 = 0;
}

void runCommand() {
  argv1[index] = '\0';
  argv2[index] = '\0';
  arg1 = atol(argv1);
  arg2 = atol(argv2);
  switch (cmd) {
    case 'e':  // READ_ENCODERS
      Serial.print(readEncoder(0)); Serial.print(' '); Serial.println(readEncoder(1));
      break;
    case 'o':  // MOTOR_RAW_PWM
      pidEnabled = false;
      setRawPWM(arg1, arg2);
      Serial.println("OK");
      break;
    case 'm':  // MOTOR_SPEEDS (PID)
      leftPID.setpoint  = arg1;
      rightPID.setpoint = arg2;
      leftPID.integral  = 0; leftPID.lastError = 0;
      rightPID.integral = 0; rightPID.lastError= 0;
      pidEnabled = (arg1 != 0 || arg2 != 0);
      Serial.println("OK");
      break;
    case 'r':  // RESET_ENCODERS
      left_enc_pos  = 0;
      right_enc_pos = 0;
      pidEnabled = false;
      Serial.println("OK");
      break;
    default:
      Serial.println("Invalid Command");
      break;
  }
}

long readEncoder(int side) {
  return (side == 0) ? left_enc_pos : -right_enc_pos;
}

void setRawPWM(long left, long right) {
  left  = constrain(left,  -500, 500);
  right = constrain(right, -500, 500);
  uint8_t pwmL = (uint8_t)map(left,  -500, 500, 0, 255);
  uint8_t pwmR = (uint8_t)map(right, -500, 500, 0, 255);
  analogWrite(pwmLeftPin,  pwmL);
  analogWrite(pwmRightPin, pwmR);
}

void updatePIDLoop() {
  if (!pidEnabled) return;
  static long lastL = 0;
  static long lastR = 0;
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - startTime) / 1000.0;
  long curL = left_enc_pos;
  long curR = right_enc_pos;
  long speedL = curL - lastL;
  long speedR = curR - lastR;
  lastL = curL;
  lastR = curR;
  // Compute PID outputs
  float errL = leftPID.setpoint - speedL;
  leftPID.integral += errL;
  float dL = errL - leftPID.lastError;
  float outL = leftPID.Kp * errL + leftPID.Ki * leftPID.integral + leftPID.Kd * dL;
  leftPID.lastError = errL;
  float errR = rightPID.setpoint - speedR;
  rightPID.integral += errR;
  float dR = errR - rightPID.lastError;
  float outR = rightPID.Kp * errR + rightPID.Ki * rightPID.integral + rightPID.Kd * dR;
  rightPID.lastError = errR;
  // Apply outputs
  setRawPWM((long)outL, (long)outR);
  // Telemetry: time, speedL, speedR, outL, outR
  Serial.print(elapsedTime); Serial.print('\t');
  Serial.print(-speedL); Serial.print('\t');
  Serial.print(speedR); Serial.print('\t');
  Serial.print(leftPID.setpoint); Serial.print('\t');
  Serial.println(rightPID.setpoint);
}
