const int encoderLPinA = 27;   // Left Encoder Channel A
const int encoderLPinB = 25;   // Left Encoder Channel B
const int encoderRPinA = 26;   // Right Encoder Channel A
const int encoderRPinB = 22;   // Right Encoder Channel B

const int relay = 41;

// Define the pin for the potentiometer
const int potPin = A0; // Analog input pin connected to the potentiometer

// Define the output PWM pin connected to SyRen S1
const int pwmPin1 = 3;  // Digital PWM pin for drive motor speed and direction
const int pwmPin2 = 11;

#include <AccelStepper.h>
// Stepper motor pins
#define stepPin 7
#define dirPin 6

// Encoder pins
#define outputA 31
#define outputB 45

// Define stepper and variables
AccelStepper stepper(1, stepPin, dirPin);
int counter = 0;
int laststate;
int state;
const int encoderMin = -16; // Minimum encoder position
const int encoderMax = 16;  // Maximum encoder position

// --- Vehicle Parameters for Ackermann Steering ---
float L = 1.41; // Wheelbase (meters)
float W = 0.914; // Track width (meters)
float r_wheel = 0.203; // Wheel radius (meters)

void setup() {
  pinMode(relay, OUTPUT);
  digitalWrite(relay, LOW); digitalWrite(relay, HIGH);
  
  pinMode(potPin, INPUT);
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  
  // Encoder pin configuration
  pinMode(outputA, INPUT);
  pinMode(outputB, INPUT);

  // Stepper motor configuration
  stepper.setMaxSpeed(1500);
  stepper.setAcceleration(900);

  // Initialize encoder and motor position
  Serial.begin(9600);
  laststate = digitalRead(outputA);
  stepper.setCurrentPosition(0);

  
}

void loop() {
  // Read encoder state
  state = digitalRead(outputA);
  if (state != laststate) { // Detect encoder pulse
    if (digitalRead(outputB) != state) {
      counter++; // Clockwise
    } else {
      counter--; // Counterclockwise
    }

    // Clamp encoder counter to limits
    if (counter > encoderMax) {
      counter = encoderMax;
    } else if (counter < encoderMin) {
      counter = encoderMin;
    }

    // Map encoder count to stepper target position
    int targetPosition = map(counter, encoderMin, encoderMax, 270, -270);
    stepper.moveTo(targetPosition);
    // Debugging output
    Serial.print("Encoder Position: ");
    Serial.print(counter);
    Serial.print(" | Stepper Target: ");
    Serial.println(targetPosition);

  }

  laststate = state; // Update last state

  // Run the stepper motor
  stepper.run();

  //Driver motor speed control
  int potValue = analogRead(potPin);
  int pwmValue = map(potValue, 0, 1023, 0, 255);
  // Write the PWM signal to the motor driver
  analogWrite(pwmPin1, pwmValue);
  analogWrite(pwmPin2, pwmValue);
  
   // --- Calculate Wheel Velocities using Ackermann Steering ---
  float v = 10.0; // Example constant velocity (m/s)
  float delta = map(counter, -16, 16, -16, 16); // Use the encoder position as the steering angle
  
  //float delta = (float)counter; // Use the encoder position as the steering angle

  // Calculate the turning radius R
  float R;
  if (delta == 0) {
    R = 1e6; // Large value for straight
  } else {
    R = L / tan(radians(delta)); // Radius of the turn
  }

  // Calculate the inner and outer radii
  float R_i = R - (W / 2); // Inner radius
  float R_o = R + (W / 2); // Outer radius

  // Calculate the angular velocity of the left and right rear wheels
  float omega = v / R; // Angular velocity of the vehicle
  float wr_l = (omega * (R_i / r_wheel)); // Left rear wheel angular velocity
  float wr_r = (omega * (R_o / r_wheel)); // Right rear wheel angular velocity


}
