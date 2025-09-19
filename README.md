// ==== Obstacle Avoiding Robot (Enhanced) ====
#include <Servo.h>
#include <NewPing.h>

// ==== Servo & Ultrasonic ====
#define SERVO_PIN 3
#define TRIG_PIN 8
#define ECHO_PIN 9
#define MAX_DISTANCE 200     // cm
#define SAFE_DISTANCE 15     // cm

// ==== Motor Driver Pins ====
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7
#define EN1 10   // Right Motor Enable (PWM)
#define EN2 11   // Left Motor Enable (PWM)

// ==== Motor Speed ====
#define MOTOR_SPEED 150   // 0–255

// ==== Objects ====
Servo myServo;
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

// ==== Setup ====
void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);

  myServo.attach(SERVO_PIN);
  myServo.write(90); // center
  delay(500);
}

// ==== Main Loop ====
void loop() {
  int distance = getDistance();

  if (distance > SAFE_DISTANCE) {
    // No obstacle → move forward
    moveForward(MOTOR_SPEED);
  } else {
    // Obstacle detected → stop and move backward briefly
    stopMotors();
    delay(100);
    moveBackward(MOTOR_SPEED);
    delay(300);
    stopMotors();
    delay(100);

    // Scan left
    myServo.write(150);
    delay(400);
    int distanceLeft = getDistance();

    // Scan right
    myServo.write(30);
    delay(400);
    int distanceRight = getDistance();

    // Return to center
    myServo.write(90);
    delay(200);

    // Decide direction
    if (distanceLeft > SAFE_DISTANCE || distanceRight > SAFE_DISTANCE) {
      if (distanceLeft >= distanceRight) {
        turnLeft();
      } else {
        turnRight();
      }
    } else {
      // Both sides blocked → move backward again
      moveBackward(MOTOR_SPEED);
      delay(500);
      stopMotors();
    }
  }
}

// ==== Functions ====
int getDistance() {
  delay(50);
  int cm = sonar.ping_cm();
  if (cm == 0) cm = MAX_DISTANCE;  // No obstacle detected
  return cm;
}

void moveForward(int speedVal) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(EN1, speedVal);
  analogWrite(EN2, speedVal);
}

void moveBackward(int speedVal) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(EN1, speedVal);
  analogWrite(EN2, speedVal);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(EN1, MOTOR_SPEED);
  analogWrite(EN2, MOTOR_SPEED);
  delay(400);
  stopMotors();
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(EN1, MOTOR_SPEED);
  analogWrite(EN2, MOTOR_SPEED);
  delay(400);
  stopMotors();
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
}
