//For- Multi-tasking robot(car) grade:10 - computer engineering--
//Here's our complete code for obstacle avoidance feature  in our car:
#include <AFMotor.h>
#include <Servo.h>
#include <NewPing.h> 
// Motor Setup
AF_DCMotor motorLeft1(1);
AF_DCMotor motorLeft2(2);
AF_DCMotor motorRight1(3);
AF_DCMotor motorRight2(4);
// Ultrasonic Sensor Pins
const int trigPin = 8;
const int echoPin = 3;
const int servoPin = 11;
// Configuration Constants
const int SAFE_DISTANCE = 25;     // Safe distance in centimeters
const int MOTOR_SPEED = 200;      // Motor speed
const int MAX_DISTANCE = 200;     // Maximum detection distance
const int OUT_OF_RANGE = 255;     // Out of range indicator

//here are global Variables: 
Servo servo_motor;
int distance;

void setup() {
  Serial.begin(9600);
  
  // Motor Setup
  motorLeft1.setSpeed(MOTOR_SPEED);
  motorLeft2.setSpeed(MOTOR_SPEED);
  motorRight1.setSpeed(MOTOR_SPEED);
  motorRight2.setSpeed(MOTOR_SPEED);

  // Ultrasonic Sensor Setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Servo Setup
  servo_motor.attach(servoPin);
  servo_motor.write(90);  //it center the servo
  delay(2000);

  //  it initialize smooth distance
  distance = getSmoothedDistance();
}

void loop() {
  int distanceRight = 0, distanceLeft = 0;
  delay(50);

  if (distance <= SAFE_DISTANCE) {
    moveStop();
    delay(300);
    moveBackward();
    delay(400);
    moveStop();
    delay(300);

    // Look directions and decide
    distanceRight = lookDirection(10);  // Look right----right------------
    distanceLeft = lookDirection(170);  // Look left------------left-----------

    if (distanceRight >= distanceLeft) {
      turnRight();
    } else {
      turnLeft();
    }
  } else {
    moveForward();
  }

  distance = getSmoothedDistance();  // Update distance
}

// Motor Control Functions-----motors-------------------
void moveForward() {
  motorLeft1.run(FORWARD);
  motorLeft2.run(FORWARD);
  motorRight1.run(FORWARD);
  motorRight2.run(FORWARD);
}

void moveBackward() {
  motorLeft1.run(BACKWARD);
  motorLeft2.run(BACKWARD);
  motorRight1.run(BACKWARD);
  motorRight2.run(BACKWARD);
}

void turnLeft() {
  motorLeft1.run(BACKWARD);
  motorLeft2.run(BACKWARD);
  motorRight1.run(FORWARD);
  motorRight2.run(FORWARD);
}

void turnRight() {
  motorLeft1.run(FORWARD);
  motorLeft2.run(FORWARD);
  motorRight1.run(BACKWARD);
  motorRight2.run(BACKWARD);
}

void moveStop() {
  motorLeft1.run(RELEASE);
  motorLeft2.run(RELEASE);
  motorRight1.run(RELEASE);
  motorRight2.run(RELEASE);
}

// Servo Logic: Look Direction------------------------
int lookDirection(int angle) {
  servo_motor.write(angle);
  delay(500);
  int distance = measureDistance();
  servo_motor.write(90);  // Reset to center-------------------------
  delay(100);
  return distance;
}

// Distance Measurement
int measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  int cm = duration * 0.034 / 2;    //--**********------logic----*****-------------

  return cm == 0 ? OUT_OF_RANGE : constrain(cm, 0, MAX_DISTANCE);
}

int getSmoothedDistance() {
  int total = 0, samples = 4;
  for (int i = 0; i < samples; i++) {
    total += measureDistance();
    delay(50);
  }
  return total / samples;
}
