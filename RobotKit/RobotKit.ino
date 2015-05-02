#include <Servo.h>

#define DEBUG

#define TARGET_SPEED 200
#define FREQ_TIME 100
#define Kp 0.75
#define Ki 0
#define Kd 0
#define TURN_LEFT_DELAY 200
#define BACKWARD_DELAY 300

const int echoPin = 0;
const int trigPin = 1;
unsigned long distance = 0;

const int modePin = 4;

// right motor
const int enA = 11;
const int inA = 12;
const int inB = 13;
// left motor
const int enB = 6;
const int inC = 7;
const int inD = 8;
int actualLeftMotorSpeed = 0;
int actualRightMotorSpeed = 0;
int errorSum = 0;
int lastError = 0;
int adjustPWM = 0;

const int maxDistance = 30;

volatile long countLeft = 0;
volatile long countRight = 0;
unsigned long lastTime = 0;
unsigned int pulsesPerTurn = 20;

Servo robotServo;

void setup() {
#ifdef DEBUG
  Serial.begin(38400);
#endif
  
  // SRF05 pin configuration
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  
  // mode pin configuration
  pinMode(modePin, INPUT);
  digitalWrite(modePin, HIGH);
  
  // motors pins configuration
  pinMode(enA, OUTPUT);
  pinMode(inA, OUTPUT);
  pinMode(inB, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(inC, OUTPUT);
  pinMode(inD, OUTPUT);
  
  // internal pull-up interrupt pins
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  //digitalWrite(2, HIGH);
  //digitalWrite(3, HIGH);
  attachInterrupt(0, readEncoderLeft, FALLING);
  attachInterrupt(1, readEncoderRight, FALLING);
  
  // disable motor run
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  
  // servo pin configuration
  robotServo.attach(10);
  robotServo.write(90);
}

void loop() {
  // make sure robot is going straight forward
  if (millis() - lastTime > FREQ_TIME) {
    getMotorsSpeed();
    adjustPWM = calculatePID(TARGET_SPEED, actualLeftMotorSpeed, actualRightMotorSpeed);
    Serial.println(adjustPWM);
    lastTime = millis();
  }
  
  if (digitalRead(modePin) == HIGH) {
    #ifdef DEBUG
    Serial.println("Auto run mode");
    #endif
    // auto run mode
    calculateDistance();
    
    if ((int)distance <= maxDistance) {
      #ifdef DEBUG
      Serial.println("Near obstacle!!!");
      #endif
      stopMotors();
      delay(500);
      goBackward();
      delay(BACKWARD_DELAY);
      turn90Left();
      delay(TURN_LEFT_DELAY);
    } else {
      #ifdef DEBUG
      Serial.println("Going straight!!!");
      #endif
      goStraight();
    }
  } else {
    // manual run mode
    #ifdef DEBUG
    //Serial.println("Manual run mode");
    #endif
    goStraight();
  }
}

void getMotorsSpeed() {
  // detached interrupt during reading
  detachInterrupt(0);
  detachInterrupt(1);
  
  static long lastCountLeft;
  static long lastCountRight;
  
  actualLeftMotorSpeed = (countLeft - lastCountLeft) * 60 / FREQ_TIME * 1000 / pulsesPerTurn;
  #ifdef DEBUG
  Serial.print(F("Left motor speed: "));
  Serial.println(actualLeftMotorSpeed);
  #endif
  lastCountLeft = countLeft;
  
  actualRightMotorSpeed = (countRight - lastCountRight) * 60 / FREQ_TIME * 1000 / pulsesPerTurn;
  #ifdef DEBUG
  Serial.print(F("Right motor speed: "));
  Serial.println(actualRightMotorSpeed);
  #endif
  lastCountRight = countRight;
  
  // reattach interrupt
  attachInterrupt(0, readEncoderLeft, FALLING);
  attachInterrupt(1, readEncoderRight, FALLING);
}

int calculatePID(int pwm, int targetSpeed, int currentSpeed) {
  float PID = 0;
  int error = 0;
  int diffError = 0;
  static int lastError = 0;
 
  error = targetSpeed - currentSpeed;
  Serial.println(error);
  errorSum += error;
  diffError = error - lastError;
  PID = Kp * error + Ki * errorSum + Kd * diffError;
  Serial.println(PID);
  
  lastError = error;
  
  return constrain(pwm + PID, 0, 255);
}

void goStraight() {
  digitalWrite(inA, LOW);
  digitalWrite(inB, HIGH);
  analogWrite(enA, adjustPWM);
  digitalWrite(inC, LOW);
  digitalWrite(inD, HIGH);
  analogWrite(enB, TARGET_SPEED);
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

void goBackward() {
  digitalWrite(inA, HIGH);
  digitalWrite(inB, LOW);
  analogWrite(enA, adjustPWM);
  digitalWrite(inC, HIGH);
  digitalWrite(inD, LOW);
  analogWrite(enB, TARGET_SPEED);
}

void turn90Left() {
  digitalWrite(inA, HIGH);
  digitalWrite(inB, LOW);
  analogWrite(enA, TARGET_SPEED);
  digitalWrite(inC, LOW);
  digitalWrite(inD, HIGH);
  analogWrite(enB, TARGET_SPEED);
}

// read distance from SRF05 sensor
void calculateDistance() {
  unsigned long duration;
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration / 29 / 2;
  
#ifdef DEBUG
  Serial.print("Distance in cm: ");
  Serial.println(distance);
#endif 
}

void readEncoderLeft() {
  countLeft++;
}

void readEncoderRight() {
  countRight++;
}
