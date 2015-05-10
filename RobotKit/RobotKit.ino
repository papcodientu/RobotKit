#include <IRremote.h>
#include <Servo.h>

#define DEBUG

#define MAX_DISTANCE 15

#define TARGET_SPEED 200
#define FREQ_TIME 200
#define Kp 0.7
#define Ki 0
#define Kd 0
#define TURN_LEFT_DELAY 200
#define BACKWARD_DELAY 300

#define FORWARD 0xEC27D43D 
#define BACKWARD 0x86BD99C
#define LEFT 0x1A422E43
#define RIGHT 0xA23BD824
#define STOP 0x7295A904

// ultrasonic sensor
const int echoPin = 0;
const int trigPin = 1;
unsigned long distance = 0;

// mode button
const int modePin = 4;

// IR
const int IRPin = 9;
IRrecv irrecv(IRPin);
decode_results results;
unsigned int command = 0;

// Servo
const int servoPin = 10;
Servo robotServo;

// left motor
const int enA = 5;
const int inA = 7;
const int inB = 8;
// right motor
const int enB = 6;
const int inC = 12;
const int inD = 13;
int actualLeftMotorSpeed = 0;
int actualRightMotorSpeed = 0;
int errorSum = 0;
int lastError = 0;
int adjustPWM = 0;
volatile long countLeft = 0;
volatile long countRight = 0;
unsigned long lastTime = 0;
unsigned int pulsesPerTurn = 20;


void setup() {
  Serial.begin(38400);
  
  // ir receiver
  irrecv.enableIRIn();
  
  // SRF05 pin configuration
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  
  // internal pull-up interrupt pins
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
  attachInterrupt(0, readEncoderLeft, FALLING);
  attachInterrupt(1, readEncoderRight, FALLING);
  
  // mode pin configuration
  pinMode(modePin, INPUT);
  digitalWrite(modePin, HIGH);
  
  // servo pin configuration
  robotServo.attach(servoPin);
  robotServo.write(90);
  
  // motors pins configuration
  pinMode(enA, OUTPUT);
  pinMode(inA, OUTPUT);
  pinMode(inB, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(inC, OUTPUT);
  pinMode(inD, OUTPUT);
  //
  digitalWrite(inA, LOW);
  digitalWrite(inB, HIGH);
  analogWrite(enA, 0);
  digitalWrite(inC, LOW);
  digitalWrite(inD, HIGH);
  analogWrite(enB, 0);
}

void loop() {
  checkPID();
  
  if (digitalRead(modePin) == HIGH) {
    #ifdef DEBUG
    Serial.println("Auto run mode");
    #endif
    // auto run mode
    calculateDistance();
    
    if ((int)distance <= MAX_DISTANCE) {
      #ifdef DEBUG
      Serial.println("Near obstacle!!!");
      #endif
      stopMotors();
      delay(500);
      goBackward();
      delay(BACKWARD_DELAY);
      turnLeft();
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
    IRDecoder(); 
  }
}

void IRDecoder() {
  if (irrecv.decode(&results)) {
    #ifdef DEBUG
    Serial.println(results.value, HEX);
    #endif
    if (results.value != 0xFFFFFFFF) {
      switch(results.value) {
        case FORWARD:
          goStraight();
          break;
        case BACKWARD:
          goBackward();
          break;
        case LEFT:
          turnLeft();
          break;
        case RIGHT:
          turnRight();
          break;
        default:
          stopMotors();
          break;
      }  
    }
  }
  irrecv.resume();
  delay(100);
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

void checkPID() {
  // make sure robot is going straight forward
  if (millis() - lastTime > FREQ_TIME) {
    getMotorsSpeed();
    adjustPWM = calculatePID(TARGET_SPEED, actualLeftMotorSpeed, actualRightMotorSpeed);
    Serial.println(adjustPWM);
    lastTime = millis();
  } 
}

int calculatePID(int pwm, int targetSpeed, int currentSpeed) {
  float PID = 0;
  int error = 0;
  int diffError = 0;
  static int lastError = 0;
 
  error = targetSpeed - currentSpeed;
  //Serial.println(error);
  errorSum += error;
  diffError = error - lastError;
  PID = Kp * error + Ki * errorSum + Kd * diffError;
  //Serial.println(PID);
  
  lastError = error;
  
  return (int)PID;
}

void stopMotors() {
  digitalWrite(inA, LOW);
  digitalWrite(inB, LOW);
  analogWrite(enA, 0);
  digitalWrite(inC, LOW);
  digitalWrite(inD, LOW);
  analogWrite(enB, 0);
}

void goStraight() {
  digitalWrite(inA, LOW);
  digitalWrite(inB, HIGH);
  analogWrite(enB, TARGET_SPEED);
  //analogWrite(enA, TARGET_SPEED);
  digitalWrite(inC, LOW);
  digitalWrite(inD, HIGH);
  analogWrite(enA, constrain (TARGET_SPEED + adjustPWM, 0, 255));
}

void goBackward() {
  digitalWrite(inA, HIGH);
  digitalWrite(inB, LOW);
  analogWrite(enA, TARGET_SPEED);
  digitalWrite(inC, HIGH);
  digitalWrite(inD, LOW);
  analogWrite(enB, constrain (TARGET_SPEED + adjustPWM, 0, 255));
}

void turnLeft() {
  digitalWrite(inA, LOW);
  digitalWrite(inB, HIGH);
  analogWrite(enA, TARGET_SPEED);
  digitalWrite(inC, HIGH);
  digitalWrite(inD, LOW);
  analogWrite(enB, TARGET_SPEED);
}

void turnRight() {
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
