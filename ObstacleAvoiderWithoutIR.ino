#include <Servo.h>
#include <L298N.h>

#define trigPin 10
#define echPin 11


Servo servo1;

//left motors
const unsigned int IN1 = 5;
const unsigned int IN2 = 6;
const unsigned int ENL = 7;

//right motors
const unsigned int IN3 = 2;
const unsigned int IN4 = 3;
const unsigned int ENR = 4;


L298N motorL(ENL, IN1, IN2);
L298N motorR(ENR, IN3, IN4);

//state tracking
bool lastTurnLeft = false;
int stuckCounter = 0;


long ultraSonicDistance(int triggerPin, int echoPin) {
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);

  return pulseIn(echoPin, HIGH);  
  }

void turnLeft(unsigned long duration) {
  motorL.backward();
  motorR.forward();
  delay(duration);
  motorL.stop();
  motorR.stop();
}

void turnRight(unsigned long duration) {
  motorL.forward();
  motorR.backward();
  delay(duration);
  motorL.stop();
  motorR.stop();
}

long angleIR(int angle) {
  servo1.write(angle);
  delay(500);
  return ultraSonicDistance(trigPin,echPin);
}



void setup() {
  Serial.begin(9600);
  //Servo 
  
  servo1.attach(9);
  servo1.write(90);

  //IR sensors
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  //DC motors
  motorL.setSpeed(100);
  motorR.setSpeed(100);

  //LED
  pinMode(13, OUTPUT);

  randomSeed(analogRead(A0));
}

void loop() {
  int cm = 0.01723 * ultraSonicDistance(trigPin,echPin);
  Serial.print(cm);
  Serial.println("cm");
  delay(10);
  long frontface = angleIR(90);

 

  if (cm <= 25) {
    
    digitalWrite(13, HIGH);
    Serial.println("Obstacle detected!");
    motorL.stop();
    motorR.stop();
    delay(200);
    Serial.println("Vehicle stopped running!");

    
    motorL.setSpeed(70);
    motorR.setSpeed(70);
    Serial.println("Vehicle moving backward as obstacle detected!");
    motorL.backward();
    motorR.backward();
    delay(1000);
    motorL.stop();
    motorR.stop();
    Serial.println("Vehicle stopped and scanning for a clear pathway!");

    long leftSide = angleIR(150);
    long rightSide = angleIR(30);
    servo1.write(90);

    if (abs(leftSide - rightSide) < 10) {
      if (random(0, 2) == 0) {
        Serial.println("Randomly turning left!");
        turnLeft(600);
        lastTurnLeft = true;
      }
      else {
        Serial.println("Randomly turning right!");
        turnRight(600);
        lastTurnLeft = false;
      }

    }

    else if (leftSide > rightSide) {
      Serial.println("Turning left!");
      turnLeft(600);
      lastTurnLeft = true;
    } 
       else {
      Serial.println("Turning Right!");
      turnRight(600);
      lastTurnLeft = false;
    }

    stuckCounter++;
    if (stuckCounter >= 3) {
      Serial.println("Stuck detected! Performing escape!");
      motorL.backward();
      motorR.backward();
      delay(1000);
      turnLeft(1000);
      stuckCounter = 0;
    }

    delay(1000);
    motorL.stop();
    motorR.stop();
      
    digitalWrite(13, LOW);
    motorL.setSpeed(100);
    motorR.setSpeed(100);

    motorL.forward();
    motorR.forward();
    Serial.println("Vehicle moving forward again!");


    } else {
      motorL.setSpeed(100);
      motorR.setSpeed(100);
      motorL.forward();
      motorR.forward();
      Serial.println("Moving forward...");

    }
 
  delay(100);
    
}
