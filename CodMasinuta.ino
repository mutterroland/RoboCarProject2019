#include <NewPing.h>
#include <RobotIRremote.h>
#include <RobotIRremoteInt.h>
#include <RobotIRremoteTools.h>
#include <L298N.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <dht.h>
#define DHT11_PIN 30
dht DHT;


NewPing sonar(12, 13); //pini ultrasonic 12, 13
SoftwareSerial BTserial(0, 1);

long cm;

Servo Servo1;

//int loop = false;

//Senzori IR
const int IR1 = 31;
const int IR2 = 32;
const int IR3 = 33;

// Pini Motor A (enableA = porneste motor, pinA1 = inainte, pinA2 = inapoi)
int enableA = 8;
int pinA1 = 3;
int pinA2 = 2;

//Pini Motor B (enabledB = porneste motor, pinB2 = inainte, pinB2 = inapoi)
int enableB = 9;
int pinB1 = 5;
int pinB2 = 4;

// Pini Servo
int servo = 6;
int distanceScari;
int distanceChallange1 = 132;
bool challange1 = true;
int distanceChallange2 = 195;
bool challange2 = true;
int distanceChallange3 = 40;
bool challange3 = true;
int distanceChallange4;
bool challange4 = true;
int distanceChallangeFinal = 40;
int distanceCurrent; // declarare variabila pentru distanta curenta
int distanceRight, distanceLeft; // declarare variabila pentru distanta dreapta, stanga
int collisionDistance = 50; // declarare distanta de coliziune maxima
char data = 0;
byte GetValue;
int distanceTo = 40;

bool linetracking = false;
bool ir = false;
bool avoid = false;
bool challange = false;

void setup() {
  pinMode(enableA, OUTPUT);
  pinMode(pinA1, OUTPUT);
  pinMode(pinA2, OUTPUT);

  pinMode(enableB, OUTPUT);
  pinMode(pinB1, OUTPUT);
  pinMode(pinB2, OUTPUT);

  Servo1.attach(servo);
  Serial.begin(9600);
  Servo1.write(75);
  BTserial.begin(9600);

  motorAOn();
  motorBOn();
  //analogWrite(enableA, 200);
  //analogWrite(enableB, 200);

}



void loop() {
  //citesteDate();
  if (Serial.available() > 0)    // Send data only when you receive data:
  {
    data = Serial.read();
    //Serial.println(data);
    switch (data) {
      case 'T':
        motorABrake();
        motorBBrake();
        linetracking = false;
        ir = false;
        avoid = false;
        break;

      // Logic for object avoidance
      case 'A':
        Serial.println("A");
        linetracking = false;
        ir = false;
        avoid = true;
        challange = false;
        break;
      // Logic for line following
      case 'I':
        linetracking = true;
        ir = false;
        avoid = false;
        challange = false;
        break;
      case 'F':
        linetracking = false;
        ir = false;
        avoid = false;
        challange = false;
        motorAForward();
        motorBForward();
        break;
      case 'R':
        linetracking = false;
        ir = false;
        avoid = false;
        motorAForward();
        motorBBackward();
        break;
      case 'L':
        linetracking = false;
        ir = false;
        avoid = false;
        challange = false;
        motorABackward();
        motorBForward();
        break;
      case 'S':
        linetracking = false;
        ir = false;
        avoid = false;
        challange = false;
        motorABrake();
        motorBBrake();
        break;
      case 'B':
        linetracking = false;
        ir = false;
        avoid = false;
        challange = false;
        motorABackward();
        motorBBackward();
        break;
      case 'X':
        linetracking = false;
        ir = false;
        avoid = false;
        challange = true;
    }
  }
  if (avoid) {
    Servo1.write(75);  // Ultrasonic in fata
    delay(90);
    distanceCurrent = sonar.ping_cm();
    Serial.println(distanceCurrent);
    if (distanceCurrent < collisionDistance) { // compara distanta curenta cu distanta de coliziune
      changePath();
    }  // Daca exista obiect in fata apeleaza changePath()
    forward(500);  // Miscare in fata
    delay(500);
  }
  if (linetracking) {
    int S3 = digitalRead(IR1);
    int S2 = digitalRead(IR2);
    int S1 = digitalRead(IR3);

    motorAForward();
    motorBForward();
    if (S1 == HIGH && S2 == LOW && S3 == HIGH) {
      motorAForward();
      motorBForward();
      Serial.println("Forward");
    }
    else if (S1 == LOW && S2 == LOW && S3 == HIGH) {
      motorABackward();
      motorBForward();
      Serial.println("Stanga");
    }
    else if (S1 == HIGH && S2 == LOW && S3 == LOW) {
      motorAForward();
      motorBBackward();
      Serial.println("Dreapta");
    }
    else if (S1 == LOW && S2 == LOW && S3 == LOW) {
      motorABrake();
      motorBBrake();
      Serial.println("Stop");
    }
    else if (S1 == LOW && S2 == LOW && S3 == LOW) {
      motorABrake();
      motorBBrake();
      Serial.println("Stop");
    }
  }
  if (challange) {
    int S3 = digitalRead(IR1);
    int S2 = digitalRead(IR2);
    int S1 = digitalRead(IR3);
    distanceCurrent = sonar.ping_cm();

    if (S1 == HIGH && S2 == LOW && S3 == HIGH) {
      motorAForward();
      motorBForward();
      Serial.println("1");
    }
    else if (S1 == LOW && S2 == LOW && S3 == HIGH) {
      motorABackward();
      motorBForward();
      Serial.println("2");
    }
    else if (S1 == HIGH && S2 == LOW && S3 == LOW) {
      motorAForward();
      motorBBackward();
      Serial.println("3");
    }
    else if (S1 == LOW && S2 == LOW && S3 == LOW) {
      motorABrake();
      motorBBrake();
      Serial.println("4");
    }
    else if (S1 == LOW && S2 == LOW && S3 == LOW) {
      motorABrake();
      motorBBrake();
      Serial.println("5");
    }
    if (distanceCurrent < distanceTo) {
      brake(2000);
      citesteDate();
      turnLeft(4000);
      if (S1 == HIGH || S2 == HIGH || S3 == HIGH) {
        irel();
      }
      else {
        do {
          motorABackward();
          motorBForward();
        } while (S1 == LOW && S2 == LOW && S3 == LOW);
        }

    }
  }
}


void citesteDate() {
  int chk = DHT.read11(DHT11_PIN);
  Serial.print("Temperature = ");
  Serial.print(DHT.temperature);
  Serial.println(" C");
  //BTserial.println(",");
  Serial.print("Humidity = ");
  Serial.println(DHT.humidity);
  //BTserial.println(",");
  distanceCurrent = sonar.ping_cm();
  Serial.print("Distance = ");
  Serial.print(distanceCurrent);
  Serial.print(" cm");
  Serial.print(";");
  delay(5000);
}
void irel() {
  int S3 = digitalRead(IR1);
  int S2 = digitalRead(IR2);
  int S1 = digitalRead(IR3);

  //motorAForward();
  //motorBForward();
  if (S1 == HIGH && S2 == LOW && S3 == HIGH) {
    motorAForward();
    motorBForward();
    Serial.println("1");
  }
  else if (S1 == LOW && S2 == LOW && S3 == HIGH) {
    motorABackward();
    motorBForward();
    Serial.println("2");
  }
  else if (S1 == HIGH && S2 == LOW && S3 == LOW) {
    motorAForward();
    motorBBackward();
    Serial.println("3");
  }
  else if (S1 == LOW && S2 == LOW && S3 == LOW) {
    motorABrake();
    motorBBrake();
    Serial.println("4");
  }
  else if (S1 == LOW && S2 == LOW && S3 == LOW) {
    motorABrake();
    motorBBrake();
    Serial.println("5");
  }
}
void avoidance() {
  Serial.println("avoidance");
  Servo1.write(75);  // Ultrasonic in fata
  delay(90);
  distanceCurrent = sonar.ping_cm();
  Serial.println(distanceCurrent);
  Serial.println("Whatever");
  if (distanceCurrent < collisionDistance) { // compara distanta curenta cu distanta de coliziune
    changePath();
  }  // Daca exista obiect in fata apeleaza changePath()
  forward(500);  // Miscare in fata
  delay(500);
}
void changePath() { // metoda care preia datele din dreapta si din stanga robotelului
  brake(1000);
  Servo1.write(0);
  delay(1000);
  distanceRight = sonar.ping_cm();

  Serial.print(distanceRight);
  Serial.print(" dreapta");
  Serial.print("\n");
  delay(1000);
  Servo1.write(170);
  delay(500);
  distanceLeft = sonar.ping_cm();
  Serial.print(distanceLeft);
  Serial.print(" stanga");
  Serial.print("\n");
  delay(1000);
  Servo1.write(75);
  delay(50);
  compare();
}

void compare() { //comparare distanta dreapta/stanga // daca apare o eroare du-te in spate 2 secunde si repeta
  if (distanceRight > distanceLeft) {
    turnRight(1500);
  }
  else if (distanceRight < distanceLeft) {
    turnLeft(1500);
  }
  else {
    backward(2000);
    compare();
  }
}

void fostchallange() {
  distanceCurrent = sonar.ping_cm();
  Serial.println(distanceCurrent);
  forward(1);
  if (distanceCurrent == distanceChallange1 && challange1) {
    brake(500);
    challange1 = false;
    Servo1.write(0);
    while (distanceCurrent != distanceChallange1 + 10) {
      motorABackward();
      motorBForward();
    }
    brake(1000);
  }
  Servo1.write(75);
  if (distanceCurrent == distanceChallange2 && challange1 == false && challange2) {
    brake(500);
    challange2 = false;
    Servo1.write(0);
    while (distanceCurrent != distanceChallange2 + 10) {
      motorABackward();
      motorBForward();
    }
    brake(1000);
    Servo1.write(75);
  }
  if (distanceCurrent == distanceChallange3 && challange1 == false && challange2 == false && challange3) {
    challange3 = false;
    brake(1000);
    // insert logic for bluetooth here
    Servo1.write(0);
    delay(1000);
    distanceScari = sonar.ping_cm();
    delay(1000);
    Servo1.write(170);
    delay(1000);
    while (distanceCurrent != distanceScari) {
      motorABackward();
      motorBForward();
    }
    brake(1000);
    Servo1.write(75);
  }

  if (distanceCurrent == distanceChallange1 && challange1 == false && challange2 == false && challange3 == false && challange4) {
    challange4 = false;
    brake(1000);
    Servo1.write(170);
    while (distanceCurrent != distanceChallange1) {
      motorAForward();
      motorBBackward();
    }
    brake(1000);
    Servo1.write(75);
  }
  if (distanceCurrent == distanceChallangeFinal && challange1 == false && challange2 == false && challange3 == false && challange4 == false) {
    brake(20000);
  }
  //delay(500);}
}


void enableMotors()
{
  motorAOn();
  motorBOn();
}

void disableMotors()
{
  motorAOff();
  motorBOff();
}

void forward(int time)
{
  motorAForward();
  motorBForward();
  delay(time);
}


void backward(int time)
{
  motorABackward();
  motorBBackward();
  delay(time);
}

void turnLeft(int time)
{
  motorABackward();
  motorBForward();
  delay(time);
}

void turnRight(int time)
{
  motorAForward();
  motorBBackward();
  delay(time);
}

void coast(int time)
{
  motorACoast();
  motorBCoast();
  delay(time);
}

void brake(int time)
{
  motorABrake();
  motorBBrake();
  delay(time);
}

void motorAOn()
{
  digitalWrite(enableA, HIGH);
}

void motorBOn()
{
  digitalWrite(enableB, HIGH);
}


void motorAOff()
{
  digitalWrite(enableB, LOW);
}

void motorBOff()
{
  digitalWrite(enableA, LOW);
}


void motorAForward()
{
  digitalWrite(pinA1, HIGH);
  digitalWrite(pinA2, LOW);
}

void motorABackward()
{
  digitalWrite(pinA1, LOW);
  digitalWrite(pinA2, HIGH);
}


void motorBForward()
{
  digitalWrite(pinB1, HIGH);
  digitalWrite(pinB2, LOW);
}

void motorBBackward()
{
  digitalWrite(pinB1, LOW);
  digitalWrite(pinB2, HIGH);
}


void motorACoast()
{
  digitalWrite(pinA1, LOW);
  digitalWrite(pinA2, LOW);
}

void motorABrake()
{
  digitalWrite(pinA1, HIGH);
  digitalWrite(pinA2, HIGH);
}

void motorBCoast()
{
  digitalWrite(pinB1, LOW);
  digitalWrite(pinB2, LOW);
}

void motorBBrake()
{
  digitalWrite(pinB1, HIGH);
  digitalWrite(pinB2, HIGH);
}
