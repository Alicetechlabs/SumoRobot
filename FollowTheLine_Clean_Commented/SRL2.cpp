
/************************************************************
SRL2.cpp - A Library for Controlling the MRK2 Sumo Robot League robot
Author - William J Ashby, PhD <washby@sumorobotleague.com>
This library is released into the public domain.
************************************************************/
#include "Arduino.h"
#include "SRL2.h"

#define pingPin 10
#define echoPin A0
#define leftSensor A1
#define rightSensor A2
#define rearSensor A3
#define buttonPin 2
#define buzzerPin 3
#define IREmitter 4
#define leftMotorSpeedPin 5
#define rightMotorSpeedPin 6
#define leftDirection 7
#define rightDirection 8
#define LED 13

bool debug2;
bool leftIsFlipped2; //If the left motor goes backwards when it is supposed to go forwards set to true
bool rightIsFlipped2; 
int lThresh2; //default threshold values inbetween black and white for the line sensors
int rThresh2;
int rearThresh2;
double tuningTurn2;
double tuningFwd2;
double tuningBwd2;

SRL2::SRL2() {
	debug2 = false;
	leftIsFlipped2 = true;
    rightIsFlipped2 = true; 
	lThresh2 = 500; //default threshold values inbetween black and white for the line sensors
	rThresh2 = 500;
	rearThresh2 = 500;
	tuningTurn2 = 1.0;
	tuningFwd2 = 1.0;
	tuningBwd2 = 1.0;
}

void SRL2::debug(int val){
	if(val == 0) {
		debug2 = false;
	}else{
		debug2 = true;
	}
}

void SRL2::flipMotorDirection(int mtr){
	if( mtr == 0 ){
		leftIsFlipped2 = !leftIsFlipped2;
	}
	if( mtr == 1 ){
		rightIsFlipped2 = !rightIsFlipped2;
	}
}

void SRL2::tuneForward(double x){
	tuningFwd2 = x;
}

void SRL2::tuneBackward(double x){
	tuningBwd2 = x;
}

void SRL2::tuneTurn(double x){
	tuningTurn2 = x;
}

void SRL2::jumpStartSetup(){
  Serial.begin(115200);
  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(leftSensor, INPUT);
  pinMode(rightSensor, INPUT);
  pinMode(rearSensor, INPUT);  
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);  
  pinMode(IREmitter, OUTPUT);
  pinMode(leftMotorSpeedPin, OUTPUT);
  pinMode(rightMotorSpeedPin, OUTPUT);  
  pinMode(leftDirection, OUTPUT);
  pinMode(rightDirection, OUTPUT);
  pinMode(LED, OUTPUT);
}

void SRL2::Forward(int amt){
  drive(80);  //turn the motors on forward 80% maximum speed
  delay(amt*50*tuningFwd2);  //wait
  drive(0);  //turn off the motors
}

void SRL2::Backward(int amt){
  drive(-80);  //turn the motors on forward 80% maximum speed
  delay(amt*50*tuningBwd2);  //wait
  drive(0);  //turn off the motors
}

void SRL2::Left(int deg){
  turnDegrees( -deg );
}

void SRL2::Right(int deg){
  turnDegrees( deg);
}

void SRL2::PD(){
  tone(buzzerPin, 3600, 800);
}

void SRL2::PU(){
  tone(buzzerPin, 2200, 400);
}

void SRL2::turnDegrees(int y) {
 
  if (y > 0) {
    setSpeedForMotor(200, 0);
    setSpeedForMotor(-200, 1);
 
  } else if (y < 0) {
    setSpeedForMotor(-200, 0);
    setSpeedForMotor(200, 1);
    
  } else {
	 if(debug2) Serial.println("0 degree turn");
  }
  delay( abs( (int) (tuningTurn2*9*y) ) );  
  setSpeedForMotor(0,0);
  setSpeedForMotor(0,1);
}

void SRL2::drive(int percentPower) {
  // map converts [-100,100] to [-255,255]
  int spd = map(percentPower, -100, 100, -255, 255);
  setSpeedForMotor(spd, 0);
  setSpeedForMotor(spd, 1);
}

void SRL2::setSpeedForMotor(int spd, int motor) {
  int speedPin, directionPin;
  bool flippedPin;
  if ( motor == 0 ) {
    speedPin = leftMotorSpeedPin;
    directionPin = leftDirection;
    if(leftIsFlipped2){
		spd = -spd;
	}
  } else if ( motor == 1 ) {
    speedPin = rightMotorSpeedPin;
    directionPin = rightDirection;
    if(!rightIsFlipped2){
       spd = -spd;
    }
  }

  if ( spd == 0 ) {
    analogWrite(speedPin, 0);
    digitalWrite(directionPin, LOW);
  } else if ( spd > 0 ) {
    digitalWrite(directionPin, HIGH);
    analogWrite(speedPin, abs(spd));
  } else if ( spd < 0 ) {
    digitalWrite(directionPin, LOW);
    analogWrite(speedPin, abs(spd));
  }
}

bool SRL2::objectWithin(int cm) {
  return getDistance() < cm;
}

int SRL2::getDistance() {
  long duration;
  int distance;
  digitalWrite(pingPin, LOW);// Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(pingPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (int) ( (duration / 2) / 29.1 );
  return distance;
}

//returns true if either front IR sensor detects white
//uses the thresholds for the left and right front IR sensors that
//are calculated via the calibrateBlackWhite function
bool SRL2::isWhite() {
  digitalWrite(IREmitter, HIGH);
  delayMicroseconds(2);
  int left = analogRead(leftSensor);
  int right = analogRead(rightSensor);
  if(debug2){
	  Serial.print("left: "); Serial.println(left);
	  Serial.print("right: "); Serial.println(right);
  }
  digitalWrite(IREmitter, LOW);
  if(debug2){
	  Serial.print("lThresh: "); Serial.println(lThresh2);
	  Serial.print("rThresh: "); Serial.println(rThresh2);
  }
  
  if (left < lThresh2 || right < rThresh2) {
    return true;
  } else return false;
}

//return the threshold value for the IR line sensors
int SRL2::getThreshold(int side) {
	if( side == 15){
		return lThresh2;
	}
	if( side == 16){
		return rThresh2;
	}
	if( side == 17){
		return rearThresh2;
	}
}

void SRL2::setThreshold(int side, int val) {
	if( side == 15){
		lThresh2 = val;
	}
	if( side == 16){
		rThresh2 = val;
	}
	if( side == 17){
		rearThresh2 = val;
	}
}

int SRL2::getIR(int side) {
	int returnMe;
	digitalWrite(IREmitter, HIGH);
	delayMicroseconds(2);
	returnMe = analogRead(side);
	digitalWrite(IREmitter, LOW);
	if(debug2) {
		Serial.print("IR reading: "); Serial.println(returnMe);
	}
	return returnMe;
}

//averages 5 black and 5 white IR readings for each front IR sensor
//then calculates the threshold between the balck and white readings
void SRL2::calibrateBlackWhite() {
  int n = 5;  //setup arrays to average n readings of the sensors on black and on white
  int lBlacks[n]; int rBlacks[n];  int rearBlacks[n];
  int lWhites[n]; int rWhites[n];  int rearWhites[n];

  Serial.println("\nPlace line sensors");
  Serial.println("on black and press button");
  tone(buzzerPin, 1500, 400);
  while (digitalRead(buttonPin) == 1) {
    delay(100);
  }

  digitalWrite(IREmitter, HIGH);
  for (int i = 0; i < n; i++) {
    lBlacks[i] = analogRead(leftSensor);
    rBlacks[i] = analogRead(rightSensor);
	rearBlacks[i] = analogRead(rearSensor);
  }
  digitalWrite(IREmitter, LOW);

  delay(2500);
  Serial.println("\nPlace line sensors");
  Serial.println(" on white and press button");
  tone(buzzerPin, 1800, 200);
  delay(250);
  tone(buzzerPin, 1800, 200);
  delay(250);

  while (digitalRead(buttonPin) == 1) {
    delay(100);
  }

  digitalWrite(IREmitter, HIGH);
  for (int i = 0; i < n; i++) {
    lWhites[i] = analogRead(leftSensor);
    rWhites[i] = analogRead(rightSensor);
    rearWhites[i] = analogRead(rearSensor);
  }
  digitalWrite(IREmitter, LOW);
  
  int lBlack = 0;  int rBlack = 0;  int rearBlack = 0;
  int lWhite = 0;  int rWhite = 0;  int rearWhite = 0;
  for (int i = 0; i < n; i++) {
    lBlack += lBlacks[i];
    rBlack += rBlacks[i];
	rearBlack += rearBlacks[i];
    lWhite += lWhites[i];
    rWhite += rWhites[i];
	rearWhite += rearWhites[i];
  }
  lBlack = lBlack / n; rBlack = rBlack / n;  rearBlack = rearBlack / n;
  lWhite = lWhite / n; rWhite = rWhite / n;  rearWhite = rearWhite / n;
  if(debug2){
	  Serial.print("Black averages: \nLeft "); Serial.println(lBlack); 
	  Serial.print("Right "); Serial.println(rBlack); 
	  Serial.print("Rear "); Serial.println(rearBlack);
	  Serial.print("White averages: \nLeft "); Serial.println(lWhite);
	  Serial.print("Right "); Serial.println(rWhite);
	  Serial.print("Rear "); Serial.println(rearWhite);
  }
  lThresh2 = lWhite/2 + lBlack/2;
  rThresh2 = rWhite/2 + rBlack/2;
  rearThresh2 = rearWhite/2 + rearBlack/2;

  delay(10);
}
