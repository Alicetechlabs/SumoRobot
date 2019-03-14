//This is a comment, comment aren't process and will help you to remember what your code does

//'#include' will load existing libraries. Libraries contain functions and such to help you (for example 'delay' or 'random')
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "SRL2.h" 	


//'#define' is used to give a name to your pins. It's important for readability.
#define buttonPin 2
#define buzzerPin 3
#define LEDpin 13
#define leftSensor A1
#define rightSensor A2
#define rearSensor A3
#define IREmitter 4


//Here we create variable that are of the 'double' type.
double angle_rad = PI/180.0;        //Thanks to these you can 'talk' to your robot in angle/degree
double angle_deg = 180.0/PI;
SRL2 bot;


//'void setup' is only run once. In here you usually put information that is never going to change.
void setup(){
    
    bot.jumpStartSetup();         //Function that can be found in the SRL2.cpp tab. It defines the mode of each pin
    Serial.begin(9600);           //This tells the robot that we want him to write to us on the Serial Monitor 9600
    pinMode(leftSensor, INPUT);   //Fuctions that defines the mode of the pins. First argument is the name of the pin, second argument is either INPUT or OUTPUT
    pinMode(rightSensor, INPUT);
    pinMode(rearSensor, INPUT);
    pinMode(IREmitter, OUTPUT);

}


//'void loop' will loop forever unless told otherwise. In here is put the code that we want to react to the environment.
void loop(){

    //The following 5 lines will make the infrared sensor 'pulse' to give us the information on the 'color' (given in number) it's sensing
    digitalWrite(IREmitter, HIGH);              //digitalWrite and analogWrite will power(HIGH) or shutdown(LOW) a designated pin (first argument)
    int rightInput = analogRead(rightSensor);   //int in front of a 'name' will create a variable that can handle non decimal number within -32 768 and 32 767
    int leftInput = analogRead(leftSensor);     //analogRead and digitalRead will 'read' the state of the pin. analog can range from 0 to 1024 (max differs from board), digital can only in two state (HIGH/1 or LOW/0)
    int rearInput = analogRead(rearSensor);
    delay(500);                                 //Makes your robot wait here for 500ms (0.5s). Very usefull as the robot process faster that us.

    bot.setSpeedForMotor(180,0);      //We use here an already existing function from one of our library that will define the speed of the motor (first argument) and which motor is concerned (second argument)
    bot.setSpeedForMotor(180,1);      //Right motor = 1
    
   
   //Next 4 'if' test for the value the sensor are getting and 'do' so that the robot follows the line
    if ((rightInput > 200) && (rearInput > 200)){
     bot.Right(15);                   //Using an existing function from our library, makes the robot turn on a given angle (here 15)
    }
    if ((leftInput > 200) && (rearInput > 200)){
       bot.Left(15);
     }
    if ((rightInput > 200)){
       bot.Right(15);
     }
     if ((leftInput > 200)){
       bot.Left(15);
     }
    _loop();          //Calls the 'void _loop' function which is empty, so it does nothing. But we can't erase it as the libraries we imported uses it.
}

void _delay(float seconds){
    long endTime = millis() + seconds * 1000;
    while(millis() < endTime)_loop();
}

void _loop(){
    
}
