
/************************************************************
 *  SRL2.h - A Library for Controlling the MRK-2 Sumo Robot League robot
 *  Author - William J. Ashby, PhD <washby@sumorobotleague.com> This library is released into the public domain.
 *  ************************************************************
 */

#ifndef SRL2_h

#define SRL2_h
#include "Arduino.h"


class SRL2 {
  public:
    SRL2();
	bool debug2;
    bool leftIsFlipped2;
    bool rightIsFlipped2;
	void debug(int val);	
    void Forward(int amt); 
    void Backward(int amt);
    void Right(int deg);
    void Left(int deg);
    void PD();
    void PU();
    void drive(int percentPower);
    void setSpeedForMotor(int spd, int motor);
    void flipMotorDirection(int mtr);
    void turnDegrees(int y);
    bool objectWithin(int cm);
    int getDistance();
	int getIR(int side);
	int getThreshold(int side);
	void setThreshold(int side, int val);
    bool isWhite();
    void calibrateBlackWhite();
	void tuneForward(double x);
	void tuneBackward(double x);
	void tuneTurn(double x);
    void jumpStartSetup();


  private:
    int lThresh2;
    int rThresh2;
	int rearThresh2;
	double tuningTurn2;
	double tuningFwd2;
	double tuningBwd2;
    
};

#endif
