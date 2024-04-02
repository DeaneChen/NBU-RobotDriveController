#ifndef _ArmSolution_H_
#define _ArmSolution_H_
#include "main.h"

void ArmDriver_Init(void);
void SetServoAngle(int nServo,float angle);
uint8_t ServoTunnerOK(void);


void ArmSolution(double x,double y);
void Arm_Grab(void);
	
void Servo_init(uint8_t nServo,int angle);
void slowPwm(uint8_t nServo);

#endif


