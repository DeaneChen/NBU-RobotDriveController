#ifndef _ArmSolution_H_
#define _ArmSolution_H_

void ArmDriver_Init(void);
void SetServoAngle(int nServo,float angle);
uint8_t ServoTunnerOK(void);


void ArmSolution(double x,double y);
void Arm_Grab(void);
	

void slowPwm(uint8_t nServo);

#endif


