#include "motor_controller.h"
#include "motor.h"
#include "BTModule.h"
#include "delay.h"
#include "keyled.h"
#include "config.h"

MotorController motorController[MOTOR_NUM]; /* 0左轮  1右轮 */

/*
PIDParam    pidLow   = {MOTORCONTROLLER_KP_LOW,   MOTORCONTROLLER_KI_LOW,   MOTORCONTROLLER_KD_LOW  },
            pidMedia = {MOTOR_SPEED_LOOP_CONTROLLER_KP, MOTOR_SPEED_LOOP_CONTROLLER_KI, MOTOR_SPEED_LOOP_CONTROLLER_KD};
*/

int16_t  carSpeed;
int16_t  rotationSpeed;
int16_t  rotationAngle;
            
/**
 * @brief:  电机控制器初始化函数
 * @retval: None
 */
void MotorController_Init(void) 
{	
    rotationSpeed  = 0;
    rotationAngle  = 0;
    
	for(uint8_t i=0;i<MOTOR_NUM;i++){

        /* 速度环 */
        motorController[i].accelerate=MOTORCONTROLLER_ACC;
        motorController[i].setSpeed=0;
        motorController[i].curSpeed=0;
        motorController[i].speedLoopPID.output=PWM_DUTY_LIMIT/2;                   // PWM_DUTY_LIMIT/2
        
        motorController[i].speedLoopPID.pidParam.kp=MOTOR_SPEED_LOOP_CONTROLLER_KP;
        motorController[i].speedLoopPID.pidParam.ki=MOTOR_SPEED_LOOP_CONTROLLER_KI;
        motorController[i].speedLoopPID.pidParam.kd=MOTOR_SPEED_LOOP_CONTROLLER_KD;
        
        motorController[i].speedLoopPID.outMINLimit= 0;         // 0
        motorController[i].speedLoopPID.outMAXLimit= PWM_DUTY_LIMIT;         // PWM_DUTY_LIMIT

        /* 位置环 */
        // motorController[i].setPosition = 0;
        motorController[i].curPosition = 0;
        motorController[i].motorPostionRef = 0;
        // motorController[i].positionLoopPID.output = 0;

        // motorController[i].positionLoopPID.pidParam.kp = MOTOR_POSITION_LOOP_CONTROLLER_KP;
        // motorController[i].positionLoopPID.pidParam.ki = MOTOR_POSITION_LOOP_CONTROLLER_KI;
        // motorController[i].positionLoopPID.pidParam.kd = MOTOR_POSITION_LOOP_CONTROLLER_KD;
        
        // motorController[i].positionLoopPID.FilterPercent = 1.0;
        // motorController[i].positionLoopPID.integrationLimit = MOTORCONTROLLER_MAX_SEPPED_DEFAULT;

        // motorController[i].positionLoopPID.outMINLimit = 0 - MOTORCONTROLLER_MAX_SEPPED_DEFAULT;
        // motorController[i].positionLoopPID.outMAXLimit =     MOTORCONTROLLER_MAX_SEPPED_DEFAULT;
    }
}

/**
 * @brief  电机速度设置函数
 * @param  nMotor 电机编号
 * @param  nSpeed 线速度，单位：mm/s
 * @retval None
 */
void SetMotorSpeed(uint8_t nMotor, int16_t nSpeed)
{
    if( nMotor >= MOTOR_NUM){
        return ;
    }

    motorController[nMotor].setSpeed = nSpeed;
    if( Motor_GetState(1) == 1){
        motorController[nMotor].speedLoopPID.output=PWM_DUTY_LIMIT/2;
        Motor_Start(nMotor);
    }
}

/**
 * @brief  获取电机速度
 *         该速度从电机控制器中直接读取，若电机控制器速度闭环任务没有定时执行刷新速度缓存值则该函数读取的速度可能不实时。
 * @param  nMotor 电机编号 可以取0或1
 * @retval 电机线速度，单位：mm/s
 */
int16_t GetMotorSpeed(uint8_t nMotor)
{
    if( nMotor >= MOTOR_NUM){
        return 0;
    }
    return motorController[nMotor].curSpeed;
}

/**
 * @brief  设置电机位置
 *         该函数并不用于电机的位置控制，而是作为重置基准值使用，便于上层规划控制。
 * @param  nMotor   电机编号 可以取0或1
 * @param  nPostion 位置，单位：um
 * @retval None
 */
void SetMotorPosition(uint8_t nMotor, int32_t nPostion)
{
    if( nMotor >= MOTOR_NUM){
        return ;
    }

    motorController[nMotor].motorPostionRef = (motorController[nMotor].curPosition - nPostion);
}

/**
 * @brief  获取电机位置
 *         该位置从电机控制器中直接读取，若电机控制器闭环任务没有定时执行刷新位置缓存值则该函数读取的位置可能不实时。
 * @param  nMotor 电机编号 可以取0或1
 * @retval 电机位置，单位：um
 */
int32_t GetMotorPosition(uint8_t nMotor)
{
    if( nMotor >= MOTOR_NUM){
        return 0;
    }
    return (motorController[nMotor].curPosition - motorController[nMotor].motorPostionRef);
}

/**
 * @brief  设定电机闭环控制的PID参数
 * @param  kp 比例系数
 * @param  ki 积分系数
 * @param  kd 微分系数
 * @retval None
 */
void SetMotorPIDParam(float Kp,float Ki,float Kd)
{
    for(uint8_t i=0;i<MOTOR_NUM;i++){
        motorController[i].speedLoopPID.pidParam.kp = Kp;
        motorController[i].speedLoopPID.pidParam.ki = Ki;
        motorController[i].speedLoopPID.pidParam.kd = Kd;	
    }
}

/**
 * @brief:  更新电机当前速度
 * @retval: None
 */
void UpdateMotorCurSpeed(void){
    int16_t cnt; 
    for(uint8_t i=0;i<MOTOR_NUM;i++){
        cnt = Encoder_GetCNT(i);
        motorController[i].curSpeed = (PI * CAR_WHEEL_DIAMETER) * cnt * ( 1000 / MOTOR_SPEED_LOOP_CONTROLLER_PERIOD / 4 ) / ENCODER_RESOLUTION;   
    }
}

/**
 * @brief:  更新电机当前位置
 * @retval: None
 */
void UpdateMotorCurPostion(void){
    int64_t cnt;
    for(uint8_t i=0;i<MOTOR_NUM;i++){
        cnt = (int64_t)Encoder_GetEncCount(i);
        motorController[i].curPosition = (int32_t)((PI * CAR_WHEEL_DIAMETER) * cnt * ( 1000 / 4 ) / ENCODER_RESOLUTION);   
    }    
}


/**
 * @brief:  电机速度环控制任务
 * @retval: None
 */
uint8_t MotorSpeedLoopControlTask(uint8_t info){

    /* 更新电机速度 */
    UpdateMotorCurSpeed();

    /* 更新电机位置 */ 
    UpdateMotorCurPostion();

    for(uint8_t i=0;i<MOTOR_NUM;i++){

        /* 按加速度渐进变化期望电机速度 */    
        if(motorController[i].setSpeed>motorController[i].curSpeed){   

            motorController[i].expSpeed+=motorController[i].accelerate*MOTOR_SPEED_LOOP_CONTROLLER_PERIOD/1000;
            if(motorController[i].expSpeed>motorController[i].setSpeed){
                motorController[i].expSpeed=motorController[i].setSpeed;
            }
        }
        else if(motorController[i].setSpeed<motorController[i].curSpeed){

            motorController[i].expSpeed-=motorController[i].accelerate*MOTOR_SPEED_LOOP_CONTROLLER_PERIOD/1000;
            if(motorController[i].expSpeed<motorController[i].setSpeed){
                motorController[i].expSpeed=motorController[i].setSpeed;
            }
        }
        else{
            motorController[i].expSpeed=motorController[i].curSpeed;
        }

        //分段式PID
        //if(motorController[i].curSpeed<200){
        //    motorController[i].PID.pidParam=pidLow;
        //}else
        //    motorController[i].PID.pidParam=pidMedia;  
        
        //调用PID控制器
        INCPID_Update(&motorController[i].speedLoopPID, motorController[i].expSpeed,motorController[i].curSpeed);

        //uint16_t pwmOutput = 0 ;
        //PID->PWM函数映射
//        if(motorController[i].PID.output>0)
//            pwmOutput = 4000 - motorController[i].PID.output;
//        else 
//            pwmOutput = 6000 - motorController[i].PID.output;
//      
        //Motor_SetPWMDuty(i,pwmOutput);
        
        //更新PWM输出
        Motor_SetPWMDuty(i,motorController[i].speedLoopPID.output);
        
        
    }
    
    return 0;
}


/**
 * @brief:  电机位置环控制任务
 * @retval: None
 */
// uint8_t MotorPositionLoopControlTask(uint8_t info){
    
//     /* 更新电机位置 */
//     UpdateMotorCurPostion();
    
//     for(uint8_t i=0;i<MOTOR_NUM;i++){

//         /* 误差死区 */
//         if( Abs(motorController[i].setPosition - motorController[i].curPosition) < POSITION_ERROR_THRESHOLD ){
//             motorController[i].expPosition = motorController[i].curPosition;
//         }else{
//             motorController[i].expPosition = motorController[i].setPosition;
//         }
        
//         /* 调用PID控制器 */
//         POSPID_Update(&motorController[i].positionLoopPID, motorController[i].expPosition, motorController[i].curPosition, 1.0f * MOTOR_POSITION_LOOP_CONTROLLER_PERIOD / 1000);
        
//         /* 速度死区 */
//         if( Abs(motorController[i].positionLoopPID.output) < MOTORCONTROLLER_MIN_SEPPED_DEFAULT ){
//             motorController[i].positionLoopPID.output = 0;
//         } 
        
//         /* 位置环传递给速度环的给定速度 */
//         SetMotorSpeed(i,motorController[i].positionLoopPID.output);
//     }
    
//     return 0;
// }


/**
 * @brief  速度模型求解
 * @retval None
 */
static void SpeedModelSolution(void){
    int16_t rightSpeed;
    int16_t leftSpeed;
    
    rightSpeed = carSpeed + rotationSpeed;
    leftSpeed  = carSpeed - rotationSpeed;
    
    SetMotorSpeed(0,leftSpeed);
    SetMotorSpeed(1,rightSpeed);
}


/*
 ***************************************************************************
 * 常规外部接口函数
 *************************************************************************** 
 */

/**
 * @brief  获取购物机器人实时线速度 
 * @retval int16_t speed 线速度  单位：mm/s
 */
int16_t GetCarSpeed(void){
    int16_t iCarSpeed = (motorController[1].curSpeed + motorController[0].curSpeed)/2;
    return iCarSpeed;
}

int16_t GetCarSetSpeed(void){
    return carSpeed;
}

int16_t GetCarExpSpeed(void){
    return (motorController[1].expSpeed + motorController[0].expSpeed)/2;
}

/**
 * @brief  获取购物机器人实时角速度
 * @retval int16_t speed 角速度  单位：mm/s
 */
int16_t GetCarRotationSpeed(void){
     int16_t iCarRotationSpeed = (motorController[1].curSpeed - motorController[0].curSpeed)/2;
    return iCarRotationSpeed;
}

int16_t GetCarSetRotationSpeed(void){
    return rotationSpeed;
}

/**
 * @brief  设定购物机器人线速度
 * @param  speed 设定的速度值 单位：mm/s
 */
void SetCarSpeed(int16_t speed){

    carSpeed = speed;
    SpeedModelSolution();
}

/**
 * @brief  设定购物机器人角速度
 * @param  speed 设定的速度值 单位：mm/s
 */
void SetCarRotationSpeed(int16_t irotationSpeed){
   
    rotationSpeed  = irotationSpeed;
    SpeedModelSolution();
}


/**
 * @brief:  发送电机状态数据   调试专用
 * @retval: None
 */
void SendMotorData(void){
    
    uint8_t data[6+1];
    int16_t speed=0;
    speed = (motorController[0].curSpeed+motorController[1].curSpeed)/2;
    data[0]=0xA5;
    data[1]=(speed&0x00FF);
    data[2]=(speed&0xFF00)>>8;
    data[3]= motorController[0].setSpeed&0x00FF;
    data[4]=(motorController[0].setSpeed&0xFF00)>>8;
    data[5]=0;
    for(uint8_t i=1;i<5;i++)
        data[5]+=data[i];
    data[6]=0x5A;
    for(uint8_t i=0;i<=6;i++){
        USART_OUT(USART2,&data[i]);
    }
    
}

