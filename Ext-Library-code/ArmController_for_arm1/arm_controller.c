#include "arm_controller.h"
#include "servo.h"
#include "function.h"
#include "math.h"

float armLength[6] = ARM_STRUCT_LENGTH;

ArmController armController;

   
/**
 * @brief:  位置向量相等判断
 * @param:  none
 * @retval: 
 */                         
// uint8_t IsArmPosEqual(ArmPos_t pos1 , ArmPos_t pos2){
//     return ( (pos1.servoPWM[0]==pos2.servoPWM[0]) && (pos1.servoPWM[1]==pos2.servoPWM[1]) && (pos1.servoPWM[2]==pos2.servoPWM[2]) && (pos1.servoPWM[3]==pos2.servoPWM[3]) );
// }

/**
 * @brief  机械臂正向运动学求解函数，通过各个关节角的角度计算机械臂末端所处的位置
 * @param  angle  关节角度结构体 
 *                作为函数的传入参数，求解函数将基于该结构体内的关节角度进行计算
 * @param  vector 位置向量结构体  
 *                作为函数的传出参数，包含机械臂末端位置向量
 * @retval isLegal 表征该关节角在物理上是否合法，当不合法时计算的位置将均为零。返回值为1是表示物理上合法。
 */
uint8_t CalArmVector(const ArmAngle_t* angle, ArmVector_t* vector){

    /* 变量记录与定义 */
    float lambda =  angle->yaw;
    float bigArmAngle = angle->bigArm;
    float smallArmAngle = angle->smallArm;
    float phi3 = PI - (bigArmAngle + smallArmAngle);

    /* 关节角度物理合法性检验 */  
    if ( (      lambda < YAW_ANGLE_LOW / 180.0f * PI         ||           lambda > YAW_ANGLE_UP / 180.0f * PI ||
          bigArmAngle < BIG_ARM_ANGLE_LOW / 180.0f * PI     || bigArmAngle > BIG_ARM_ANGLE_UP / 180.0f * PI || 
          phi3 < INC_ARM_ANGLE_LOW / 180.0f * PI            ||      phi3 > INC_ARM_ANGLE_UP / 180.0f * PI ||
          smallArmAngle < SMALL_ARM_ANGLE_LOW / 180.0f * PI ||   smallArmAngle >  SMALL_ARM_ANGLE_UP / 180.0f * PI) ){
              
        vector->x = 0;
        vector->y = 0;
        vector->z = 0;
        return 0;
    }

    /* 公式计算末端位置向量 */  
    float distance = armLength[0] * cosf(bigArmAngle) + armLength[1] * cosf(smallArmAngle) + armLength[2];
    vector->x = (int16_t)(distance * cosf(lambda) * 1000);
    vector->y = (int16_t)(distance * sinf(lambda) * 1000);
    vector->z = (int16_t)((armLength[0] * sinf(bigArmAngle) - armLength[1] * sinf(smallArmAngle)) * 1000);
    return 1;
}

/**
 * @brief  机械臂逆运动学求解函数，通过末端位置向量计算各个关节角的角度
 * @param  vector 位置向量结构体  
 *                作为函数的传入参数，求解函数将基于该结构体内的位置坐标进行计算
 * @param  angle  关节角度结构体  
 *                作为函数的传出参数，包含各个关节角的角度
 * @retval isreachable 表征该位置在物理上是否可达，当不可达时计算的角度将均为零。返回值为1是表示物理上可达。
 */
uint8_t CalArmAngle(const ArmVector_t* vector, ArmAngle_t* angle){

    /* 向量单位转换 */
    float x = 1.0f * (vector->x) / 1000;
    float y = 1.0f * (vector->y) / 1000;
    float z = 1.0f * (vector->z) / 1000;

    /* 计算四象限反正切 求得 lambda */
    float lambda = atan2f(y, x);

    /* 推算第二关节角度 */
    float x2 = x - armLength[2] * cosf(lambda);
    float y2 = y - armLength[2] * sinf(lambda);
    float z2 = z;

    /* 计算第二关节位置向量的模长 */
    float D1 = sqrtf(x2 * x2 + y2 * y2 + z2 * z2);

    /* 正弦定理计算关节夹角 */
    float phi1 = acosf((armLength[0] * armLength[0] + D1 * D1 - armLength[1] * armLength[1]) / (2 * armLength[0] * D1));
    float phi2 = atan2f(z2, sqrtf(x2 * x2 + y2 * y2));
    float phi3 = acosf((armLength[0] * armLength[0] + armLength[1] * armLength[1] - D1 * D1) / (2 * armLength[0] * armLength[1]));

    /* 计算大臂关节角 */
    float bigArmAngle = phi1 + phi2;

    /* 计算小臂关节角 */
    float smallArmAngle = PI - bigArmAngle - phi3;

    /* 关节角度物理可行性检验 */    
    if ( (      lambda < YAW_ANGLE_LOW / 180.0f * PI         ||           lambda > YAW_ANGLE_UP / 180.0f * PI ||
          bigArmAngle < BIG_ARM_ANGLE_LOW / 180.0f * PI     || bigArmAngle > BIG_ARM_ANGLE_UP / 180.0f * PI ||
                 phi3 < INC_ARM_ANGLE_LOW / 180.0f * PI     ||      phi3 > INC_ARM_ANGLE_UP / 180.0f * PI || 
          smallArmAngle < SMALL_ARM_ANGLE_LOW / 180.0f * PI ||   smallArmAngle >  SMALL_ARM_ANGLE_UP / 180.0f * PI) ){
        
        /* 若物理不可行则置零结果值再返回 */
        angle->yaw = 0;
        angle->bigArm = 0;
        angle->smallArm = 0;

        /* 物理不可行则返回值为零 */
        return 0;
    }

    /* 保存结果值 */
    angle->yaw = lambda;
    angle->bigArm = bigArmAngle;
    angle->smallArm = smallArmAngle;

    return 1;
}

/**
 * @brief:  机械臂初始化
 * @param:  none
 * @retval: 
 */
void Arm_Init(void){
    
    Servo_Init();
    
    ArmVector_t armPos = ARM_INIT_POSITION;
    ArmAngle_t armAng;

    /* 初始位置校验不通过 */
    if(!CalArmAngle(&armPos, &armAng)){
        ServoCalibratedError();
    }

    EnforceArmTarget(&armPos, armLength[4] * 1000);

    ServoEnable(1);

    armController.armCurState = ARM_DILE;
}


/**
 * @brief  设置机械臂目标点函数
 * @param  vector 目标位置向量
 * @retval 目标是否可达。
 *         若目标不可达则返回0，且机械臂保持不动。
 *         若目标可达则返回1，表示设置成功。
 * 
 *         错误代码 2  表示机械臂正在移动中，请等待移动完成 
 *            
 */
uint8_t SetArmTarget(const ArmVector_t *vector, ArmPlaning_e armPlan){

    ArmAngle_t armAngle = armController.armCurAngle;

    /* 机械臂正在运动 */
    if(armController.armCurState != ARM_DILE){
        return 2;
    }

    /* 位置相同则不需要移动 */
    if( vector->x == armController.armCurPostion.x && 
        vector->y == armController.armCurPostion.y &&
        vector->z == armController.armCurPostion.z  ){

        armController.armCurState = ARM_READY;
        return 1;
    }

    /* 目标检验不通过，目标不可达 */
    if(!CalArmAngle(vector,&armAngle)){
        return 0;
    }

    switch (armPlan){

        case ARM_LINEAR_PATH: {
            
            /* 计算移动距离 */
            int32_t distance = (int32_t)sqrtf(powf((float)(vector->x - armController.armCurPostion.x), 2) +
                                            powf((float)(vector->y - armController.armCurPostion.y), 2) +
                                            powf((float)(vector->z - armController.armCurPostion.z), 2));

            /* 计算细分度 */
            /* 细分度 = 需要的控制周期数 = 距离 / 速度 / 每周期时间 */
            uint8_t div = (uint8_t)(distance * 1000 / ARM_MOVE_SPEED / ARM_CONTROLLER_PERIOD);

            /* 细分度最大值幅限 */
            if(div > ARM_MAX_DIVISION){
                div = ARM_MAX_DIVISION;
            }

            /* 细分角度分配 */
            ArmVector_t vectorUnit;
            ArmAngle_t angleUnit;
            armController.totalNumber = 0; /* 细分控制栈置空 */
            for (uint8_t i = 1; i <= div;i++){
                
                vectorUnit.x = (int16_t)((int32_t)(vector->x - armController.armCurPostion.x) * i / div) + armController.armCurPostion.x;
                vectorUnit.y = (int16_t)((int32_t)(vector->y - armController.armCurPostion.y) * i / div) + armController.armCurPostion.y;
                vectorUnit.z = (int16_t)((int32_t)(vector->z - armController.armCurPostion.z) * i / div) + armController.armCurPostion.z;
                
                /* 路径检验不通过，路径有障碍 */
                if(!CalArmAngle(&vectorUnit,&angleUnit)){
                    return 0;
                }

                armController.armAngleUnit[armController.totalNumber++] = angleUnit;
            }

            armController.currentNumber = 0;

            break;
        }

        case ARM_CURVE_PATH:{

            /* 计算四象限反正切 求得 Lambda */
            float currentLambda = atan2f(armController.armCurPostion.y, armController.armCurPostion.x);
            float targetLambda = atan2f(vector->y, vector->x);

            /* 计算臂长投影 */
            float armLenthInXOY = sqrtf(powf(armController.armCurPostion.x, 2) + powf(armController.armCurPostion.y, 2));

            /* 计算圆弧细分度 */
            /* 细分度 = 需要的控制周期数 = 弧长 / 速度 / 每周期时间 */
            uint16_t curveDiv = (uint8_t)(fabs(targetLambda - currentLambda) * armLenthInXOY / 2 * 1000 / ARM_MOVE_SPEED / ARM_CONTROLLER_PERIOD);

            /* 计算中间位置 */
            ArmAngle_t midAngle = {targetLambda, armController.armCurAngle.bigArm, armController.armCurAngle.smallArm, armController.armCurAngle.hand};
            ArmVector_t midVector;
            if(!CalArmVector(&midAngle, &midVector)){  /* 路径检验不通过，路径有障碍 */
                return 0;
            }

            /* 计算直线距离 */
            int32_t distance = (int32_t)sqrtf(powf((float)(vector->x - midVector.x), 2) +
                                              powf((float)(vector->y - midVector.y), 2) +
                                              powf((float)(vector->z - midVector.z), 2));

            /* 计算直线细分度 */
            uint16_t div = (uint8_t)(distance * 1000 / ARM_MOVE_SPEED / ARM_CONTROLLER_PERIOD);         

            /* 细分度最大值幅限 */
            if(curveDiv + div > ARM_MAX_DIVISION){
                curveDiv = curveDiv  * ARM_MAX_DIVISION / (curveDiv + div) ;
                div = ARM_MAX_DIVISION - curveDiv;
            }

            /* 细分角度分配 */
            ArmVector_t vectorUnit;
            ArmAngle_t angleUnit;
            armController.totalNumber = 0; /* 细分控制栈置空 */

            angleUnit.bigArm = armController.armCurAngle.bigArm;
            angleUnit.smallArm = armController.armCurAngle.smallArm;
            angleUnit.hand = armController.armCurAngle.hand;
            for (uint8_t i = 1; i <= curveDiv; i++){

                angleUnit.yaw = (targetLambda - currentLambda) * i / curveDiv + currentLambda;
                
                /* 路径检验不通过，路径有障碍 */
                if(!CalArmVector(&angleUnit,&vectorUnit)){
                    return 0;
                }

                armController.armAngleUnit[armController.totalNumber++] = angleUnit;
            }

            for (uint8_t i = 1; i <= div;i++){
                
                vectorUnit.x = (int16_t)((int32_t)(vector->x - midVector.x) * i / div) + midVector.x;
                vectorUnit.y = (int16_t)((int32_t)(vector->y - midVector.y) * i / div) + midVector.y;
                vectorUnit.z = (int16_t)((int32_t)(vector->z - midVector.z) * i / div) + midVector.z;
                
                /* 路径检验不通过，路径有障碍 */
                if(!CalArmAngle(&vectorUnit,&angleUnit)){
                    return 0;
                }

                armController.armAngleUnit[armController.totalNumber++] = angleUnit;
            }

            armController.currentNumber = 0;

            break;
        }
    
        default:
            break;
    }

    /* 机械臂运动准备好 */
    armController.armCurState = ARM_READY;
    armController.armCurPostion = *vector;
    armController.armCurAngle = armAngle;

    return 1;
}

/**
 * @brief  强制重置机械臂目标位置，该状态下不进行平滑移动，而是直接性的位置重置，细分度为1
 * @param  vector 目标位置向量
 * @param  handOpenDis 手指张开大小 单位：mm
 * @retval 目标是否可达。
 *         若目标不可达则反应0，且机械臂保持不动。
 */
uint8_t EnforceArmTarget(const ArmVector_t *vector,const int16_t handOpenDis){

    ArmAngle_t armAngle;

    /* 目标检验不通过，目标不可达 */
    if(!CalArmAngle(vector,&armAngle) || !CalHandOpenAngle(handOpenDis,&armAngle) ){
        return 0;
    }

    /* 设置机械臂位置 */
    SetServoPWM(0,ArmAngleToPulse(0, armAngle.yaw));
    SetServoPWM(1,ArmAngleToPulse(1, armAngle.bigArm));
    SetServoPWM(2,ArmAngleToPulse(2, armAngle.smallArm));
    SetServoPWM(3,ArmAngleToPulse(3, armAngle.hand));
    
    
    /* 状态更新 */
    armController.armCurPostion = *vector;
    armController.armCurAngle = armAngle;
    armController.handOpenDis = handOpenDis;

    return 1;
}


/**
 * @brief  计算手指张开角度，本函数将根据张开长度解算出对应的关节角
 * @param  handOpenLength 设置手张开的长度 单位 mm
 * @param  armAngle       关节角度结构体  
 *                        作为函数的传出参数，包含各个关节角的角度
 * @retval 目标是否可达。
 *         若目标不可达则反应0，且机械臂保持不动。
 */
uint8_t CalHandOpenAngle(const int16_t handOpenLength, ArmAngle_t* armAngle){
    
    /* 手指距离等于 =  手掌张开距离 - 虎口大小 */
    float dx = 1.0f * handOpenLength / 1000 - armLength[4];

    /* 计算手指张角 */
    float handAngle = PI / 2 - asinf(dx / 2 / armLength[5]); 

    if( handAngle < HAND_ANGLE_LOW / 180.0f * PI  || handAngle > HAND_ANGLE_UP / 180.0f * PI ){
        return 0;
    }

    armAngle->hand = handAngle;

    return 1;
}


/**
 * @brief  设置手指张开长度，本函数将根据张开长度解算出对应的关节角并进行控制
 * @param  handOpenLength 设置手张开的长度 单位 mm
 * @retval 目标是否可达。
 *         若目标不可达则反应0，且机械臂保持不动。
 */
uint8_t SetHandOpenLength(const int16_t handOpenLength){
    
    ArmAngle_t armAngle;

    if( !CalHandOpenAngle(handOpenLength,&armAngle) ){
        return 0;
    }

    armController.handOpenDis = handOpenLength;
    armController.armCurAngle.hand = armAngle.hand;

    /* 直接驱动 */
    SetServoPWM(3, ArmAngleToPulse(3, armAngle.hand));

    return 1;
}


/**
 * @brief  将机械臂关节角转换为舵机控制脉冲
 * @param  servoID 舵机ID，亦即关节ID
 * @param  angle   角度（弧度制）
 * @retval 脉冲宽度（单位us）
 */
uint16_t ArmAngleToPulse(uint8_t servoID, float angle){

    uint16_t pulse = 0;

    switch(servoID){
        case 0 :
            pulse = (angle) / PI * 180  * YAW_SERVO_US_PER_ANGLE + YAW_ARM_ANGLE_ZERO;
            break;
        case 1 :
            pulse = (angle) / PI * 180 * BIG_ARM_SERVO_US_PER_ANGLE + BIG_ARM_ANGLE_ZERO;
            break;
        case 2 :
            pulse = (angle) / PI * 180 * SMALL_ARM_SERVO_US_PER_ANGLE + SMALL_ARM_ANGLE_ZERO;
            break;
        case 3 :
            pulse = (angle) / PI * 180 * HAND_SERVO_US_PER_ANGLE + HAND_ARM_ANGLE_ZERO;
            break;
        default:
            break;
    }

    return pulse;
}

/**
 * @brief:  更新机械臂姿态任务，请将该函数在任务调度器或者相关多线程系统中
 *          以ARM_CONTROLLER_PERIOD为周期进行循环调用
 * @param:
 * @retval: 
 */
uint8_t UpdateArmPosTask(uint8_t info){
    
    if(info==0)
        return 0;
    
    /* 机械臂空闲，无需移动 */
    if(armController.armCurState == ARM_DILE){
        return 0;
    }

    /* 机械臂准备好状态 变更为移动状态 */
    if(armController.armCurState == ARM_READY){
        armController.armCurState = ARM_MOTION;
    }

    if(armController.armCurState == ARM_MOTION){

        if(armController.currentNumber < armController.totalNumber ){

            /* 设置机械臂位置 */
            SetServoPWM(0,ArmAngleToPulse(0, armController.armAngleUnit[armController.currentNumber].yaw));
            SetServoPWM(1,ArmAngleToPulse(1, armController.armAngleUnit[armController.currentNumber].bigArm));
            SetServoPWM(2,ArmAngleToPulse(2, armController.armAngleUnit[armController.currentNumber].smallArm));
            //SetServoPWM(3,ArmAngleToPulse(3, armController.armAngleUnit[armController.currentNumber].hand));

            /* 状态更新 */
            armController.armCurAngle = armController.armAngleUnit[armController.currentNumber++];
        }else{

            /* 移动完成，机械臂进入空闲状态 */
            armController.armCurState = ARM_DILE;
        }
    }   
    
    return 0;
}

/**
 * @brief:  获取机械臂当前状态
 * @retval: 机械臂状态  ready就绪 / motion运动中
 */
ArmState_e GetArmState(void){
    return (armController.armCurState);
}



