/*
 * @Author       : LuHeQiu
 * @Date         : 2022-04-04 15:28:39
 * @LastEditTime : 2023-09-05 19:27:41
 * @LastEditors  : LuHeQiu
 * @Description  : 该控制器包含了对机械臂运动的细分平滑控制，可以适当借鉴，并不复杂。
 *                 由于常规舵机的从角度1运动到角度2需要时间，倘若在程序运行中不考虑这个过程，则会导致机械臂运行轨迹的顿挫。
 *                 合适的细分控制和路径规划可以使得机械臂移动更加稳定，平直。
 * @FilePath     : /NBU-RobotDriveController/Ext-Library-code/ArmController_for_arm1/arm_controller.h
 * @HomePage     : https://www.luheqiu.com
 */
#ifndef __ARM_CONTROLLER_H
#define __ARM_CONTROLLER_H

#include "stm32f4xx.h"

/*
 ************************************************
 * 机械臂物理结构校准及零点校准部分
 ************************************************
 */

/**
 *  @brief 机械臂各个结构的长度
 *         从左至右分别为  大臂  小臂  手腕关节至手掌中心的长度  手掌大小                     
 *         单位：m
 */
                            /*     大臂          小臂      手腕关节至手掌中心      手掌大小       虎口大小     手指等效张角比距  */
#define ARM_STRUCT_LENGTH   {     0.147f   ,    0.160f     ,    0.170f      ,      0.060f     ,    0.016f    ,     0.036f       }

/**
 *  @brief 舵机角度控制的转换系数
 *         单位：us/度
 */
/* 云台 servoID = 0 */
#define YAW_SERVO_US_PER_ANGLE       ( 8.105f)
/* 大臂 servoID = 1 */
#define BIG_ARM_SERVO_US_PER_ANGLE   (10.000f)
/* 小臂 servoID = 2 */
#define SMALL_ARM_SERVO_US_PER_ANGLE (11.589f)
/* 机械手 servoID = 3 */
#define HAND_SERVO_US_PER_ANGLE      (11.111f) 

/**
 *  @brief 机械臂关节角度零点偏移校准
 */
/* 云台 */
#define YAW_ARM_ANGLE_ZERO     ((  1320 ))
/* 大臂 */
#define BIG_ARM_ANGLE_ZERO      ((  700 ))
/* 小臂 */
#define SMALL_ARM_ANGLE_ZERO    ((  900 ))
/* 机械手 */
#define HAND_ARM_ANGLE_ZERO     (( 1000 ))

/**
 *  @brief 机械臂关节角度范围
 *         单位：度
 */
/* 云台范围 */
#define YAW_ANGLE_UP          (137.0f)
#define YAW_ANGLE_LOW         (-97.0f)
/* 大臂 */
#define BIG_ARM_ANGLE_UP      (128.0f)
#define BIG_ARM_ANGLE_LOW     (-15.0f)
/* 小臂 */
#define SMALL_ARM_ANGLE_UP    (116.0f)
#define SMALL_ARM_ANGLE_LOW   (-19.5f)
/* 两臂夹角 */
#define INC_ARM_ANGLE_UP      (160.0f)
#define INC_ARM_ANGLE_LOW     (25.0f)
/* 机械手 */
#define HAND_ANGLE_UP         (102.0f)
#define HAND_ANGLE_LOW        (5.0f)


/*
 ************************************************
 * 机械臂控制参数
 ************************************************
 */

/* 机械臂初始位置 单位mm       x       y      z   */
#define ARM_INIT_POSITION  {  -170  , 170 ,   0}
  
/**
 * @brief 机械臂移动细分最大值
 * 该参数决定了机械臂移动的精细程度，表示一次移动最多会被细分成多少次进行
 * 注意：细分度占用内存空间，若细分度过大可能导致内存不足，建议细分最大值在100以下。 
 */
#define ARM_MAX_DIVISION    (100)

/**
 * @brief 机械臂移动速度 单位 mm/s
 * 该参数决定了机械臂默认的移动速度，与细分最大值共同决定了一次移动的细分程度
 */
#define ARM_MOVE_SPEED      (230)  

/**
 * @brief 机械臂控制周期 单位 ms
 * 该参数决定了机械臂一次细分后的移动响应需要的时间
 * 舵机控制信号为T=20ms的脉冲，因此不建议控制周期小于1~2T
 * 如果不存在影子寄存器可能出现无法预料的结果
 */
#define ARM_CONTROLLER_PERIOD  (50)    //机械臂控制周期

/**
 * @brief 机械臂空间坐标向量
 *        坐标系以云台中心为原点，x轴水平向前，y轴水平向左，z轴竖直向上。
 *        云台绕z轴旋转。
 *        单位：mm
 */
typedef struct
{
    int16_t x, y, z;  
}ArmVector_t;

/**
 * @brief 四自由度机械臂每个关节的角度
 *        yaw为云台角度
 *        bigArm为大臂的关节角
 *        smallArm为小臂的关节角
 *        hand为机械手的关节角
 *        单位：rad
 */
typedef struct
{
    float yaw, bigArm, smallArm, hand;
}ArmAngle_t;

/**
 * @brief  机械臂正向运动学求解函数，通过各个关节角的角度计算机械臂末端所处的位置
 * @param  angle  关节角度结构体  作为函数的传入参数，求解函数将基于该结构体内的关节角度进行计算
 * @param  vector 位置向量结构体  作为函数的传出参数，包含机械臂末端位置向量
 * @retval isLegal 表征该关节角在物理上是否合法，当不合法时计算的位置将均为零。返回值为1是表示物理上合法。
 */
uint8_t CalArmVector(const ArmAngle_t *angle, ArmVector_t *vector);

/**
 * @brief  机械臂逆运动学求解函数，通过末端位置向量计算各个关节角的角度
 * @param  vector 位置向量结构体  作为函数的传入参数，求解函数将基于该结构体内的位置坐标进行计算
 * @param  angle  关节角度结构体  作为函数的传出参数，包含各个关节角的角度
 * @retval isreachable 表征该位置在物理上是否可达，当不可达时计算的角度将均为零。返回值为1是表示物理上可达。
 */
uint8_t CalArmAngle(const ArmVector_t *vector, ArmAngle_t *angle);

typedef enum{
    
    ARM_LINEAR_PATH = 0 ,
    ARM_CURVE_PATH
    
}ArmPlaning_e;

typedef enum{
    
    ARM_DILE = 0 ,
    ARM_READY,
    ARM_MOTION
    
}ArmState_e;

/**
 * @brief 机械臂控制器结构体
 *        用于记录机械臂的状态变量和各类输入输出变量 
 */
typedef struct{
    
    /* --- 状态控制部分 --- */

    /* 机械臂当前状态 */
    ArmState_e  armCurState;      

    /* 机械臂当前位置*/ 
    /* 为降低解算开销，该位置不记录移动过程中的位置，因此在移动过程中改变量不可靠 */  
    ArmVector_t armCurPostion;

    /* 机械臂当前关节角 */
    ArmAngle_t  armCurAngle;   

    /* 机械臂手指张合距离 */
    int16_t  handOpenDis;      
    
    uint8_t posFlag;

    /* --- 细分控制部分 --- */
    ArmAngle_t armAngleUnit[ARM_MAX_DIVISION];
    uint8_t totalNumber;
    uint8_t currentNumber;
    
}ArmController;

extern ArmController armController;

void Arm_Init(void);

/**
 * @brief  设置机械臂目标点函数
 * @param  vector 目标位置向量
 * @param  armPlan 路径规划方式 可以取值为 ARM_LINEAR_PATH 表示为线性路径
 *                                         ARM_CURVE_PATH 表示为曲线路径
 * @retval 目标是否可达。
 *         若目标不可达则反应0，且机械臂保持不动。
 */
uint8_t SetArmTarget(const ArmVector_t *vector, ArmPlaning_e armPlan);

/**
 * @brief  强制重置机械臂目标位置，该状态下不进行平滑移动，而是直接性的位置重置，细分度为1
 * @param  vector 目标位置向量
 * @retval 目标是否可达。
 *         若目标不可达则反应0，且机械臂保持不动。
 */
uint8_t EnforceArmTarget(const ArmVector_t *vector,const int16_t handOpenDis);

/**
 * @brief  计算手指张开角度，本函数将根据张开长度解算出对应的关节角
 * @param  handOpenLength 设置手张开的长度 单位 mm
 * @param  armAngle       关节角度结构体  
 *                        作为函数的传出参数，包含各个关节角的角度
 * @retval 目标是否可达。
 *         若目标不可达则反应0，且机械臂保持不动。
 */
uint8_t CalHandOpenAngle(const int16_t handOpenLength, ArmAngle_t* armAngle);

/**
 * @brief  设置手张开长度，本函数将根据张开长度解算出对应的关节角用于控制
 * @param  handOpenLength 设置手张开的长度
 * @retval 目标是否可达。
 *         若目标不可达则反应0，且机械臂保持不动。
 */
uint8_t SetHandOpenLength(int16_t handOpenLength);

/**
 * @brief  将机械臂关节角转换为舵机控制脉冲
 * @param  servoID 舵机ID，亦即关节ID
 * @param  angle   角度（弧度制）
 * @retval 脉冲宽度（单位us）
 */
uint16_t ArmAngleToPulse(uint8_t servoID, float angle);

/**
 * @brief:  获取机械臂当前状态
 * @retval: 机械臂状态 idle空闲/ ready就绪 / motion运动中
 */
ArmState_e GetArmState(void);

/**
 * @brief:  更新机械臂姿态任务，请将该函数在任务调度器或者相关多线程系统中
 *          以ARM_CONTROLLER_PERIOD为周期进行循环调用
 * @param:
 * @retval: 
 */
uint8_t UpdateArmPosTask(uint8_t info);

#endif
