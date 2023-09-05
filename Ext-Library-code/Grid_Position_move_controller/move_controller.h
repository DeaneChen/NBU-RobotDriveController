/*
 * @Author       : LuHeQiu
 * @Date         : 2022-04-04 15:28:39
 * @LastEditTime : 2023-09-05 19:38:46
 * @LastEditors  : LuHeQiu
 * @Description  : 基于移动位置距离和网格地图方向的移动控制器，可以移动任意粒度的距离，从而替换基于速度和十字路口判定的简单网格移动。
 *                 该部分仅供思路参考，调参和代码调整较为复杂，需要因地制宜。
 * @FilePath     : /NBU-RobotDriveController/Ext-Library-code/Grid_Position_move_controller/move_controller.h
 * @HomePage     : https://www.luheqiu.com
 */
#ifndef __MOVE_CONTROLLER_H
#define __MOVE_CONTROLLER_H

#include "stm32f4xx.h"
#include "config.h"
#include "function.h"

/**
 *  机器人位置环和角度环相关定义
 */
#define CAR_MOVE_LOOP_CONTROLLER_PERIOD  (10)      /* 购物车移动闭环控制周期 单位：ms */

#define CAR_POSITION_LOOP_CONTROLLER_KP  (3.6f)    /* PID控制器比例系数 */  
#define CAR_POSITION_LOOP_CONTROLLER_KI  (0.0f)    /* PID控制器积分系数 */
#define CAR_POSITION_LOOP_CONTROLLER_KD  (0.9f)    /* PID控制器微分系数 */

#define CAR_ANGLE_LOOP_CONTROLLER_KP     ( 3.4f * CAR_EQUAL_DIAMETER )  /* PID控制器比例系数 */  
#define CAR_ANGLE_LOOP_CONTROLLER_KI     ( 0.0f * CAR_EQUAL_DIAMETER )  /* PID控制器积分系数 */
#define CAR_ANGLE_LOOP_CONTROLLER_KD     ( 0.9f * CAR_EQUAL_DIAMETER )  /* PID控制器微分系数 */

#define CAR_MAX_LINEAR_SPEED             (400)        /* 购物车线速度最大值 mm/s */
#define CAR_DEFAULT_BACK_LINEAR_SPEED    (150)        /* 购物车默认倒车线速度 mm/s */ 
 
#define CAR_MAX_ANGLE_SPEED              (200)        /* 购物车角速度最大值 mm/s */
 
#define CAR_POSITION_ERROR_THRESHOLD     (10)         /* 位置误差阈值 */
#define CAR_ANGLE_ERROR_THRESHOLD        (0.025f)     /* 角度误差阈值 */

#define GRID_DISTANCE           (BLACK_BLOCK_WIDTH)  /* 单位坐标长度，即网格间距 单位mm  400mm  禁止为零，否则可能产生不可预料的错误 */ 
#define POS_UPDATE_TOL          (150)                /* 位置校正更新容忍值  单位mm */ 

/* 小车初始位置 */
#define CAR_PLACE_POS_UNIT      ((MovePosUnitVector_t){400 * 8.0f, 190})
#define CAR_PLACE_DIR_UNIT      ((MovePosUnitVector_t){0.0f, 1.0f})
#define CAR_PLACE_POSVECTOR     ((CarPosVector_t){(CAR_PLACE_POS_UNIT),(CAR_PLACE_DIR_UNIT)})

#define USE_GRID_POS_SYSTEM  1                       /* 使用网格位置系统，计算位置时将方向向量被近似为四个方向，使得小车位置为矩形网格系统 */ 
#define USE_GRID_DIR_SYSTEM  0                       /* 使用网格方向系统，方向计算是直接被近似为四个方向 */

/**
 * @brief 移动控制器向量结构体
 */
typedef struct {
    float x, y;
}MovePosUnitVector_t;

/**
 * @brief 移动位矢，包含位置和方向向量
 */
typedef struct{
    MovePosUnitVector_t position;
    MovePosUnitVector_t direction;
}CarPosVector_t;

/**
 * @brief 机器人移动状态
 */
typedef enum{
    CAR_STOP = 0,
    CAR_MOVE
}CarMoveState_e;

/**
 * @brief 机器人循迹状态
 */
typedef enum{
    CAR_CENTER_TRACE = 0,
    CAR_LEFT_TRACE,
    CAR_NONE_TRACE
}CarTraceState_e;

/**
 * @brief 机器人姿态状态
 */
typedef enum{
    CAR_NORMAL_POS = 0,
    CAR_FIXING_POS,
}CarPosState_e;

typedef struct{

    int32_t curMicroPosition; /* 当前实时位置 单位：um */

    int32_t curPosition;      /* 当前实时位置 单位：mm */
    int32_t setPosition;      /* 设定位置 单位：mm */
    int32_t expPosition;      /* 瞬时期望位置 单位：mm */
    
    float curAngel;          /* 当前实时角度 单位：rad */
    float setAngel;          /* 设定角度 单位：rad */
    float expAngel;          /* 瞬时期望位置  单位：rad */

    POSPIDController positionLoopPID;  /* 位置环PID控制器 */
    POSPIDController angleLoopPID;     /* 角度环PID控制器 */
}MoveLoopController_t;


typedef struct{
    
    /* 小车移动状态 */
    CarMoveState_e carMoveState;
    CarTraceState_e carTraceState;
    CarPosState_e carPosState;

    CarPosVector_t carPos;

    MoveLoopController_t carDivMoveController; /* 增量位置控制器 */

}MoveController_t;

extern MoveController_t moveController;

void MoveControllerInit(void);

/**
 * @brief:  电机移动闭环控制任务
 *          应在调度系统中以 Car_MOVE_LOOP_CONTROLLER_PERIOD 时间为周期严格定时调用
 *          调用时间的误差会影响电机的控制性能
 */
extern uint8_t CarMoveLoopControlTask(uint8_t info);

/**
 * @brief  设置小车循迹状态
 * @param  traceState 目标循迹状态 
 *                    可选 CAR_CENTER_TRACE 中央循迹方式,
                           CAR_LEFT_TRACE   偏左循迹方式
 * @retval 
 */
extern void CarSetTraceState(CarTraceState_e traceState);

/**
 * @brief  小车移动命令函数
 * @param  distance 距离，小车移动的距离
 * @param  angle    角度，小车转弯的角度
 * @retval none
 */
extern void CarMove(int32_t distance, float angle);


/**
 * @brief  基于网格的小车移动命令函数，该函数会使得小车在直行方向对齐至网格
 * @param  count 网格刻度，小车对齐的网格刻度值
 * @param  angle 角度，小车转弯的角度
 * @retval none
 */
extern void CarMoveWithGrid(int16_t count, float angle);


/**
 * @brief  设置小车速度限制
 * @param  linearSpeed 线速度 单位mm/s
 * @param  angleSpeed  角速度 单位mm/s
 * @retval 设置状态 1 设置成功
 *                  0 速度值非法，超过移动控制器限制（多为电机限制值）
 */
uint8_t CarSetSpeedLimit(int16_t linearSpeed, int16_t invLinearSpeed, int16_t angleSpeed);


/**
 * @brief  基于曲线的小车移动命令函数，该函数会使得小车沿圆弧曲线行驶。
 * @param  radius 转弯半径，亦即圆弧半径。该命令基于极坐标，因此取正时为左转，取负时为右转。 单位：mm
 * @param  angle  转弯角度，取正时为前进，取负时为倒车。 单位：rad
 * @retval 
 */
extern void CarMoveWithCurve(int16_t radius, float angle);


/**
 * @brief  获取小车坐标
 * @retval 小车坐标指针
 */
extern const CarPosVector_t* GetCarPosVector(void);


/**
 * @brief  获取小车运动状态
 * @retval 运动状态
 */
extern CarMoveState_e GetCarMoveState(void);



#define MOVE_CONTROLLER_PERIOD  5                    //位置检测周期  单位mm

#define ROATION_UPDATE_TOL      26                   //方向矢量更新容忍值 单位度
#define ROATION_ADJUST_TOL      6                    //方向矢量校正容忍值 
//#define ROATION_DEC_VALUE       10     //旋转微调量

#define POINT_SCAN_FILTER       2      //坐标检测滤波系数

#define MAP_SIZE 12                            //地图尺寸

#define MAP {\
              {7,7,7,7,7,7,7,7,7,7,7,7}, \
              {7,1,0,0,1,1,1,1,1,1,1,7}, \
              {7,1,0,0,0,0,0,0,0,0,0,7}, \
              {7,1,0,0,0,0,0,0,0,0,0,7}, \
              {7,1,0,0,1,1,1,1,0,0,1,7}, \
              {7,1,0,0,1,1,1,1,0,0,1,7}, \
              {7,1,0,0,1,1,1,1,0,0,1,7}, \
              {7,1,0,0,1,1,1,1,0,0,1,7}, \
              {0,0,0,0,0,0,0,0,0,0,1,7}, \
              {7,6,0,0,0,0,0,0,0,0,1,7}, \
              {7,1,1,1,1,1,1,1,0,0,1,7}, \
              {7,7,7,7,7,7,7,7,7,7,7,7}  }     //地图

//#define CAR_PLACE_POSVECTOR {{8,0},{0,1}}      //机器人放置点
//#define CAR_START_POSVECTOR {{8,1},{0,1}}      //机器人起始点
#define CAR_BUSY_POSVECTOR  {{8,1},{0,-1}}      //机器人购买点    //{{8,1},{0,-1}}

#define PATH_MAX_LENGTH 100                    //路径最大长度

#define TRACK_SENSOR_MID 72
#define TRACK_CONTROLLER_PERIOD  20

#define CAR_MOVE_SPEED     400  
#define CAR_MOVE_SPEED_LOW 300  







//uint8_t	PointScan(void);
//uint8_t UpdateCarCurPosTask(uint8_t info);
//void PosVectorCalibrate(void);
//uint8_t PathPlanning(PosVector_t targetPos);
//PosVector_t GetCarPos(void);
//uint8_t CarTrackingTask(uint8_t info);
//uint8_t PathFollowingTask(uint8_t info);

//uint8_t CarGoCertainDistance(int16_t distance);
//uint8_t CarGoCertainAngle(int16_t angle);
//uint8_t IsCarPosEqual(PosVector_t pos1 , PosVector_t pos2);

//void SetCarPos(PosVector_t pos);

#endif
