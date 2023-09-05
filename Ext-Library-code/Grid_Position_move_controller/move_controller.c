#include "move_controller.h"
#include "motor.h"
#include "motor_controller.h"
#include "amt1450_uart.h"
#include "function.h"
#include "delay.h"
#include "config.h"
#include "task.h"
#include "mpu6050dmp.h"
#include "math.h"


MoveController_t moveController;

uint8_t carPosDif = 0;

/* 小车初始世界角度 */
static float carStartAngle;

/**
 * @brief:  移动控制器初始化
 * @retval: None
 */
void MoveControllerInit(void){

    /* -------- 设定初始坐标 -------- */
    /* 位置 */
    moveController.carPos = CAR_PLACE_POSVECTOR;
    moveController.carDivMoveController.curMicroPosition = moveController.carPos.position.y * 1000;
    moveController.carDivMoveController.curPosition = moveController.carPos.position.y;
    moveController.carDivMoveController.setPosition = moveController.carPos.position.y;

    /* 方向 */
    carStartAngle = atan2f(moveController.carPos.direction.y,moveController.carPos.direction.x);

    /* -------- 位置环PID参数设定 -------- */
    moveController.carDivMoveController.positionLoopPID.output = 0;
    moveController.carDivMoveController.positionLoopPID.pidParam.kp = CAR_POSITION_LOOP_CONTROLLER_KP;
    moveController.carDivMoveController.positionLoopPID.pidParam.ki = CAR_POSITION_LOOP_CONTROLLER_KI;
    moveController.carDivMoveController.positionLoopPID.pidParam.kd = CAR_POSITION_LOOP_CONTROLLER_KD;
    
    moveController.carDivMoveController.positionLoopPID.FilterPercent = 1.0f; /* 不滤波 */
    moveController.carDivMoveController.positionLoopPID.integrationLimit = CAR_MAX_LINEAR_SPEED;

    moveController.carDivMoveController.positionLoopPID.outMINLimit = 0 - Abs(CAR_DEFAULT_BACK_LINEAR_SPEED);
    moveController.carDivMoveController.positionLoopPID.outMAXLimit =     CAR_MAX_LINEAR_SPEED;
    
    /* -------- 角度环PID参数设定-------- */
    moveController.carDivMoveController.angleLoopPID.output = 0;
    moveController.carDivMoveController.angleLoopPID.pidParam.kp = CAR_ANGLE_LOOP_CONTROLLER_KP;
    moveController.carDivMoveController.angleLoopPID.pidParam.ki = CAR_ANGLE_LOOP_CONTROLLER_KI;
    moveController.carDivMoveController.angleLoopPID.pidParam.kd = CAR_ANGLE_LOOP_CONTROLLER_KD;
    
    moveController.carDivMoveController.angleLoopPID.FilterPercent = 1.0f; /* 不滤波 */
    moveController.carDivMoveController.angleLoopPID.integrationLimit = CAR_MAX_ANGLE_SPEED;

    moveController.carDivMoveController.angleLoopPID.outMINLimit = 0 - CAR_MAX_ANGLE_SPEED;
    moveController.carDivMoveController.angleLoopPID.outMAXLimit =     CAR_MAX_ANGLE_SPEED;

    /* 小车初始状态设置 */
    moveController.carMoveState = CAR_STOP;
    moveController.carTraceState = CAR_CENTER_TRACE;
    moveController.carPosState = CAR_NORMAL_POS;
}



/**
 * @brief:  购物车移动闭环控制任务
 * @retval: None
 */
uint8_t CarMoveLoopControlTask(uint8_t info){

    static uint8_t lastBegin;
    static int32_t lastPostion;
    
    if(info == TASK_CALL_INIT){
        lastPostion = moveController.carDivMoveController.curPosition;
        return 0;
    }

    /* -------- 数据更新 -------- */

    /* 更新相对位置数据 */
    moveController.carDivMoveController.curMicroPosition += (GetMotorPosition(0) + GetMotorPosition(1)) / 2;
    moveController.carDivMoveController.curPosition = moveController.carDivMoveController.curMicroPosition / 1000;

    /* 更新角度数据 */
    moveController.carDivMoveController.curAngel += 1.0f * (GetMotorPosition(1) - GetMotorPosition(0)) / CAR_EQUAL_DIAMETER / 1000 ;

    SetMotorPosition(0, 0);
    SetMotorPosition(1, 0);

    /* 更新循迹数据 */
    for (uint8_t i = 0; i < 2; i++ ){
        get_AMT1450Data_UART(&(amt1450_Data[i].begin), &(amt1450_Data[i].jump),
                              (amt1450_Data[i].count), i);
        if(amt1450_Data[i].jump==2){
            amt1450_Data[i].position = 0.5f * (amt1450_Data[i].count[0] + amt1450_Data[i].count[1]);
        }
    }

    

    if(moveController.carTraceState == CAR_NONE_TRACE)
        goto _Calibrate_Skip;
        
    /* 角度校准 */
    /* 校准条件： */
    /* 1、小车为前进状态。受限于偏心安装的循迹传感器，后退时小车偏转量被退行量补偿使得矫正量出错。不建议后退 */
    /* 2、当角速度小于50mm/s 且 角度在(-0.3 , 0.3 rad/s)内，约正负20度 */
    if( GetCarSpeed() >= 80 && GetCarRotationSpeed() < 80 &&  fabsf((moveController.carDivMoveController.curAngel/PI*2) - roundf((moveController.carDivMoveController.curAngel/PI*2))) < 0.3f ){    
        if( amt1450_Data[moveController.carTraceState].jump == 2){
            moveController.carDivMoveController.curAngel =
                (((float)amt1450_Data[moveController.carTraceState].position - 72) / 145 * SCANNER_AREA_WIDTH / SCANNER_REL_POS) +
                roundf(moveController.carDivMoveController.curAngel / PI * 2) * PI / 2;
        }
    }
    
    /* 计算位置预估偏差 */
//    carPosDif = (moveController.carDivMoveController.curPosition/BLOCK_DISTANCE+1)*BLOCK_DISTANCE - moveController.carDivMoveController.curPosition - SCANNER_REL_POS - WHITE_LINE_WIDTH/2;

    /* 位置校准 */
    if(GetCarRotationSpeed() < 80 && amt1450_Data[moveController.carTraceState].jump == 0 && lastBegin == 1 && amt1450_Data[moveController.carTraceState].begin == 0 ){
         
        lastBegin = 0;

            /* 如果小车前进，且距离目标校准点位置 BLOCK_DISTANCE - ( SCANNER_REL_POS + WHITE_LINE_WIDTH/2 ) 误差在 POS_UPDATE_TOL 以内 则校准位置 */
            if( GetCarSpeed()>=0){

                int32_t dif = 0;
                /* 如果当前方向为x轴 */
                if( Abs(Abs(roundf(moveController.carPos.direction.x)) - 1) < 0.2f ){
                    dif = fabsf(((int32_t)(moveController.carPos.position.x)/GRID_DISTANCE+roundf(moveController.carPos.direction.x/2+0.5f))*GRID_DISTANCE - (int32_t)(moveController.carPos.position.x)) - SCANNER_REL_POS - WHITE_LINE_WIDTH/2;
                    if( Abs(dif)<POS_UPDATE_TOL ){
                        moveController.carDivMoveController.curPosition += dif;
                        moveController.carDivMoveController.curMicroPosition += dif*1000;
                    }
                }else{
                    dif = fabsf(((int32_t)(moveController.carPos.position.y)/GRID_DISTANCE+roundf(moveController.carPos.direction.y/2+0.5f))*GRID_DISTANCE - (int32_t)(moveController.carPos.position.y)) - SCANNER_REL_POS - WHITE_LINE_WIDTH/2;
                    if( Abs(dif)<POS_UPDATE_TOL ){
                        moveController.carDivMoveController.curPosition += dif;
                        moveController.carDivMoveController.curMicroPosition += dif*1000;
                    }
                }     
                
                
            /* 如果小车后退，且距离目标校准点位置 BLOCK_DISTANCE - ( SCANNER_REL_POS - WHITE_LINE_WIDTH/2 ) 误差在 POS_UPDATE_TOL 以内 则校准位置 */
            }else if( GetCarSpeed()<0 ){

                int32_t dif = 0;
                /* 如果当前方向为x轴 */
                if( Abs(Abs(roundf(moveController.carPos.direction.x)) - 1) < 0.2f ){
                    dif = fabsf(((int32_t)(moveController.carPos.position.x)/GRID_DISTANCE+roundf(moveController.carPos.direction.x/2+0.5f))*GRID_DISTANCE - (int32_t)(moveController.carPos.position.x)) - SCANNER_REL_POS + WHITE_LINE_WIDTH/2;
                    if( Abs(dif)<POS_UPDATE_TOL ){
                        moveController.carDivMoveController.curPosition += dif;
                        moveController.carDivMoveController.curMicroPosition += dif*1000;
                    }
                }else{
                    dif = fabsf(((int32_t)(moveController.carPos.position.y)/GRID_DISTANCE+roundf(moveController.carPos.direction.y/2+0.5f))*GRID_DISTANCE - (int32_t)(moveController.carPos.position.y)) - SCANNER_REL_POS + WHITE_LINE_WIDTH/2;
                    if( Abs(dif)<POS_UPDATE_TOL ){
                        moveController.carDivMoveController.curPosition += dif;
                        moveController.carDivMoveController.curMicroPosition += dif*1000;
                    }
                }     
            }
        

            // /* 如果小车前进，且距离目标校准点位置 BLOCK_DISTANCE - ( SCANNER_REL_POS + WHITE_LINE_WIDTH/2 ) 误差在 POS_UPDATE_TOL 以内 则校准位置 */
            // if( GetCarSpeed()>=0){
            //     int32_t dif = (moveController.carDivMoveController.curPosition/GRID_DISTANCE+1)*GRID_DISTANCE - moveController.carDivMoveController.curPosition - SCANNER_REL_POS - WHITE_LINE_WIDTH/2;
            //     if( Abs(dif)<POS_UPDATE_TOL ){
            //         moveController.carDivMoveController.curPosition += dif;
            //         moveController.carDivMoveController.curMicroPosition += dif*1000;
            //     }
                
            // /* 如果小车后退，且距离目标校准点位置 BLOCK_DISTANCE - ( SCANNER_REL_POS - WHITE_LINE_WIDTH/2 ) 误差在 POS_UPDATE_TOL 以内 则校准位置 */
            // }else if( GetCarSpeed()<0 ){
            //     int32_t dif = (moveController.carDivMoveController.curPosition/GRID_DISTANCE+1)*GRID_DISTANCE - moveController.carDivMoveController.curPosition - SCANNER_REL_POS + WHITE_LINE_WIDTH/2;
            //     if( Abs(dif)<POS_UPDATE_TOL ){
            //         moveController.carDivMoveController.curPosition += dif;
            //         moveController.carDivMoveController.curMicroPosition += dif*1000;
            //     }
            // }
        
        
    }
    
    if(amt1450_Data[moveController.carTraceState].begin==1){
        lastBegin = 1;
    }
   
    
_Calibrate_Skip:
   
    /* 转换至世界坐标系 */
    {
        int32_t posDiv = (moveController.carDivMoveController.curPosition - lastPostion);
#if USE_GRID_POS_SYSTEM == 1
        moveController.carPos.position.x += posDiv * roundf(moveController.carPos.direction.x);
        moveController.carPos.position.y += posDiv * roundf(moveController.carPos.direction.y);
#else
        moveController.carPos.position.x += posDiv * (moveController.carPos.direction.x);
        moveController.carPos.position.y += posDiv * (moveController.carPos.direction.y);
#endif

#if USE_GRID_DIR_SYSTEM == 1
        moveController.carPos.direction.x = (moveController.carDivMoveController.curAngel) / (PI/2)  ; /* 废弃 */ 
#else
        moveController.carPos.direction.x = cosf(moveController.carDivMoveController.curAngel + carStartAngle);
        moveController.carPos.direction.y = sinf(moveController.carDivMoveController.curAngel + carStartAngle);
#endif

        lastPostion = moveController.carDivMoveController.curPosition;
    }

    /* --------- 位置环 --------- */

    /* 误差死区 */
    if( Abs(moveController.carDivMoveController.setPosition - moveController.carDivMoveController.curPosition) < CAR_POSITION_ERROR_THRESHOLD ){
        moveController.carDivMoveController.expPosition = moveController.carDivMoveController.curPosition;
    }else{
        moveController.carDivMoveController.expPosition = moveController.carDivMoveController.setPosition;
    }
    
    /* 调用PID控制器 */
    POSPID_Update(&moveController.carDivMoveController.positionLoopPID, moveController.carDivMoveController.expPosition, moveController.carDivMoveController.curPosition, 1.0f * CAR_MOVE_LOOP_CONTROLLER_PERIOD / 1000);
    
    /* 速度死区 */
//    if( Abs(moveController.carDivMoveController.positionLoopPID.output) < MOTORCONTROLLER_MIN_SEPPED_DEFAULT ){
//        moveController.carDivMoveController.positionLoopPID.output = 0;
//    } 
    
    /* 位置环传递给速度环的给定速度 */
    SetCarSpeed(moveController.carDivMoveController.positionLoopPID.output);


    /* --------- 角度环 --------- */

    /* 误差死区 */
    if( Abs(moveController.carDivMoveController.setAngel - moveController.carDivMoveController.curAngel) < CAR_ANGLE_ERROR_THRESHOLD ){
        moveController.carDivMoveController.expAngel = moveController.carDivMoveController.curAngel;
    }else{
        moveController.carDivMoveController.expAngel = moveController.carDivMoveController.setAngel;
    }
    
    /* 调用PID控制器 */
    POSPID_Update(&moveController.carDivMoveController.angleLoopPID, moveController.carDivMoveController.expAngel, moveController.carDivMoveController.curAngel, 1.0f * CAR_MOVE_LOOP_CONTROLLER_PERIOD / 1000);
    
    /* 速度死区 */
//     if( Abs(moveController.carDivMoveController.angleLoopPID.output) < MOTORCONTROLLER_MIN_SEPPED_DEFAULT ){
//         moveController.carDivMoveController.angleLoopPID.output = 0;
//     } 

    /* 角度环传递给速度环的给定速度 */
    SetCarRotationSpeed(moveController.carDivMoveController.angleLoopPID.output);

    
    
    /* --------- 更新小车状态 --------- */
    if( Abs(moveController.carDivMoveController.curAngel - moveController.carDivMoveController.expAngel) < 1e-4f &&
        moveController.carDivMoveController.curPosition == moveController.carDivMoveController.expPosition &&
        GetCarSpeed() == 0 && GetCarRotationSpeed() == 0 &&
        moveController.carDivMoveController.positionLoopPID.output == 0 &&
        moveController.carDivMoveController.positionLoopPID.output == 0 )
    {
        moveController.carMoveState = CAR_STOP;
    }else{
        moveController.carMoveState = CAR_MOVE;
    }
       
    
    return 0;
}


/**
 * @brief  设置小车循迹状态
 * @param  traceState 目标循迹状态 
 *                    可选 CAR_CENTER_TRACE 中央循迹方式,
                           CAR_LEFT_TRACE   偏左循迹方式
 * @retval 
 */
void CarSetTraceState(CarTraceState_e traceState){
    moveController.carTraceState = traceState;
    return;
}


/**
 * @brief  基于距离的小车移动命令函数
 * @param  distance 距离，小车移动的距离
 * @param  angle    角度，小车转弯的角度
 * @retval none
 */
void CarMove(int32_t distance, float angle){

    moveController.carMoveState = CAR_MOVE;

    CarSetSpeedLimit(CAR_MAX_LINEAR_SPEED, CAR_DEFAULT_BACK_LINEAR_SPEED, CAR_MAX_ANGLE_SPEED);

    moveController.carDivMoveController.setPosition += distance;
    moveController.carDivMoveController.setAngel += angle;
    return;
}



/**
 * @brief  基于网格的小车移动命令函数，该函数会使得小车在直行方向对齐至网格
 * @param  count 网格刻度，小车对齐的网格刻度值
 * @param  angle 角度，小车转弯的角度
 * @retval none
 */
void CarMoveWithGrid(int16_t count, float angle){

    moveController.carMoveState = CAR_MOVE;
    
    CarSetSpeedLimit(CAR_MAX_LINEAR_SPEED, CAR_DEFAULT_BACK_LINEAR_SPEED, CAR_MAX_ANGLE_SPEED);

    /* 如果当前方向为x轴 */
    if( Abs(Abs(roundf(moveController.carPos.direction.x)) - 1) < 0.2f ){
        moveController.carDivMoveController.setPosition += count * GRID_DISTANCE - fmodf((moveController.carPos.position.x), GRID_DISTANCE);
    }else{
        moveController.carDivMoveController.setPosition += count * GRID_DISTANCE - fmodf((moveController.carPos.position.y), GRID_DISTANCE);
    }

    //moveController.carPosState = CAR_FIXING_POS;

    moveController.carDivMoveController.setAngel += angle;
    return;
}


/**
 * @brief  基于曲线的小车移动命令函数，该函数会使得小车沿圆弧曲线行驶。
 * @param  radius 转弯半径，亦即圆弧半径。该命令基于极坐标，因此取正时为左转，取负时为右转。 单位：mm
 * @param  angle  转弯角度，取正时为前进，取负时为倒车。 单位：rad
 * @retval 
 */
void CarMoveWithCurve(int16_t radius, float angle){

    moveController.carMoveState = CAR_MOVE;

    float targetAngleSpeed = 1.0f * CAR_MAX_LINEAR_SPEED / radius * ((float)CAR_EQUAL_DIAMETER / 2);

    CarSetSpeedLimit(CAR_MAX_LINEAR_SPEED, CAR_MAX_LINEAR_SPEED, targetAngleSpeed);

    moveController.carDivMoveController.setPosition += radius * angle;
    moveController.carDivMoveController.setAngel += angle;

}

/**
 * @brief:  小车停止命令函数
 * @param:  
 * @retval: 
 */
void CarStop(){

}


/**
 * @brief  设置小车速度限制
 * @param  linearSpeed 线速度 单位mm/s
 * @param  angleSpeed  角速度 单位mm/s
 * @retval 设置状态 1 设置成功
 *                  0 速度值非法，超过移动控制器限制（多为电机限制值）
 */
uint8_t CarSetSpeedLimit(int16_t linearSpeed, int16_t invLinearSpeed, int16_t angleSpeed){

    if( Abs(linearSpeed)>CAR_MAX_LINEAR_SPEED || Abs(invLinearSpeed)>CAR_MAX_LINEAR_SPEED || Abs(angleSpeed)>CAR_MAX_ANGLE_SPEED )
        return 0;

    moveController.carDivMoveController.positionLoopPID.integrationLimit = Abs(linearSpeed);
    moveController.carDivMoveController.positionLoopPID.outMINLimit = 0 - Abs(invLinearSpeed);
    moveController.carDivMoveController.positionLoopPID.outMAXLimit =     Abs(linearSpeed);

    moveController.carDivMoveController.angleLoopPID.integrationLimit = Abs(angleSpeed);
    moveController.carDivMoveController.angleLoopPID.outMINLimit = 0 - Abs(angleSpeed);
    moveController.carDivMoveController.angleLoopPID.outMAXLimit =     Abs(angleSpeed);

    return 1;
}

/**
 * @brief  获取小车坐标
 * @retval 小车坐标指针
 */
const CarPosVector_t* GetCarPosVector(void){
    return &moveController.carPos;
}


/**
 * @brief  获取小车运动状态
 * @retval 运动状态
 */
CarMoveState_e GetCarMoveState(void){
    return moveController.carMoveState;
}

//PosVector_t carPosVector={{0,0},{0,0}}; 

uint8_t isPoint = 0;
uint8_t isPointBuff = 0;
uint8_t isPointSendBuff = 0;
float moveXdistance = 0;
float moveYdistance = 0;
float movedirection = 0;


    
uint8_t isCalibrated = 0;   //是否已经被零点校准

uint8_t map[MAP_SIZE][MAP_SIZE] = MAP;

//PosVector_t carPathVector[PATH_MAX_LENGTH]={{{0,0},{0,0}}};
uint8_t pathIndex;
uint8_t pathLength;

//传感器AMT1450返回值

extern uint8_t isShopping;

//POSPIDController TrackPIDController;





/**
 * @brief:  位置向量相等判断
 * @param:  none
 * @retval: 
 */                         
// uint8_t IsCarPosEqual(PosVector_t pos1 , PosVector_t pos2){
//     return ( (pos1.position[0]==pos2.position[0]) && (pos1.position[1]==pos2.position[1]) && (pos1.direction[0]==pos2.direction[0]) && (pos1.direction[1]==pos2.direction[1]) );
// }



// /**
//  * @brief:  坐标点检测（绝对位置）
//  * @retval: None
//  */
// uint8_t	PointScan(void)
// {
//     static uint8_t scanState = 0;
//     //static uint8_t preScan[POINT_SCAN_FILTER*2]={0};
    
//     //get_AMT1450Data_UART(&begin, &jump, count);
    
//     //for(uint8_t i=0;i<POINT_SCAN_FILTER*2-1;i++){
//     //    preScan[i]=preScan[i+1];
//     //}
//     //preScan[POINT_SCAN_FILTER*2-1]=begin;
    
// //     if(scanState==0 && begin==0 && jump==0){
// // //        uint8_t i=0;
// // //        for(i=0;i<POINT_SCAN_FILTER;i++){
// // //            if(preScan[i]!=0)
// // //                return(scanState);
// // //        }
// // //        for(;i<POINT_SCAN_FILTER*2;i++){
// // //            if(preScan[i]!=1)
// // //                return(scanState);
// // //        }
// //         scanState=1;
// //         isPoint=1;
        
// //         isPointBuff=1;
// //         isPointSendBuff=1;
// //     }
    
//     //if(scanState==1 && begin==1){
// //        uint8_t i=0;
// //        for(i=0;i<POINT_SCAN_FILTER;i++){
// //            if(preScan[i]!=1)
// //                return(scanState);
// //        }
// //        for(;i<POINT_SCAN_FILTER*2;i++){
// //            if(preScan[i]!=0)
// //                return(scanState);
// //        }
//     //     scanState=0;
//     //     isPoint=0;
//     // }
    
//     return(scanState);
// }

// /**
//  * @brief:  机器人坐标更新任务
//  * @retval: None
//  */
// uint8_t UpdateCarCurPosTask(uint8_t info){
    
//     if(info==0)
//         return 0;
    
//     //空间坐标更新
//     int16_t carSpeed = GetCarSpeed(); 
//     moveXdistance +=  1.0f * carSpeed * carPosVector.direction[0] * MOVE_CONTROLLER_PERIOD / 1000 ;
//     moveYdistance +=  1.0f * carSpeed * carPosVector.direction[1] * MOVE_CONTROLLER_PERIOD / 1000 ;
    
//     PointScan();
    
//     if(moveXdistance>=BLOCK_DISTANCE-POS_UPDATE_TOL){
//         carPosVector.position[0]+=1;
//         moveXdistance=-(POS_UPDATE_TOL);
//     }
//     else if(moveXdistance<=0-(BLOCK_DISTANCE-POS_UPDATE_TOL)){
//         carPosVector.position[0]-=1;
//         moveXdistance=(POS_UPDATE_TOL);
//     }
    
//     if(moveYdistance>=(BLOCK_DISTANCE-POS_UPDATE_TOL)){
//         carPosVector.position[1]+=1;
//         moveYdistance=-(POS_UPDATE_TOL);
//     }
//     else if(moveYdistance<=0-(BLOCK_DISTANCE-POS_UPDATE_TOL)){
//         carPosVector.position[1]-=1;
//         moveYdistance=(POS_UPDATE_TOL);
//     }
    
//     if(isPointBuff){
//         float correctingDiv;
        
//         if(isPoint){
//             correctingDiv = 0;
//         }else{
//             correctingDiv = WHITE_LINE_WIDTH / 2 ;
//         }
        
//         if(isCalibrated==0){
            
//             moveXdistance = 0;
//             moveYdistance = BLOCK_DISTANCE-(SCANNER_REL_POS)+correctingDiv;
//             carPosVector = (PosVector_t)CAR_PLACE_POSVECTOR;
//             isCalibrated=1;
            
//         }else{
            
//             if(Abs(moveXdistance+(SCANNER_REL_POS-WHITE_LINE_WIDTH)-BLOCK_DISTANCE)<=POS_UPDATE_TOL * 2){
//                 moveXdistance = BLOCK_DISTANCE-(SCANNER_REL_POS-WHITE_LINE_WIDTH)+correctingDiv;
//                 //isAdjustBuff=1;
//             }else if(Abs(-moveXdistance+(SCANNER_REL_POS-WHITE_LINE_WIDTH)-BLOCK_DISTANCE)<=POS_UPDATE_TOL * 2){
//                 moveXdistance = -(BLOCK_DISTANCE-(SCANNER_REL_POS-WHITE_LINE_WIDTH)+correctingDiv);
//                 //isAdjustBuff=1;
//             }
// //            else if(Abs(moveXdistance-SCANNER_REL_POS-BLOCK_DISTANCE)<POS_UPDATE_TOL){
// //                moveXdistance = correctingDiv;
// //                carPosVector[0][0]+=1;
// //            }else if(Abs(moveXdistance+BLOCK_DISTANCE)<POS_UPDATE_TOL){
// //                moveXdistance = correctingDiv;
// //                carPosVector[0][0]-=1;
// //            }
            
//             if(Abs(moveYdistance+(SCANNER_REL_POS-WHITE_LINE_WIDTH)-BLOCK_DISTANCE)<=POS_UPDATE_TOL * 2){
//                 moveYdistance = BLOCK_DISTANCE-(SCANNER_REL_POS-WHITE_LINE_WIDTH)+correctingDiv;
//                 //isAdjustBuff=1;
//             }else if(Abs(-moveYdistance+(SCANNER_REL_POS-WHITE_LINE_WIDTH)-BLOCK_DISTANCE)<=POS_UPDATE_TOL * 2){
//                 moveYdistance = -(BLOCK_DISTANCE-(SCANNER_REL_POS-WHITE_LINE_WIDTH)+correctingDiv);
//                 //isAdjustBuff=1;
//             }
// //            else if(Abs(moveYdistance-BLOCK_DISTANCE)<POS_UPDATE_TOL){
// //                moveYdistance = correctingDiv;
// //                carPosVector[0][1]+=1;
// //            }else if(Abs(moveYdistance+BLOCK_DISTANCE)<POS_UPDATE_TOL){
// //                moveYdistance = correctingDiv;
// //                carPosVector[0][1]-=1;
// //            }
        
//             isPointBuff=0;
//         }   
//     }
    
//     /* 方向矢量更新 */
//     int16_t carRotationSpeed = GetCarRotationSpeed();
//     movedirection +=  ( 1.0f * carRotationSpeed * MOVE_CONTROLLER_PERIOD / 1000 / ( CAR_EQUAL_DIAMETER * PI ) * 360 );
    
//     if(movedirection>=90-ROATION_UPDATE_TOL){
//         int8_t x=carPosVector.direction[0];
//         int8_t y=carPosVector.direction[1];
        
//         carPosVector.direction[0]=0-y;
//         carPosVector.direction[1]=x;
        
//         movedirection=-(ROATION_UPDATE_TOL-ROATION_ADJUST_TOL);
//     }
//     else if(movedirection<=-(90-ROATION_UPDATE_TOL)){
//         int8_t x=carPosVector.direction[0];
//         int8_t y=carPosVector.direction[1];
        
//         carPosVector.direction[0]=y;
//         carPosVector.direction[1]=0-x;
        
//         movedirection=(ROATION_UPDATE_TOL-ROATION_ADJUST_TOL);
//     }
    
// //    get_AMT1450Data_UART(&begin, &jump, count);
// //	if(jump == 2){
// //        position = 0.5f * (count[0] + count[1]);
// //    
// //        if( (Abs(movedirection) <= ROATION_UPDATE_TOL) && (Abs(position-TRACK_SENSOR_MID) <= ROATION_ADJUST_TOL) ){
// //            movedirection=0;
// //            isAdjustBuff=1;
// //        }
// //    }
    
//     return 0;
// }

// /**
//  * @brief:  机器人坐标校准
//  * @retval: None
//  */
// void PosVectorCalibrate(void){
//     uint16_t calibratedTimeout=0;
//     SetCarSpeed(CAR_MOVE_SPEED_LOW);
//     while(1){
//         Delay_ms(1);
//         calibratedTimeout++;
//         if(isCalibrated){
//             if(IsCarPosEqual(GetCarPos(),(PosVector_t)CAR_START_POSVECTOR)){
//                 break;
//             }
//         }
//         if(calibratedTimeout>=3000){
//             PosCalibratedError();
//         }
//     }
//     SetCarSpeed(0);
//     SetCarRotationSpeed(0);
//     //carState=carStop;
// }


// /**
//  * @brief:  获取机器人状态
//  * @retval: None
//  */
// //CarState_e GetCarState(void){
// //    return carState;
// //}


// /**
//  * @brief:  获取机器人坐标
//  * @retval: None
//  */
// PosVector_t GetCarPos(void){
//     return carPosVector;
// }

// /**
//  * @brief:  设置机器人坐标
//  * @retval: None
//  */
// void SetCarPos(PosVector_t pos){
//     carPosVector=pos;
// }

// /**
//  * @brief:  A*搜索算法
//  * @retval: None
//  */
// int8_t AStarSearchDir[4][2]= {{0,1},{1,0},{0,-1},{-1,0}};
    
// uint8_t AStarSearch(uint8_t (*valueMap)[MAP_SIZE], PosVector_t curPos){
//     uint8_t res[4] = { 255,255,255,255 };
//     uint8_t curValue = valueMap[curPos.position[0]][curPos.position[1]];

//     for (uint8_t i = 0; i < 4; i++) {
        
//         int8_t x = curPos.position[0] + AStarSearchDir[i][0];
//         int8_t y = curPos.position[1] + AStarSearchDir[i][1];
        
//         if((x<0||x>=MAP_SIZE)||(y<0||y>=MAP_SIZE))continue;
        
//         if (map[x][y] == 0 || map[x][y] == 5) {
//             if (valueMap[x][y] > curValue + 1) {
//                 valueMap[x][y] = curValue + 1;
//                 if (map[x][y] == 5)
//                     return (curValue + 1);
//                 res[i] = AStarSearch(valueMap, (PosVector_t){ x, y, 0, 0 } );
//             }
//         }
//     }
    
//     uint8_t min = 255;
//     for (uint8_t i = 0; i < 4; i++){
//         if (res[i] < min){
//             min = res[i];
//         }        
//     }
//     if (min != 255){
//         return min;
//     }
        
//     return 255;
// }


// /**
//  * @brief:  机器人路径规划
//  * @retval: None
//  */
// uint8_t PathPlanning(PosVector_t targetPos){
    
//     //if(carState==carMove)return 255;
//     if(Abs(targetPos.direction[0]+targetPos.direction[1])!=1)return 255;
    
//     PosVector_t startPos = GetCarPos();  //获取机器人当前位置
    
//     ///PosVector_t pathVectorStack[PATH_MAX_LENGTH];  //路径堆栈
//     //uint8_t stackIndex = 0;
 
//     uint8_t pathNodeNum = 0;

//     //建立路径评估地图
//     uint8_t valueMap[MAP_SIZE][MAP_SIZE];
//     for(uint8_t i=0;i<MAP_SIZE;i++){
//         for(uint8_t j=0;j<MAP_SIZE;j++){
//             valueMap[i][j]=255;
//         }
//     }
//     valueMap[startPos.position[0]][startPos.position[1]]=0;
//         map[startPos.position[0]][startPos.position[1]] = 0;
//     map[targetPos.position[0]][targetPos.position[1]] = 5;
    
//     pathNodeNum = AStarSearch(valueMap, startPos);
    
//     if(pathNodeNum==255)return 0;
    
//     pathIndex=0;
//     pathLength=0;
//     //建立机器人路径矢量
//     carPathVector[pathIndex]=targetPos;
//     for(uint8_t i=0;i<pathNodeNum;){
        
//         for (uint8_t j = 0; j < 4; j++) {
        
//             int8_t x = carPathVector[pathIndex].position[0] + AStarSearchDir[j][0];
//             int8_t y = carPathVector[pathIndex].position[1] + AStarSearchDir[j][1];
            
//             if((x<0||x>=MAP_SIZE)||(y<0||y>=MAP_SIZE))continue;
            
//             if(valueMap[x][y]==(pathNodeNum-i-1)){
//                 if( (carPathVector[pathIndex].direction[0]+AStarSearchDir[j][0]==0) && (carPathVector[pathIndex].direction[1]+AStarSearchDir[j][1]==0) ){
//                     pathIndex++;
//                     carPathVector[pathIndex]=carPathVector[pathIndex-1];
//                     carPathVector[pathIndex].position[0]=x;
//                     carPathVector[pathIndex].position[1]=y;
//                     i++;
//                     break;
//                 }else{
//                     pathIndex++;
//                     carPathVector[pathIndex]=carPathVector[pathIndex-1];
//                     carPathVector[pathIndex].direction[0]=0 - AStarSearchDir[j][0];
//                     carPathVector[pathIndex].direction[1]=0 - AStarSearchDir[j][1];
//                     break;
//                 }
//             }
//         }
//     }
//     carPathVector[++pathIndex]=startPos;
//     pathLength=pathIndex+1;
    
//     moveXdistance = 0;
//     moveYdistance = 0;
//     movedirection = 0;
    
//     CreateTask(&PathFollowingTask,1,MOVE_CONTROLLER_PERIOD);
//     //carState=carMove;
    
//     return pathIndex;
// }

// /**
//  * @brief:  机器人路径导航任务
//  * @retval: None
//  */
// uint8_t PathFollowingTask(uint8_t info){
    
//     if(info==0){
//         moveXdistance = 0;
//         moveYdistance = 0;
//         movedirection = 0;
//         return 0;
//     }
    
//     if(pathLength==0)
//         return 0;
    
//     PosVector_t CarCurPos=GetCarPos();
//     if(IsCarPosEqual(CarCurPos,carPathVector[pathIndex])){
        
//         pathIndex--;
//         pathLength--;
        
//         if(pathLength==0){
//             SetCarSpeed(0);
//             SetCarRotationSpeed(0);
//             if(GetCarSpeed()!=0 || GetCarRotationSpeed()!=0){
//                 pathIndex++;
//                 pathLength++;
//                 return 0;
//             }
//             //carState=carStop;
//             moveXdistance = 0;
//             moveYdistance = 0;
//             movedirection = 0;
//             return 3;
//         }
        
//         int8_t posXdif = carPathVector[pathIndex].position[0]-CarCurPos.position[0];
//         int8_t posYdif = carPathVector[pathIndex].position[1]-CarCurPos.position[1];
       
//         int8_t dir =  CarCurPos.direction[0] * carPathVector[pathIndex].direction[1] - CarCurPos.direction[1] * carPathVector[pathIndex].direction[0];
        
//         if(posXdif!=0 || posYdif!=0 ){
//             SetCarRotationSpeed(0);
//             if(GetCarRotationSpeed()!=0){
//                 pathIndex++;
//                 pathLength++;
//                 return 0;
//             }
//             SetCarSpeed(CAR_MOVE_SPEED);
//         }
        
//         if(dir==1){
//             SetCarSpeed(0);
//             if(GetCarSpeed()!=0){
//                 pathIndex++;
//                 pathLength++;
//                 return 0;
//             }
//             //SetCarSpeed(ROATION_DEC_VALUE);
//             SetCarRotationSpeed(CAR_MOVE_SPEED);
//         }else if(dir==-1){
//             SetCarSpeed(0);
//             if(GetCarSpeed()!=0){
//                 pathIndex++;
//                 pathLength++;
//                 return 0;
//             }
//             //SetCarSpeed(ROATION_DEC_VALUE);
//             SetCarRotationSpeed(0-CAR_MOVE_SPEED);
//         }else{
//             if( (CarCurPos.direction[0]!=carPathVector[pathIndex].direction[0]) ||  (CarCurPos.direction[1]!=carPathVector[pathIndex].direction[1]) ){
//                 SetCarSpeed(0);
//                 if(GetCarSpeed()!=0){
//                     pathIndex++;
//                     pathLength++;
//                     return 0;
//                 }
//                 //SetCarSpeed(ROATION_DEC_VALUE);
//                 SetCarRotationSpeed(0-CAR_MOVE_SPEED);
//             }else{
//                 SetCarRotationSpeed(0);
// //                if(GetCarRotationSpeed()!=0){
// //                    pathIndex++;
// //                    pathLength++;
// //                    return 0;
// //                }
//             }
//         }   
//     }
    
//     return 0;
// }


// uint8_t CarGoCertainDistance(int16_t distance){
    
//     //if(GetCarState()==carMove)return 0;
    
//     moveXdistance=0;
//     moveYdistance=0;
    
//     if(distance>0)
//         SetCarSpeed(CAR_MOVE_SPEED_LOW);
//     else
//         SetCarSpeed(0-CAR_MOVE_SPEED_LOW);
    
//     Delay_ms(Abs(distance)*1000/CAR_MOVE_SPEED_LOW);
//     SetCarSpeed(0);
//     while(GetCarSpeed()!=0);
    
//     return 0;
// }

// uint8_t CarGoCertainAngle(int16_t angle){

//     //if(GetCarState()==carMove)return 0;
    
//     movedirection=0;
    
//     if(angle>0)
//         SetCarRotationSpeed(CAR_MOVE_SPEED_LOW);
//     else
//         SetCarRotationSpeed(0-CAR_MOVE_SPEED_LOW);
    
//     Delay_ms(Abs(angle));
//     SetCarRotationSpeed(0);
//     while(GetCarRotationSpeed()!=0);
    
//     return 0;
// }



