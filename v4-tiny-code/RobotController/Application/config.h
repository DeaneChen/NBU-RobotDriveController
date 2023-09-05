/**
 * @file  config.h
 * @brief 全局配置文件
 * @note  该文件整合了机器人控制器的大部分基本控制参数，从而便于系统的调整
 *        默认情况下，该文件已被 main.h 包含 
 *        采用 $ 标记的为不了解的情况下不推荐修改的参数
 */
#ifndef __CONFIG_H
#define __CONFIG_H


/* ------------------------------------------------- 用户自定义部分 ----- */


/* ------------------------------------------------- 扩展控制部分 ------- */



/* ------------------------------------------------- 基本控制部分 ------- */

/* 低电量报警电压 单位mV */
#define LOW_VBAT_ALARM_THRESHOLD  (11100)

/* $ 后台循环程序的调用时间周期，需根据中断周期设定 单位ms */
#define BACKEND_LOOP_CYCLE_TIME   (20)

/* ------------ 电机闭环控制相关 -------------- */
/* 编码器等效分辨率，即轮子转一圈输出的脉冲个数，通常=编码器其分辨率x电机减速比 */
#define MOTOR_CONTROLLER_ENCODER_RESOLUTION (15000)
/* 电机外接车轮的直径 单位mm 可为float类型数据 */
#define MOTOR_WHEEL_DIAMETER                (82.0f)
/* 电机最大加速度限制 单位mm/s^(-2) */
#define MOTOR_CONTROLLER_ACC_LIMIT          (800)
/* $ 电机闭环控制器控制周期，使用了定时器6，需根据定时器6中断周期设定 单位ms  */
#define MOTOR_CONTROLLER_PERIOD             (10) 
/* 机闭环控制器的PID参数初始值，运行中可通过调用函数改变 */
#define MOTOR_CONTROLLER_KP      (20)
#define MOTOR_CONTROLLER_KI      (4)
#define MOTOR_CONTROLLER_KD      (0.8f)

/* ------------------------------------------------- 核心驱动部分 ------- */

/* ADC 基准电压 单位mV */
#define ADC_REF_VOLTAGE (3300)

/* ------------ 电池检测相关 ------------------ */
/* $ 电池电压采样电阻比 (高侧分压电阻+低侧分压电阻)/低侧分压电阻 */
/* $ 警告：该处故意不用括号保护运算，不要自行添加，避免发生错误 */
#define BAT_VOLTAGE_R_SAMPLE_RATIO (680+100)/100
/* 启用电池采样的均值窗口滤波 1为启用 0为禁止 */
#define IS_ENABLE_VBAT_MEAN_WINDOW_FILTER (1)

/* ------------ 电机与编码器相关 -------------- */

/* 直流电机数量，最小为0，最大为4 */
#define MOTOR_COUNT    (4)
/* $ 编码器数量 */
#define ENCODER_COUNT  (MOTOR_COUNT)

/* $ 启用直流电机电流检测 */
#define IS_ENABLE_MOTOR_CURRENT_DETECTION  (1)
/* $ 直流电机电流全通道采样 */
/* $ 当启用该配置时，不论电机数量为几，均进行全通道（4个电机）采样，若不启用该配置，需要自行修改ADC的通道配置，避免报错 */
#define IS_ENABLE_MOTOR_CURRENT_FULL_DETECTION  (1)
/* $ 电流检测的采样电阻阻值，用于整形计算公式 单位：欧姆 */
#define MOTOR_CURRENT_DETECTION_R_SAMPLE     (1000)
/* $ 电流检测的采样电阻阻值的倒数，用于浮点计算公式 单位：欧姆^(-1) */
#define MOTOR_CURRENT_DETECTION_R_SAMPLE_REC (0.001f)

/* $ 直连电机驱动的PWM占空比最大值 0 ~ MOTOR_PWM_DUTY_LIMIT 对应 0~100% */
#define MOTOR_PWM_DUTY_LIMIT  (10000)
/* $ 编码器范围 */
#define MOTOR_TIM_ENCODER_ARR (60000)
/* $ 舵机控制的PWM占空比最大值 */
#define SERVO_PWM_DUTY_LIMIT  (10000)





#endif

