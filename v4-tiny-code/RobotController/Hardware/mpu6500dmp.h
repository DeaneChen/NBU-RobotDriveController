#ifndef __mpu6500dmp_H
#define __mpu6500dmp_H

#include "main.h"

//****************************************
// 定义MPU6500内部地址
//****************************************
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	MPU6500_CONFIG	0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)
#define	MPU6500_Addr	0x68	//IIC写入时的地址字节数据，+1为读取
#define MPU6500_I2C     hi2c1   //MPU6500 I2C端口


extern double pitch,roll,yaw;

int stm32_i2c_write(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data);
                     
int stm32_i2c_read(unsigned char slave_addr,
                   unsigned char reg_addr,
                   unsigned char length,
                   unsigned char *data);

int stm32_get_clock_ms(unsigned long* count);

//I2C通讯超时参数
#define MPU_I2C_FLAG_TIMEOUT  ((uint32_t)0x000A)
#define MPU_I2C_LONG_TIMEOUT  ((uint32_t)(10 * MPU_I2C_FLAG_TIMEOUT))

/**
 * @brief:MPU6500 DMP 初始化
 */         
uint16_t MPU6500_DMP_Init(void);

extern double mpu_pitch,mpu_roll,mpu_yaw;
/**
 * @brief 获取MPU6500数据，存储在全局变量 mpu_roll，mpu_pitch，mpu_yaw 中，
 * @note  该函数为阻塞查询函数，消耗的时间约为2ms，（在 STM32F407 168MHz 工作环境下 运行耗时约为2106us）
 *        但在最坏的情况下可能发生数据缓冲复位或I2C通讯超时（MPU异常）等问题。
 *        [Warning] MPU6500的传感器数据产生速率约为100Hz，倘若获取MPU6500的DMP数据速度慢于100Hz，长时间积累下会导致MPU6500内部的数据缓冲区溢出，
 *        从而触发MPU6500的数据缓冲区复位，这一过程耗时的测量值约为60ms，倘若为了避免这一过程，最好确保该函数的调用速度略微大于100Hz。   
 */
extern uint8_t Get_MPU6500_DMP_Data(void);
                   
#endif
