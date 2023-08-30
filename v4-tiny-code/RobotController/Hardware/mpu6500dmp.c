#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "mpu6500dmp.h"
#include "delay.h"
#include "i2c.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "led.h"

/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};
static struct hal_s hal = {0};

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};


/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

//static inline void msp430_reset(void)
//{
//    PMMCTL0 |= PMMSWPOR;
//}
/* stm32 复位函数 */
//static inline void stm32_reset(void){
//    __set_FAULTMASK(1);
//    NVIC_SystemReset();
//}

static inline void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x03 ) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        
        //accel_sens = 0;  //设置当前参考系的加速度为零
        
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    }
    
}

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
static void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}


/**
 *@brief:MPU6500 DMP 初始化
 */
uint16_t MPU6500_DMP_Init(void){
    
    int result;
    //unsigned char accel_fsr;
    //unsigned short gyro_rate, gyro_fsr;
    //unsigned long timestamp;
    struct int_param_s int_param;


    /* Set up gyro.
     * Every function preceded by mpu_ is a driver function and can be found
     * in inv_mpu.h.
     */
    result = mpu_init(&int_param);
    if (result){
        //stm32_reset();
        return 1;
    }

    /* If you're not using an MPU9150 AND you're not using DMP features, this
     * function will place all slaves on the primary bus.
     * mpu_set_bypass(1);
     */

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    if(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))return 2;
    /* Push both gyro and accel data into the FIFO. */
    if(mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))return 3;
    if(mpu_set_sample_rate(DEFAULT_MPU_HZ))return 4;
    /* Read back configuration in case it was set improperly. */
    //mpu_get_sample_rate(&gyro_rate);
    //mpu_get_gyro_fsr(&gyro_fsr);
    //mpu_get_accel_fsr(&accel_fsr);

    /* Initialize HAL state variables. */
    memset(&hal, 0, sizeof(hal));
    hal.sensors = ACCEL_ON | GYRO_ON;
    //hal.report = PRINT_QUAT;

    /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    if(dmp_load_motion_driver_firmware())return 5;
    if(dmp_set_orientation(
          inv_orientation_matrix_to_scalar(gyro_orientation)))return 6;

    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    if(dmp_enable_feature(hal.dmp_features))return 7;
    if(dmp_set_fifo_rate(DEFAULT_MPU_HZ))return 8;
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;
    
    return 0;
}


static unsigned long timestamp;
static short gyro[3], accel[3], sensors;
double mpu_pitch,mpu_roll,mpu_yaw;

/**
 * @brief: 获取MPU6500数据，存储在全局变量 mpu_roll，mpu_pitch，mpu_yaw 中
 */
uint8_t Get_MPU6500_DMP_Data(void){
    
    unsigned long sensor_timestamp;
 
    stm32_get_clock_ms(&timestamp);
    
    gyro_data_ready_cb();                   //数据采集结束标志位
    
    if (hal.new_gyro && hal.dmp_on) {
        
        unsigned char more;
        long quat[4];
        /* This function gets new data from the FIFO when the DMP is in
         * use. The FIFO can contain any combination of gyro, accel,
         * quaternion, and gesture data. The sensors parameter tells the
         * caller which data fields were actually populated with new data.
         * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
         * the FIFO isn't being filled with accel data.
         * The driver parses the gesture data to determine if a gesture
         * event has occurred; on an event, the application will be notified
         * via a callback (assuming that a callback function was properly
         * registered). The more parameter is non-zero if there are
         * leftover packets in the FIFO.
         */
        if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,
            &more))return 1;
        
        if (!more)
            hal.new_gyro = 0;
        
        if(sensors & INV_WXYZ_QUAT)
        {
            double q0=quat[0];
            double q1=quat[1];
            double q2=quat[2];
            double q3=quat[3];
            
            q0/=((unsigned long)(0x01)<<30);
            q1/=((unsigned long)(0x01)<<30);
            q2/=((unsigned long)(0x01)<<30);
            q3/=((unsigned long)(0x01)<<30);
                        
            //运算耗时: STM32F407VE 168MHz 296us
            mpu_roll  = atan2(2*q2*q3+2*q0*q1,(-2)*q1*q1-2*q2*q2+1)*57.3;
            mpu_pitch = asin ((-2)*q1*q3+2*q0*q2)*57.3;
            mpu_yaw   = atan2(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)*57.3;
            
            //printf("%.3lf,%.3lf,%.3lf\n",mpu_roll,mpu_pitch,mpu_yaw);
            //printf("%.0lf %.0lf %.0lf\r\n",roll,pitch,yaw);
            //uint8_t sendData[5]={0xA5,0,0,0,0x5A};
            //sendData[1] = (int)roll;
            //sendData[2] = (int)pitch;
            //sendData[3] = (int)yaw*120/180;
            
            
        }
        
        
    } else if (hal.new_gyro) {
        short gyro[3], accel[3];
        unsigned char sensors, more;
        /* This function gets new data from the FIFO. The FIFO can contain
         * gyro, accel, both, or neither. The sensors parameter tells the
         * caller which data fields were actually populated with new data.
         * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
         * being filled with accel data. The more parameter is non-zero if
         * there are leftover packets in the FIFO.
         */
        mpu_read_fifo(gyro, accel, &sensor_timestamp, &sensors, &more);
        if (!more)
            hal.new_gyro = 0;
        
    }
    
    return 0;
}

int stm32_i2c_write(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data)
{
    if (!length)
        return 0;	
	if(length > 24) return -1;
   
    HAL_I2C_Mem_Write(&MPU6500_I2C, slave_addr<<1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, length,0xfff);
    return 0;
}


int stm32_i2c_read(unsigned char slave_addr,
                   unsigned char reg_addr,
                   unsigned char length,
                   unsigned char *data)
{
    if (!length)
        return 0;
    
    HAL_I2C_Mem_Read(&MPU6500_I2C, slave_addr<<1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, length,0xfff);
    return 0;
}

int stm32_get_clock_ms(unsigned long *count)
{
    if (!count)
        return 1;
    count[0] = HAL_GetTick();
    return 0;
}
