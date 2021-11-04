/*
 * imu.h
 *
 *  Created on: Oct 31, 2021
 *      Author: Seo
 */

/**
 * @brief 
 * 
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_


#include "cmsis_os.h"
#include "i2c.h"


/*
   ___           __ _                    _   _             
  / __|___ _ _  / _(_)__ _ _  _ _ _ __ _| |_(_)___ _ _  ___
 | (__/ _ \ ' \|  _| / _` | || | '_/ _` |  _| / _ \ ' \(_-<
  \___\___/_||_|_| |_\__, |\_,_|_| \__,_|\__|_\___/_||_/__/
                     |___/                                 
*/

#define IMU_ADDR_MPU 0b11010000
#define IMU_ADDR_MAG 0b00011000
#define IMU_REG_PWR_MGMT 0x6B
#define IMU_REG_WHO_AM_I 0x75
#define IMU_REG_PIN_CONFIG 0x37
#define IMU_REG_GYR_CONFIG 0x1B
#define IMU_REG_MAG_CONFIG 0x0A
#define IMU_REG_ACC_DATA 0x3B
#define IMU_REG_GYR_DATA 0x43
#define IMU_REG_MAG_DATA 0x03

/*
  ___ _               _     __      __   __        _      _    _        
 / __| |_ _ _ _  _ __| |_  / _|___  \ \ / /_ _ _ _(_)__ _| |__| |___ ___
 \__ \  _| '_| || / _|  _| > _|_ _|  \ V / _` | '_| / _` | '_ \ / -_|_-<
 |___/\__|_|  \_,_\__|\__| \_____|    \_/\__,_|_| |_\__,_|_.__/_\___/__/

*/

typedef struct{
    HAL_StatusTypeDef i2c_status;

    uint8_t imu_id;

    uint8_t acc_buff[6];
    uint8_t gyr_buff[6];
    uint8_t mag_buff[7];

    int16_t acc[3];
    int16_t gyr[3];
    int16_t mag[3];
}IMU_Handle;

extern IMU_Handle himu;


/*
  ___             _   _             
 | __|  _ _ _  __| |_(_)___ _ _  ___
 | _| || | ' \/ _|  _| / _ \ ' \(_-<
 |_| \_,_|_||_\__|\__|_\___/_||_/__/
                                    
*/

void IMU_WaitForReady(IMU_Handle* himu);

void IMU_Init(IMU_Handle* himu);
void IMU_ReadData(IMU_Handle* himu);
void IMU_ReadBuffData(IMU_Handle* himu);

// void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);


#endif /* INC_IMU_H_ */
