/*
 * imu.c
 *
 *  Created on: Oct 31, 2021
 *      Author: Seo
 */


#include "imu.h"


IMU_Handle himu;

void IMU_SetI2C_Channel(HW_I2C_Channel* i2c_ch)
{
    himu.i2c_ch = i2c_ch;
}

void IMU_WaitForReady()
{
    HW_I2C_WaitForI2C_Ready(himu.i2c_ch, IMU_ADDR_MPU, 1000);
}

void IMU_Configure()
{
    HW_I2C_RegisterTx(himu.i2c_ch, IMU_ADDR_MPU, IMU_REG_PWR_MGMT,   &((uint8_t){0x80}), 1);
    HW_I2C_RegisterTx(himu.i2c_ch, IMU_ADDR_MPU, IMU_REG_PWR_MGMT,   &((uint8_t){0x01}), 1);
    HW_I2C_RegisterTx(himu.i2c_ch, IMU_ADDR_MPU, IMU_REG_GYR_CONFIG, &((uint8_t){0x18}), 1);
    HW_I2C_RegisterTx(himu.i2c_ch, IMU_ADDR_MPU, IMU_REG_PIN_CONFIG, &((uint8_t){0x02}), 1);
    HW_I2C_RegisterTx(himu.i2c_ch, IMU_ADDR_MAG, IMU_REG_MAG_CONFIG, &((uint8_t){0x16}), 1);
    HW_I2C_RegisterRx(himu.i2c_ch, IMU_ADDR_MPU, IMU_REG_WHO_AM_I,   &(himu.imu_id), 1); 
}

void IMU_ReadData()
{
    HW_I2C_RegisterRx(himu.i2c_ch, IMU_ADDR_MPU, IMU_REG_ACC_DATA, himu.acc_buff, 6);
    HW_I2C_RegisterRx(himu.i2c_ch, IMU_ADDR_MPU, IMU_REG_GYR_DATA, himu.gyr_buff, 6);
    uint8_t st1;
    HW_I2C_RegisterRx(himu.i2c_ch, IMU_ADDR_MAG, 0x02, &st1, 1);
    if ((st1 & 0x11) == 0x01) { // ST2 must be read!
        HW_I2C_RegisterRx(himu.i2c_ch, IMU_ADDR_MAG, IMU_REG_MAG_DATA, himu.mag_buff, 7);
    }
}

void IMU_ReadBuffData()
{
    for(int i=0; i<3; i++){
        himu.acc[i] = himu.acc_buff[2*i] << 8 | himu.acc_buff[2*i+1];
        himu.gyr[i] = himu.gyr_buff[2*i] << 8 | himu.gyr_buff[2*i+1];
        himu.mag[i] = himu.mag_buff[2*i+1] << 8 | himu.mag_buff[2*i];
	}
}