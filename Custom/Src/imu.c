/*
 * imu.c
 *
 *  Created on: Oct 31, 2021
 *      Author: Seo
 */


#include "imu.h"


IMU_Handle himu;

/*
  ___     _          _       
 | _ \_ _(_)_ ____ _| |_ ___ 
 |  _/ '_| \ V / _` |  _/ -_)
 |_| |_| |_|\_/\__,_|\__\___|
                                                                                     
*/

static void RegisterRx(uint16_t dev_addr, uint16_t reg_addr, uint8_t* rx, uint16_t size)
{
    if(osSemaphoreAcquire(himu.i2c1_binsem, osWaitForever) == osOK){
        himu.i2c_status = HAL_I2C_Mem_Read_DMA(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, rx, size);
    }
    
}

static void RegisterTx(uint16_t dev_addr, uint16_t reg_addr, uint8_t* tx, uint16_t size)
{
    if(osSemaphoreAcquire(himu.i2c1_binsem, osWaitForever) == osOK){
        himu.i2c_status = HAL_I2C_Mem_Write_DMA(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, tx, size);
    }
}


/*
  ___      _    _ _    
 | _ \_  _| |__| (_)__ 
 |  _/ || | '_ \ | / _|
 |_|  \_,_|_.__/_|_\__|
                       
*/


void IMU_SetBinSem(osSemaphoreId_t i2c_binsem){
    himu.i2c1_binsem = i2c_binsem;
}

void IMU_WaitForReady()
{
    while(HAL_I2C_IsDeviceReady(&hi2c1, IMU_ADDR_MPU, 1, 1000) != HAL_OK){
        osDelay(1);
    }
}

void IMU_Init()
{
    RegisterTx(IMU_ADDR_MPU, IMU_REG_PWR_MGMT,   &((uint8_t){0x80}), 1);
    RegisterTx(IMU_ADDR_MPU, IMU_REG_PWR_MGMT,   &((uint8_t){0x01}), 1);
    RegisterTx(IMU_ADDR_MPU, IMU_REG_GYR_CONFIG, &((uint8_t){0x18}), 1);
    RegisterTx(IMU_ADDR_MPU, IMU_REG_PIN_CONFIG, &((uint8_t){0x02}), 1);
    RegisterTx(IMU_ADDR_MAG, IMU_REG_MAG_CONFIG, &((uint8_t){0x16}), 1);
    RegisterRx(IMU_ADDR_MPU, IMU_REG_WHO_AM_I,   &(himu.imu_id), 1);   
}

void IMU_ReadData()
{
    RegisterRx(IMU_ADDR_MPU, IMU_REG_ACC_DATA, himu.acc_buff, 6);
    RegisterRx(IMU_ADDR_MPU, IMU_REG_GYR_DATA, himu.gyr_buff, 6);
    uint8_t st1;
    RegisterRx(IMU_ADDR_MAG, 0x02, &st1, 1);
    if ((st1 & 0x11) == 0x01) { // ST2 must be read!
        RegisterRx(IMU_ADDR_MAG, IMU_REG_MAG_DATA, himu.mag_buff, 7);
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

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    osSemaphoreRelease(himu.i2c1_binsem);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    osSemaphoreRelease(himu.i2c1_binsem);
}