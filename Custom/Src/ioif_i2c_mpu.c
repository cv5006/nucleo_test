/*
 * ioif_i2c_mpu.c
 *
 *  Created on: 2022. 5. 31.
 *      Author: keonyoung
 */

#include "ioif_i2c_mpu.h"

extern osSemaphoreId_t I2C1BinSemHandle;
I2C_HandleTypeDef hi2c1;


void IOIF_WaitForASec(uint32_t timeout)
{
    osDelay(timeout);
}

void IOIF_I2C_WaitForI2C_Ready(uint16_t dev_addr, uint32_t timeout)
{
    while(HAL_I2C_IsDeviceReady(&hi2c1, dev_addr, 1, timeout) != HAL_OK){
        osDelay(1);
    }
}

int32_t IOIF_I2C_RegisterRx(uint16_t dev_addr, uint16_t reg_addr, uint8_t* rx, uint16_t size){
    if(osSemaphoreAcquire(I2C1BinSemHandle, osWaitForever) == osOK){
        if(HAL_I2C_Mem_Read_DMA(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, rx, size) == HAL_OK){
        	return IOIF_I2C_OK;
        }
        else return IOIF_I2C_READ_ERROR;
    }
    else return IOIF_I2C_ERROR;
}

int32_t IOIF_I2C_RegisterTx(uint16_t dev_addr, uint16_t reg_addr, uint8_t* tx, uint16_t size)
{
    if(osSemaphoreAcquire(I2C1BinSemHandle, osWaitForever) == osOK){
        if(HAL_I2C_Mem_Write_DMA(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, tx, size) == HAL_OK){
        	return IOIF_I2C_OK;
        }
        else return IOIF_I2c_WRITE_ERROR;
    }
    else return IOIF_I2C_ERROR;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c == &hi2c1){
		osSemaphoreRelease(I2C1BinSemHandle);
	}
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c1){
		osSemaphoreRelease(I2C1BinSemHandle);
	}
}
