/*
 * ioif_i2c_bno.c
 *
 *  Created on: Jun 20, 2022
 *      Author: keonyoung
 */

#include "bno055_driver.h"
#include "ioif_i2c_bno.h"

extern osSemaphoreId_t I2C1BinSemHandle;
I2C_HandleTypeDef hi2c1;
uint8_t i2c1_buff[1000] __attribute__((section(".MY_Section")));

int32_t IOIF_WaitASec(uint32_t timeout)
{
	osDelay(timeout);
	return IOIF_I2C_OK;
}

int32_t IOIF_I2C_MemTx(uint16_t dev_addr, uint16_t mem_addr, uint16_t mem_size, uint8_t *data, uint16_t size)
{
    if(osSemaphoreAcquire(I2C1BinSemHandle, osWaitForever) == osOK) {
        memcpy(i2c1_buff, data, size);
        if(HAL_I2C_Mem_Write_DMA(&hi2c1, dev_addr, mem_addr, mem_size, i2c1_buff, size) == HAL_OK) {
            return IOIF_I2C_OK;
        } else {
            return IOIF_I2C_WRITE_ERROR;
        }
    } else {
        return IOIF_I2C_TIMEOUT;
    }
}

int32_t IOIF_I2C_Tx(uint16_t dev_addr, uint8_t *data, uint16_t size)
{
    if(osSemaphoreAcquire(I2C1BinSemHandle, osWaitForever) == osOK) {
        memcpy(i2c1_buff, data, size);
        if(HAL_I2C_Master_Seq_Transmit_DMA(&hi2c1, dev_addr, i2c1_buff, size, I2C_FIRST_FRAME) == HAL_OK) {
            return IOIF_I2C_OK;
        } else {
            return IOIF_I2C_WRITE_ERROR;
        }
    } else {
        return IOIF_I2C_TIMEOUT;
    }
}

int32_t IOIF_I2C_Rx(uint16_t dev_addr, uint8_t *data, uint16_t size)
{

    if (osSemaphoreAcquire(I2C1BinSemHandle, osWaitForever) == osOK) {
//    	isitgoingright = 2;
        if(HAL_I2C_Master_Seq_Receive_IT(&hi2c1, dev_addr, i2c1_buff, size, I2C_LAST_FRAME) != HAL_OK) {
//        	isitgoingright = 2;
            return IOIF_I2C_READ_ERROR;
        }
    } else {
//    	isitgoingright = 2;
        return IOIF_I2C_TIMEOUT;
    }



    if (osSemaphoreAcquire(I2C1BinSemHandle, osWaitForever) == osOK) {
        memcpy(data, i2c1_buff, size);
        osSemaphoreRelease(I2C1BinSemHandle);
        return IOIF_I2C_OK;
    } else {
        return IOIF_I2C_TIMEOUT;
    }
//    isitgoingright = 2;
}


void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c == &hi2c1) {
        osSemaphoreRelease(I2C1BinSemHandle);
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c == &hi2c1) {
        if(hi2c == &hi2c1) {
            osSemaphoreRelease(I2C1BinSemHandle);
        }
    }
}


