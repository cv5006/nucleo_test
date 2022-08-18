/*
 * hal_wrapper.c
 *
 *  Created on: Nov 5, 2021
 *      Author: SHW
 */

#include "hal_wrapper.h"

/*
  ___ ___ ___  __      __                             ___             _   _             
 |_ _|_  ) __| \ \    / / _ __ _ _ __ _ __  ___ _ _  | __|  _ _ _  __| |_(_)___ _ _  ___
  | | / / (__   \ \/\/ / '_/ _` | '_ \ '_ \/ -_) '_| | _| || | ' \/ _|  _| / _ \ ' \(_-<
 |___/___\___|   \_/\_/|_| \__,_| .__/ .__/\___|_|   |_| \_,_|_||_\__|\__|_\___/_||_/__/
                                |_|  |_|                                                
*/


HW_I2C_Interface i2c_ch1;

void HW_I2C_InitChannel(HW_I2C_Interface* i2c_ch, I2C_HandleTypeDef* hi2c, osSemaphoreId_t sem)
{
    i2c_ch->hal_handle = hi2c;
    i2c_ch->sem = sem;
}

void HW_I2C_WaitForI2C_Ready(HW_I2C_Interface* i2c_ch, uint16_t dev_addr, uint32_t time_out)
{
    while(HAL_I2C_IsDeviceReady(i2c_ch->hal_handle, dev_addr, 1, time_out) != HAL_OK){
        osDelay(1);
    }
}


void HW_I2C_RegisterRx(HW_I2C_Interface* i2c_ch, uint16_t dev_addr, uint16_t reg_addr, uint8_t* rx, uint16_t size)
{
    if(osSemaphoreAcquire(i2c_ch->sem, osWaitForever) == osOK){
        i2c_ch->status = HAL_I2C_Mem_Read_DMA(i2c_ch->hal_handle, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, rx, size);
    }
}

void HW_I2C_RegisterTx(HW_I2C_Interface* i2c_ch, uint16_t dev_addr, uint16_t reg_addr, uint8_t* tx, uint16_t size)
{
    if(osSemaphoreAcquire(i2c_ch->sem, osWaitForever) == osOK){
        i2c_ch->status = HAL_I2C_Mem_Write_DMA(i2c_ch->hal_handle, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, tx, size);
    }
}


//void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//    if(hi2c == i2c_ch1.hal_handle){
//        osSemaphoreRelease(i2c_ch1.sem);
//    }
//}
//
//void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//    if(hi2c == i2c_ch1.hal_handle){
//        osSemaphoreRelease(i2c_ch1.sem);
//    }
//}
