/*
 * hal_wrapper.h
 *
 *  Created on: Nov 5, 2021
 *      Author: SHW
 */

#ifndef INC_HAL_WRAPPER_H_
#define INC_HAL_WRAPPER_H_

#include "cmsis_os.h"
#include "i2c.h"

typedef struct{
    HAL_StatusTypeDef status;
    osSemaphoreId_t sem;
    I2C_HandleTypeDef* hal_handle;
}HW_I2C_Channel;

extern HW_I2C_Channel i2c_ch1;

void HW_I2C_WaitForI2C_Ready(HW_I2C_Channel* i2c_ch, uint16_t dev_addr, uint32_t time_out);

void HW_I2C_InitChannel(HW_I2C_Channel* i2c_ch, I2C_HandleTypeDef* hi2c, osSemaphoreId_t sem);
void HW_I2C_RegisterRx(HW_I2C_Channel* i2c_ch, uint16_t dev_addr, uint16_t reg_addr, uint8_t* rx, uint16_t size);
void HW_I2C_RegisterTx(HW_I2C_Channel* i2c_ch, uint16_t dev_addr, uint16_t reg_addr, uint8_t* tx, uint16_t size);


// void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
// void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);


#endif /* INC_HAL_WRAPPER_H_ */
