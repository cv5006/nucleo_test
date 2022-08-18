/*
 * ioif_i2c_mpu.h
 *
 *  Created on: 2022. 5. 31.
 *      Author: keonyoung
 */

#ifndef INC_IOIF_I2C_MPU_H_
#define INC_IOIF_I2C_MPU_H_

#include "cmsis_os.h"
#include "i2c.h"
#include "main.h"

#define IOIF_I2C_OK            0
#define IOIF_I2C_ERROR        -1
#define IOIF_I2C_TIMEOUT      -2
#define IOIF_I2C_READ_ERROR   -3
#define IOIF_I2c_WRITE_ERROR  -4

void IOIF_WaitForASec(uint32_t timeout);
void IOIF_I2C_WaitForI2C_Ready(uint16_t dev_addr, uint32_t timeout);
int32_t IOIF_I2C_RegisterRx(uint16_t dev_addr, uint16_t reg_addr, uint8_t* rx, uint16_t size);
int32_t IOIF_I2C_RegisterTx(uint16_t dev_addr, uint16_t reg_addr, uint8_t* tx, uint16_t size);

#endif /* INC_IOIF_I2C_MPU_H_ */
