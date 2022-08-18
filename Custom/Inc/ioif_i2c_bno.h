/*
 * ioif_i2c_bin.h
 *
 *  Created on: Jun 20, 2022
 *      Author: keonyoung
 */

#ifndef INC_IOIF_I2C_BNO_H_
#define INC_IOIF_I2C_BNO_H_


#include "cmsis_os.h"
#include "i2c.h"
#include "main.h"

#define IOIF_I2C_OK                  0
#define IOIF_I2C_ERROR              -1
#define IOIF_I2C_TIMEOUT            -2
#define IOIF_I2C_READ_ERROR         -3
#define IOIF_I2C_WRITE_ERROR        -4

int32_t IOIF_WaitASec(uint32_t timeout);
int32_t IOIF_I2C_MemTx(uint16_t dev_addr, uint16_t mem_addr, uint16_t mem_size, uint8_t *data, uint16_t size);
int32_t IOIF_I2C_Tx(uint16_t dev_addr, uint8_t *data, uint16_t size);
int32_t IOIF_I2C_Rx(uint16_t dev_addr, uint8_t *data, uint16_t size);


#endif /* INC_IOIF_I2C_BNO_H_ */
