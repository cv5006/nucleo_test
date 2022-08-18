/*
 * bno055_driver.h
 *
 *  Created on: Jun 20, 2022
 *      Author: keonyoung
 */

#ifndef INC_BNO055_DRIVER_H_
#define INC_BNO055_DRIVER_H_

#include "bno055.h"
#include "ioif_i2c_bno.h"


void BNO_Init();
void BNO_Run();
void Serial_Send();


#endif /* INC_BNO055_DRIVER_H_ */
