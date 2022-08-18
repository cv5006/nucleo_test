/*
 * mpu9250.h
 *
 *  Created on: 2022. 5. 31.
 *      Author: keonyoung
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

#include "mpu9250_regmap.h"
#include <stdint.h>

// typedef void (*MPU9250_SetI2CChannel_Fnc) (IOIF_I2C_Interface*);
// typedef void (*MPU9250_Configure_Fnc)     (IOIF_I2C_Interface*, uint16_t, uint16_t, uint8_t*, uint16_t);
typedef void (*MPU9250_Wait_Fnc)          (uint16_t, uint32_t);
typedef int32_t (*MPU9250_RegisterRx_Fnc)    (uint16_t, uint16_t, uint8_t*, uint16_t);
typedef int32_t (*MPU9250_RegisterTx_Fnc)    (uint16_t, uint16_t, uint8_t*, uint16_t);

typedef int32_t MPU9250_STATUS;

#define MPU9250_STATUS_OK                0
#define MPU9250_STATUS_ERROR            -1
#define MPU9250_STATUS_TIMEOUT          -2
#define MPU9250_STATUS_ADDRESS_ERROR    -3
#define MPU9250_STATUS_READ_ERROR       -4
#define MPU9250_STATUS_WRITE_ERROR      -5


/*
  ___ _               _     __      __   __        _      _    _
 / __| |_ _ _ _  _ __| |_  / _|___  \ \ / /_ _ _ _(_)__ _| |__| |___ ___
 \__ \  _| '_| || / _|  _| > _|_ _|  \ V / _` | '_| / _` | '_ \ / -_|_-<
 |___/\__|_|  \_,_\__|\__| \_____|    \_/\__,_|_| |_\__,_|_.__/_\___/__/

*/
typedef struct MPU9250_IOctx {
    // MPU9250_SetI2CChannel_Fnc   SetI2CChannel;
    // MPU9250_Configure_Fnc       Configure;
    MPU9250_Wait_Fnc            Wait;
    MPU9250_RegisterRx_Fnc      RegisterRx;
    MPU9250_RegisterTx_Fnc      RegisterTx;
} MPU9250_IOctx;

typedef struct{
    uint8_t imu_id;

    uint8_t acc_buff[6];
    uint8_t gyr_buff[6];
    uint8_t mag_buff[7];

    int16_t acc[3];
    int16_t gyr[3];
    int16_t mag[3];

    MPU9250_IOctx io;
}MPU_obj;

/*
  ___             _   _
 | __|  _ _ _  __| |_(_)___ _ _  ___
 | _| || | ' \/ _|  _| / _ \ ' \(_-<
 |_| \_,_|_||_\__|\__|_\___/_||_/__/

*/
MPU9250_STATUS MPU9250_RegisterIOContext(MPU_obj* obj, MPU9250_IOctx* ioctx);
MPU9250_STATUS MPU9250_Initialize(MPU_obj* obj);

MPU9250_STATUS MPU9250_ReadData(MPU_obj* obj);
MPU9250_STATUS MPU9250_ReadBuffData(MPU_obj* obj);

#endif /* INC_MPU9250_H_ */
