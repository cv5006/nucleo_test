/*
 * bno055.h
 *
 *  Created on: Apr 19, 2022
 *      Author: SHW
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#include "stdint.h"
#include "string.h"
#include "bno055_regmap.h"

#define BNO055_STATUS_OK             0
#define BNO055_STATUS_ERROR         -1
#define BNO055_STATUS_TIMEOUT       -2
#define BNO055_STATUS_ADDRESS_ERROR -3
#define BNO055_STATUS_READ_ERROR    -4
#define BNO055_STATUS_WRITE_ERROR   -5

#define BNO055_TIMEOUT 200

#define BNO055_DEFAULT_GCC   0xFF
#define BNO055_DEFAULT_SCALE 0xFF

typedef int32_t BNO055_Status;

typedef int32_t (*BNO055_Init_Fnc)       (void);
typedef int32_t (*BNO055_DeInit_Fnc)     (void);
typedef int32_t (*BNO055_IsDevReady_Fnc) (uint16_t, uint32_t, uint32_t); // DevAddr, Trials, Timeout
typedef int32_t (*BNO055_ReadReg_Fnc)    (uint16_t, uint16_t, uint8_t*, uint16_t); // DevAddr, RegAddr, Data, DataSize
typedef int32_t (*BNO055_WriteReg_Fnc)   (uint16_t, uint16_t, uint8_t*, uint16_t); // DevAddr, RegAddr, Data, DataSize
typedef int32_t (*BNO055_Wait_Fnc)       (uint32_t); // ms to wait

// IO context
typedef struct bno055_IOctx {
    BNO055_Init_Fnc       Init;
    BNO055_DeInit_Fnc     DeInit;
    BNO055_IsDevReady_Fnc IsDevReady;
    BNO055_ReadReg_Fnc    ReadReg;
    BNO055_WriteReg_Fnc   WriteReg;
    BNO055_Wait_Fnc       Wait;
} bno055_IOctx;

// Object handle
typedef struct bno055_Object {
    uint16_t addr;
    uint16_t is_initialized;
    bno055_IOctx io;
    void* data;
} bno055_Object;

BNO055_Status BNO055_RegisterIOContext(bno055_Object* obj, bno055_IOctx* ioctx);
BNO055_Status BNO055_Init(bno055_Object* obj);
BNO055_Status BNO055_DeInit(bno055_Object* obj);

BNO055_Status BNO055_ScanAll(bno055_Object* obj, uint8_t* acc, uint8_t* gyr, uint8_t* mag);

#endif /* INC_BNO055_H_ */
