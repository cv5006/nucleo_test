/*
 * mpu9250.c
 *
 *  Created on: 2022. 5. 31.
 *      Author: keonyoung
 */


#include "mpu9250.h"



static MPU9250_STATUS RegisterRx(MPU_obj* obj, uint16_t dev_addr, uint16_t reg_addr, uint8_t* rx, uint16_t size)
{
    if(obj->io.RegisterRx(dev_addr, reg_addr, rx, size) < 0){
    	return MPU9250_STATUS_READ_ERROR;
    }
    else return MPU9250_STATUS_OK;
}

static MPU9250_STATUS RegisterTx(MPU_obj* obj, uint16_t dev_addr, uint16_t reg_addr, uint8_t* tx, uint16_t size)
{
	if(obj->io.RegisterTx(dev_addr, reg_addr, tx, size) < 0){
		return MPU9250_STATUS_WRITE_ERROR;
	}
	else return MPU9250_STATUS_OK;
}

static void Wait(MPU_obj* obj, uint16_t dev_addr, uint32_t timeout)
{
    obj->io.Wait(dev_addr, timeout);
}

MPU9250_STATUS MPU9250_RegisterIOContext(MPU_obj* obj, MPU9250_IOctx* ioctx)
{
	if(!obj || !ioctx->Wait || !ioctx->RegisterRx || !ioctx->RegisterTx) return MPU9250_STATUS_ERROR;
    obj->io.Wait         = ioctx->Wait;
    obj->io.RegisterRx   = ioctx->RegisterRx;
    obj->io.RegisterTx   = ioctx->RegisterTx;
    return MPU9250_STATUS_OK;
}

// waitforready + configure
MPU9250_STATUS MPU9250_Initialize(MPU_obj* obj)
{
	MPU9250_STATUS status = MPU9250_STATUS_OK;
    // waitforready
    Wait(obj, IMU_ADDR_MPU, 1000);

    // imu_configure
    status = RegisterTx(obj, IMU_ADDR_MPU, IMU_REG_PWR_MGMT,   &((uint8_t){0x80}), 1);
    status = RegisterTx(obj, IMU_ADDR_MPU, IMU_REG_PWR_MGMT,   &((uint8_t){0x01}), 1);
    status = RegisterTx(obj, IMU_ADDR_MPU, IMU_REG_GYR_CONFIG, &((uint8_t){0x18}), 1);
    status = RegisterTx(obj, IMU_ADDR_MPU, IMU_REG_PIN_CONFIG, &((uint8_t){0x02}), 1);
    status = RegisterTx(obj, IMU_ADDR_MAG, IMU_REG_MAG_CONFIG, &((uint8_t){0x16}), 1);
    status = RegisterRx(obj, IMU_ADDR_MPU, IMU_REG_WHO_AM_I,   &(obj->imu_id), 1);

    return status;
}

MPU9250_STATUS MPU9250_ReadData(MPU_obj* obj)
{
	MPU9250_STATUS status = MPU9250_STATUS_OK;
    status = RegisterRx(obj, IMU_ADDR_MPU, IMU_REG_ACC_DATA, obj->acc_buff, 6);
    status = RegisterRx(obj, IMU_ADDR_MPU, IMU_REG_GYR_DATA, obj->gyr_buff, 6);
    uint8_t st1;
    status = RegisterRx(obj, IMU_ADDR_MAG, 0x02, &st1, 1);
    if((st1 & 0x11) == 0x01){
        status = RegisterRx(obj, IMU_ADDR_MAG, IMU_REG_MAG_DATA, obj->mag_buff, 7);
    }
    return status;
}

MPU9250_STATUS MPU9250_ReadBuffData(MPU_obj* obj){
    for(int i=0; i<3; i++){
        obj->acc[i] = obj->acc_buff[2*i] << 8 | obj->acc_buff[2*i+1];
        obj->gyr[i] = obj->gyr_buff[2*i] << 8 | obj->gyr_buff[2*i+1];
        obj->mag[i] = obj->mag_buff[2*i+1] << 8 | obj->mag_buff[2*i];
	}
    return MPU9250_STATUS_OK;

}
