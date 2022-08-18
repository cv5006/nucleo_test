/*
 * bno055.c
 *
 *  Created on: Apr 19, 2022
 *      Author: SHW
 */

#include "bno055_driver.h"
#include "bno055.h"

//extern int isitgoingright;

static BNO055_Status ReadReg(bno055_Object* obj, uint16_t reg, uint8_t* data, uint32_t size)
{
    if (obj->io.ReadReg(obj->addr, reg, data, size) < 0) {
        return BNO055_STATUS_READ_ERROR;
    } else {
        return BNO055_STATUS_OK;
    }
}

static BNO055_Status WriteReg(bno055_Object* obj, uint16_t reg, uint8_t* data, uint32_t size)
{
    if (obj->io.WriteReg(obj->addr, reg, data, size) < 0) {
        return BNO055_STATUS_WRITE_ERROR;
    } else {
        return BNO055_STATUS_OK;
    }
}

static BNO055_Status RegSetting(bno055_Object* obj, uint16_t reg, uint8_t* data, uint32_t size)
{
    BNO055_Status status = BNO055_STATUS_OK;
    uint8_t readval[size];

    status = WriteReg(obj, reg, data, size);

    if (status == BNO055_STATUS_OK) {
        status = ReadReg(obj, reg, readval, size);
    }

    if (status == BNO055_STATUS_OK) {
        if (memcmp(data, &readval, size) != 0) {
            status = BNO055_STATUS_ERROR;
        }
    }

    return status;
}

static BNO055_Status SetPage(bno055_Object* obj, uint8_t page_id)
{
    uint8_t page = page_id;
    return RegSetting(obj, BNO055_PAGE_REG, &page, BNO055_PAGE_SIZE);
}

BNO055_Status BNO055_RegisterIOContext(bno055_Object* obj, bno055_IOctx* ioctx)
{
    if (!obj || !ioctx->IsDevReady || !ioctx->ReadReg || !ioctx->WriteReg || !ioctx->Wait) {
        return BNO055_STATUS_ERROR;
    }

    obj->io.Init       = ioctx->Init;
    obj->io.DeInit     = ioctx->DeInit;
    obj->io.IsDevReady = ioctx->IsDevReady;
    obj->io.ReadReg    = ioctx->ReadReg;
    obj->io.WriteReg   = ioctx->WriteReg;
    obj->io.Wait       = ioctx->Wait;

    return BNO055_STATUS_OK;
}

BNO055_Status BNO055_Init(bno055_Object* obj)
{
    BNO055_Status status = BNO055_STATUS_OK;

    // Init IO Context
    if (!obj->is_initialized) {
        if (obj->io.Init != 0) {
            obj->io.Init();
        }
    }

    // Find Address
    status = BNO055_STATUS_ADDRESS_ERROR;
    for (uint16_t hwid = 0; hwid <= BNO055_DEV_ADDR_MAX_HW_ID; ++hwid) {
        uint16_t addr = (BNO055_DEV_ADDR_PREFIX | hwid) << 1;
        if (obj->io.IsDevReady(addr, 10, BNO055_TIMEOUT) == 0) {
        	obj->addr = addr;
            status = BNO055_STATUS_OK;
            break;
        }
    }

    // Set Page to 0
    if (status == BNO055_STATUS_OK) {
        status = SetPage(obj, BNO055_PAGE_0);

        // Enter the Config mode
        if (status == BNO055_STATUS_OK) {
            uint8_t opr_mode = BNO055_OPR_MODE_CONFIGMODE;
            status = RegSetting(obj, BNO055_OPR_MODE_REG, &opr_mode, BNO055_OPR_MODE_SIZE);
        }

        // Axis setting
        if (status == BNO055_STATUS_OK) {
            // Set axis
            uint8_t axis = BNO055_AXIS_CONF_X_TO(BNO055_AXIS_CONF_X_AXIS) |
                        BNO055_AXIS_CONF_Y_TO(BNO055_AXIS_CONF_Z_AXIS) |
                        BNO055_AXIS_CONF_Z_TO(BNO055_AXIS_CONF_Y_AXIS);
            status = RegSetting(obj, BNO055_AXIS_CONF_REG, &axis, BNO055_AXIS_CONF_SIZE);

            // Set sign
            uint8_t sign = BNO055_AXIS_SIGN_X_TO(BNO055_AXIS_SIGN_N) |
                        BNO055_AXIS_SIGN_Y_TO(BNO055_AXIS_SIGN_P) |
                        BNO055_AXIS_SIGN_Z_TO(BNO055_AXIS_SIGN_P);
            status = RegSetting(obj, BNO055_AXIS_SIGN_REG, &sign, BNO055_AXIS_SIGN_SIZE);
        }
    }

    // Set Page to 1
    if (status == BNO055_STATUS_OK) {
        status = SetPage(obj, BNO055_PAGE_1);

        // ACC configuration
        if (status == BNO055_STATUS_OK) {
            uint8_t acc_conf = BNO055_ACC_CONFIG_RANGE_4G | BNO055_ACC_CONFIG_BAND_62HZ5 | BNO055_ACC_CONFIG_MODE_NORMAL;
            status = RegSetting(obj, BNO055_ACC_CONFIG_REG, &acc_conf, BNO055_ACC_CONFIG_SIZE);
        }

        // GYR configuration
        if (status == BNO055_STATUS_OK) {
            uint8_t gyr_conf = BNO055_GYR_CONFIG_RANGE_2000DPS| BNO055_GYR_CONFIG_BAND_32HZ | BNO055_ACC_CONFIG_MODE_NORMAL;
            status = RegSetting(obj, BNO055_GYR_CONFIG_REG, &gyr_conf, BNO055_GYR_CONFIG_SIZE);
        }
    }

    // Return to Page 0
    if (status == BNO055_STATUS_OK) {
        status = SetPage(obj, BNO055_PAGE_0);

        // Set Operation Mode
        if (status == BNO055_STATUS_OK) {
            uint8_t opr_mode = BNO055_OPR_MODE_ACCMAGGYR;
            status = RegSetting(obj, BNO055_OPR_MODE_REG, &opr_mode, BNO055_OPR_MODE_SIZE);
        }
    }
    return status;
}

BNO055_Status BNO055_DeInit(bno055_Object* obj)
{
    return BNO055_STATUS_OK;
}

BNO055_Status BNO055_ScanAll(bno055_Object* obj, uint8_t* acc, uint8_t* gyr, uint8_t* mag)
{

    uint8_t data[BNO055_ALL_DATA_SIZE];
    if (obj->io.ReadReg(obj->addr, BNO055_ACC_DATA_X_REG, data, BNO055_ALL_DATA_SIZE) < 0) {
//    	isitgoingright = 2;
        return BNO055_STATUS_READ_ERROR;
    }

    memcpy(acc, &data[0],  BNO055_ACC_DATA_SIZE);
    memcpy(mag, &data[6],  BNO055_MAG_DATA_SIZE);
    memcpy(gyr, &data[12], BNO055_GYR_DATA_SIZE);
    return BNO055_STATUS_OK;
}



