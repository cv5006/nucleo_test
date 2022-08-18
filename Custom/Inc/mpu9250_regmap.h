/*
 * mpu9250_regmap.h
 *
 *  Created on: 2022. 5. 31.
 *      Author: keonyoung
 */

#ifndef INC_MPU9250_REGMAP_H_
#define INC_MPU9250_REGMAP_H_

/*
   ___           __ _                    _   _
  / __|___ _ _  / _(_)__ _ _  _ _ _ __ _| |_(_)___ _ _  ___
 | (__/ _ \ ' \|  _| / _` | || | '_/ _` |  _| / _ \ ' \(_-<
  \___\___/_||_|_| |_\__, |\_,_|_| \__,_|\__|_\___/_||_/__/
                     |___/
*/

#define IMU_ADDR_MPU 0b11010000
#define IMU_ADDR_MAG 0b00011000
#define IMU_REG_PWR_MGMT 0x6B
#define IMU_REG_WHO_AM_I 0x75
#define IMU_REG_PIN_CONFIG 0x37
#define IMU_REG_GYR_CONFIG 0x1B
#define IMU_REG_MAG_CONFIG 0x0A
#define IMU_REG_ACC_DATA 0x3B
#define IMU_REG_GYR_DATA 0x43
#define IMU_REG_MAG_DATA 0x03

#endif /* INC_MPU9250_REGMAP_H_ */
