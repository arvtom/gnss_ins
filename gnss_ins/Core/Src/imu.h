/*
 * mpu6050.h
 *
 *  Created on: 1 Dec 2021
 *      Author: tomku
 */

#ifndef SRC_IMU_H_
#define SRC_IMU_H_

#include "system_defines.h"

void imu_init_acc();
void imu_calibrate_acc(acc_t *p_x, acc_t *p_y, acc_t *p_z);
void imu_calibrate_gyro(gyro_t *p_x, gyro_t *p_y, gyro_t *p_z);
void imu_read_acc(acc_t *p_x, acc_t *p_y, acc_t *p_z);
void imu_read_gyro(gyro_t *p_x, gyro_t *p_y, gyro_t *p_z);

#endif /* SRC_IMU_H_ */
