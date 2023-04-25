/*
 * mpu6050.c
 *
 *  Created on: 1 Dec 2021
 *      Author: tomku
 */

#include "imu.h"

//ACC SETTINGS
#define ACC_ADDR 0xD0
#define ACC_CHECK 0x75
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B

void imu_init_acc()
{
	//check acc
	while(g.buffer.i2c3_rx[0]!=0x68)
	{
		HAL_I2C_Mem_Read(&hi2c3, ACC_ADDR,ACC_CHECK,1, (uint8_t *)g.buffer.i2c3_rx, 1, 1000);
		if(g.buffer.i2c3_rx[0]==0x68)
			break;
	}

	// power management register 0X6B we should write all 0's to wake the sensor up
	g.buffer.i2c3_tx[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c3, ACC_ADDR, PWR_MGMT_1_REG, 1, (uint8_t *)g.buffer.i2c3_tx, 1, 1000);

	// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
	g.buffer.i2c3_tx[0] = 0x07;
	HAL_I2C_Mem_Write(&hi2c3, ACC_ADDR, SMPLRT_DIV_REG, 1, (uint8_t *)g.buffer.i2c3_tx, 1, 1000);

	// Set accelerometer configuration in ACCEL_CONFIG Register
	// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
	g.buffer.i2c3_tx[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c3, ACC_ADDR, ACCEL_CONFIG_REG, 1, (uint8_t *)g.buffer.i2c3_tx, 1, 1000);

	// Set Gyroscopic configuration in GYRO_CONFIG Register
	// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
	g.buffer.i2c3_tx[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c3, ACC_ADDR, GYRO_CONFIG_REG, 1, (uint8_t *)g.buffer.i2c3_tx, 1, 1000);
}

void imu_calibrate_acc(acc_t *p_x, acc_t *p_y, acc_t *p_z)
{
	p_x->offset = g.filter_acc_x.value;
	p_y->offset = g.filter_acc_y.value;
	p_z->offset = g.filter_acc_z.value;
}

void imu_calibrate_gyro(gyro_t *p_x, gyro_t *p_y, gyro_t *p_z)
{
	p_x->offset = g.filter_gyro_x.value;
	p_y->offset = g.filter_gyro_y.value;
	p_z->offset = g.filter_gyro_z.value;
}

void imu_read_acc(acc_t *p_x, acc_t *p_y, acc_t *p_z)
{
	HAL_I2C_Mem_Read (&hi2c3, ACC_ADDR, ACCEL_XOUT_H_REG, 1, (uint8_t *)g.buffer.i2c3_rx, 6, 1000);

	p_x->raw = (int16_t)(g.buffer.i2c3_rx[0] << 8 | g.buffer.i2c3_rx[1]);
	p_y->raw = (int16_t)(g.buffer.i2c3_rx[2] << 8 | g.buffer.i2c3_rx[3]);
	p_z->raw = (int16_t)(g.buffer.i2c3_rx[4] << 8 | g.buffer.i2c3_rx[5]);

	p_x->value = (float)p_x->raw/16384 - p_x->offset;
	p_y->value = (float)p_y->raw/16384 - p_y->offset;
	p_z->value = (float)p_z->raw/16384 - p_z->offset;
}

void imu_read_gyro(gyro_t *p_x, gyro_t *p_y, gyro_t *p_z)
{
	HAL_I2C_Mem_Read (&hi2c3, ACC_ADDR, GYRO_XOUT_H_REG, 1, (uint8_t *)g.buffer.i2c3_rx, 6, 1000);

	p_x->raw = (int16_t)(g.buffer.i2c3_rx[0] << 8 | g.buffer.i2c3_rx[1]);
	p_y->raw = (int16_t)(g.buffer.i2c3_rx[2] << 8 | g.buffer.i2c3_rx[3]);
	p_z->raw = (int16_t)(g.buffer.i2c3_rx[4] << 8 | g.buffer.i2c3_rx[5]);

	p_x->value = (float)p_x->raw/131 - p_x->offset;
	p_y->value = (float)p_y->raw/131 - p_y->offset;
	p_z->value = (float)p_z->raw/131 - p_z->offset;
}


