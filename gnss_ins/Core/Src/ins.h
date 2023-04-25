/*
 * inertial_navigation.h
 *
 *  Created on: 3 Dec 2021
 *      Author: tomku
 */

#ifndef SRC_INS_H_
#define SRC_INS_H_

#include "system_defines.h"

void joint_result_calculate_bearing(joint_result_t *p_joint, gnss_t *p_gnss, acc_t *p_acc, gyro_t *p_gyro);
float ins_calculate_2d_acceleration(float x, float y);
float ins_calculate_3d_acceleration(float x, float y, float z);
float ins_calculate_velocity(float *velocity_0, float acceleration, float delta_t);
void joint_result_calculate_lat_lon(joint_result_t *p_joint);

#endif /* SRC_INS_H_ */
