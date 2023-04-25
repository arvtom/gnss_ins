/*
 * inertial_navigation.c
 *
 *  Created on: 3 Dec 2021
 *      Author: tomku
 */

#include "ins.h"

void joint_result_calculate_bearing(joint_result_t *p_joint, gnss_t *p_gnss, acc_t *p_acc, gyro_t *p_gyro)
{
	if(FALSE == g.flag.gnss_active)
	{
		//execute only once when gnss lost
		if(FALSE == g.flag.ins_begin)
		{

//			p_joint->latitude = p_gnss->lat;
//			p_joint->longitude = p_gnss->lon;
//			p_joint->velocity_m_s = p_gnss->kmh * 5 / 18;
//			p_joint->bearing = p_gnss->bearing;

			g.flag.ins_begin = TRUE;

		}

		p_joint->bearing = p_joint->bearing + p_gyro->value_filtered;
		p_joint->velocity_m_s = p_joint->velocity_m_s + p_acc->value_filtered * 1;
		p_joint->velocity_kmh = p_joint->velocity_m_s * 3.6;
		p_joint->distance = p_joint->velocity_m_s * 1;

		joint_result_calculate_lat_lon(p_joint);
	}
	else
	{
		p_joint->bearing = p_gnss->bearing;
		p_joint->latitude = p_gnss->lat;
		p_joint->longitude = p_gnss->lon;
		p_joint->velocity_m_s = p_gnss->kmh * 5 / 18;
		p_joint->velocity_kmh = p_joint->velocity_m_s * 3.6;
		p_joint->distance = p_joint->velocity_m_s * 1;

		g.flag.ins_begin = FALSE;
	}
}

float ins_calculate_2d_acceleration(float x, float y)
{
	float result = pow(x,2) + pow(y,2);

	return result;
}

float ins_calculate_3d_acceleration(float x, float y, float z)
{
	float temp = pow(x,2) + pow(y,2);
	float result = pow(temp,2) + pow(z,2);

	return result;
}

float ins_calculate_velocity(float *velocity_0, float acceleration, float delta_t)
{
	float estimate =  *velocity_0 + acceleration*delta_t;
	if(estimate > 150)
	{
		estimate = 150;
	}
	if(estimate < 0)
	{
		estimate = 0;
	}

	*velocity_0 = estimate;

	return estimate;
}

void joint_result_calculate_lat_lon(joint_result_t *p_joint)
{
	float R = 6371000;	//meters
	float bearing_rad = p_joint->bearing * PI / 180; //convert to radians
	float lat1 = p_joint->latitude * PI / 180; //convert to radians
	float lon1 = p_joint->longitude * PI / 180; //convert to radians

	float lat2 = asin(sin(lat1)*cos(p_joint->distance/R) + cos(lat1)*sin(p_joint->distance/R)*cos(bearing_rad));
	float lon2 = lon1 + atan2(sin(bearing_rad)*sin(p_joint->distance/R)*cos(lat1), cos(p_joint->distance/R)-sin(lat1)*sin(lat2));

	lat2 = lat2 * 180 / PI; //convert to degrees
	lon2 = lon2 * 180 / PI; //convert to degrees

	p_joint->latitude = lat2;
	p_joint->longitude = lon2;
}
