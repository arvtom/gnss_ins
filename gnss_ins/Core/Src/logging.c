/*
 * logging.c
 *
 *  Created on: 5 Dec 2021
 *      Author: tomku
 */


//snprintf(microsd_buf, MICROSD_BUFFER_SIZE, "%4.2f;%4.2f;%3.2f;"
////				"%3.2f;%4.2f;"
////				"%3.2f;%4.2f;%3.2f;"
////				"%3.2f;%3.2f;%3.2f;"
////				"%d;%d;%d;%d\r\n",f_uwb1_cm.value,f_uwb2_cm.value,f_usonic_cm.value,
////				f_trig_angle_deg.value,f_trig_distance_cm.value,
////				pid_angle.error,pid_distance.error,pid_altitude.error,
////				pid_angle.pidsum,pid_distance.pidsum,pid_altitude.pidsum,
////				object_tracking,time_10hz,dwm_loc_get1.packet_loss[0],dwm_loc_get2.packet_loss[0]
////				);

#include "logging.h"
#include "system_defines.h"
//#include "stdint.h"
#include "stdio.h"
#include "usart.h"
#include <string.h>

void logging()
{

	uint8_t size = snprintf(g.buffer.uart7_tx, LOGGING_BUFFER_SIZE, "%d;%.4f;%.4f;%d;%d;",
			g.counter.tim4,g.acc_x.value_filtered, g.gyro_z.value_filtered, g.flag.gnss_active, g.flag.ins_begin);

	//check snprintf error
	if(size > 0)
	{
		HAL_UART_Transmit(&huart7, (uint8_t *)g.buffer.uart7_tx, size, UART_TX_TIMEOUT);
	}

	size = snprintf(g.buffer.uart7_tx, LOGGING_BUFFER_SIZE,
					"%.8f;%.8f;%.4f;%.4f;%.4f;",
				g.gnss.lat, g.gnss.lon, g.gnss.kmh, g.gnss.m_s, g.gnss.bearing);

	if(size > 0)
	{
		HAL_UART_Transmit(&huart7, (uint8_t *)g.buffer.uart7_tx, size, UART_TX_TIMEOUT);
	}

	size = snprintf(g.buffer.uart7_tx, LOGGING_BUFFER_SIZE,
					"%.8f;%.8f;%.4f;%.4f;%.4f;\r\n",
				g.joint_result.latitude, g.joint_result.longitude, g.joint_result.velocity_kmh, g.joint_result.velocity_m_s, g.joint_result.bearing);

	if(size > 0)
	{
		HAL_UART_Transmit(&huart7, (uint8_t *)g.buffer.uart7_tx, size, UART_TX_TIMEOUT);
	}

}

