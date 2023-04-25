/*
 * system_defines.h
 *
 *  Created on: 4 Dec 2021
 *      Author: tomku
 */

#ifndef SRC_SYSTEM_DEFINES_H_
#define SRC_SYSTEM_DEFINES_H_

#include "math.h"
#include "stdint.h"
#include "usart.h"
#include "stdlib.h"
#include "string.h"

//#define GNSS_TEST 1

#define ACC_SAMPLE_PERIOD 0.001

#define UART_TX_TIMEOUT 10
#define UART4_RX_SIZE 1000
#define UART4_TX_SIZE 19
#define SET_NMEA_BAUDRATE_115200 "$PMTK251,115200"

#define FILTER_SIZE 100
#define FILTER_SIZE_CALIBRATION 300
#define MAX_FILTER_SIZE FILTER_SIZE_CALIBRATION

#define LOGGING_BUFFER_SIZE 200

#define BUFFSIZE 1418

#define PI 3.14159

#define TRUE 1u
#define FALSE 0u

#define UNIT 1

#ifdef UNIT
	#define UNIT_SAMPLES 500
#endif


extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern I2C_HandleTypeDef hi2c3;
extern TIM_HandleTypeDef htim3;

typedef struct
{
	uint8_t uart4_rx_interrupt :1;
	uint8_t acc_ready :1;
	uint8_t acc_calibrated :1;
	uint8_t gnss_initiated :1;
	uint8_t tim3_interrupt :1;
	uint8_t tim4_interrupt :1;
	uint8_t gnss_active :1;
	uint8_t ins_begin :1;
//	uint8_t gnss_lock :1;
}flag_t;

typedef struct
{
//	float latitude;
//	float longitude;
//	float bearing;
	float velocity_m_s;
	float velocity_km_h;
	float acceleration_2d;
	float acceleration_3d;
	float velocity_initial;
//	float distance;
}ins_t;

typedef struct {
	float lon;     //gnss Latitude and longitude
	float lat;
	float lon_rad;
	float lat_rad;
	float lon_last;
	float lat_last;
	float lon_last_rad;
	float lat_last_rad;
	float bearing;
    uint8_t lon_area;
    uint8_t lat_area;
    uint8_t time_h;   //Time
    uint8_t time_m;
    uint8_t time_s;
    uint8_t status;   //1:Successful positioning 0:Positioning failed
    double knots;
    double kmh;
    double m_s;
    uint16_t no_packet_counter;
    uint8_t last_timestamp;
}gnss_t;

typedef struct
{
	float latitude;
	float longitude;
	float bearing;
	float velocity_m_s;
	float velocity_kmh;
	float distance;
}joint_result_t;

typedef struct
{
	int16_t raw;
	float value;
	float offset;
	float value_filtered;
}acc_t;

typedef struct
{
	int16_t raw;
	float value;
	float offset;
	float value_filtered;
}gyro_t;

typedef struct
{
	float value;
	float moving_sum;
	uint16_t window_index;
	float buf[MAX_FILTER_SIZE];
	uint16_t buf_size;
	uint16_t filter_size;
}filter_t;

typedef struct
{
	uint8_t acc_read;
	uint8_t gnss_parse;
	uint8_t test_ins;
	uint8_t uart4_error;
//	uint8_t gnss;
	uint8_t pps;
	uint8_t tim3;
	uint8_t tim4;
}counter_t;

typedef struct
{
	uint8_t uart4_rx[UART4_RX_SIZE];
	uint8_t uart4_tx[UART4_TX_SIZE];

	uint8_t uart7_tx[LOGGING_BUFFER_SIZE];

	uint8_t i2c3_rx[10];
	uint8_t i2c3_tx[10];
}buffer_t;

typedef struct
{
	joint_result_t unit[500];

	buffer_t buffer;

	counter_t counter;

	flag_t flag;

	ins_t ins;
	gnss_t gnss;
	joint_result_t joint_result;

	acc_t acc_x;
	acc_t acc_y;
	acc_t acc_z;

	gyro_t gyro_x;
	gyro_t gyro_y;
	gyro_t gyro_z;

	filter_t filter_acc_x;
	filter_t filter_acc_y;
	filter_t filter_acc_z;

	filter_t filter_gyro_x;
	filter_t filter_gyro_y;
	filter_t filter_gyro_z;
}g_t;

g_t g; //global system structure

#endif /* SRC_SYSTEM_DEFINES_H_ */
