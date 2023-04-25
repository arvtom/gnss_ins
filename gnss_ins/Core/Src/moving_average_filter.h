/*
 * moving_average_filter.h
 *
 *  Created on: 3 Dec 2021
 *      Author: tomku
 */

#ifndef SRC_MOVING_AVERAGE_FILTER_H_
#define SRC_MOVING_AVERAGE_FILTER_H_

#include "system_defines.h"

//#include "stdint.h"

//#define MAX_FILTER_SIZE 6000

//struct f_float
//{
//	float value;
//	float moving_sum;
//	uint16_t window_index;
//	float buf[MAX_FILTER_SIZE];
//	uint16_t buf_size;
//	uint16_t filter_size;
//};

//typedef struct
//{
//	float value;
//	float moving_sum;
//	uint16_t window_index;
//	float buf[MAX_FILTER_SIZE];
//	uint16_t buf_size;
//	uint16_t filter_size;
//}filter_t;

void moving_average_filter_update_float(filter_t *s, float input);
void moving_average_filter_reset(filter_t *s, uint16_t filter_size);

#endif /* SRC_MOVING_AVERAGE_FILTER_H_ */
