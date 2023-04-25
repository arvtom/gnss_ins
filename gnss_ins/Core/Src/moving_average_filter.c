/*
 * moving_average_filter.c
 *
 *  Created on: 3 Dec 2021
 *      Author: tomku
 */

#include "moving_average_filter.h"
//#include <string.h>


void moving_average_filter_update_float(filter_t *s, float input)
{
	s->moving_sum = s->moving_sum - s->buf[s->window_index] + input;
	s->buf[s->window_index] = input;
	s->window_index++;
	if (s->window_index > s->filter_size - 1)
	{
		s->window_index = 0;
		s->buf_size = s->filter_size;
	}
	if (s->buf_size == s->filter_size)
	{
		s->value = s->moving_sum / s->filter_size;
	}
}

void moving_average_filter_reset(filter_t *s, uint16_t filter_size)
{
	s->moving_sum = 0;
	s->value = 0;
	s->filter_size = filter_size;
	memset(&(s->buf), 0x00, filter_size);
//	memset(&(s->buf), 0x00, MAX_FILTER_SIZE-1);
}
