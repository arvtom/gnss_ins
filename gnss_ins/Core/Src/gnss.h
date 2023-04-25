/*
 * l76x.h
 *
 *  Created on: 1 Dec 2021
 *      Author: tomku
 */

#ifndef SRC_GNSS_H_
#define SRC_GNSS_H_

#include "system_defines.h"

void gnss_parse_GNRMC(uint8_t buf[], uint16_t buf_size);
void gnss_coordinate_deg_to_rad();
void gnss_calculate_bearing(float lat_a, float lon_a, float lat_b, float lon_b);

#endif /* SRC_GNSS_H_ */
