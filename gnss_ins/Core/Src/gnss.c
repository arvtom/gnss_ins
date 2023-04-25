/*
 * l76x.c
 *
 *  Created on: 1 Dec 2021
 *      Author: tomku
 */

#include "gnss.h"

//$GNRMC,185823.40,A,4808.7402374,N,01133.9324760,E,0.00,112.64,130117,3.00,E,A*14

void gnss_parse_GNRMC(uint8_t buf[], uint16_t buf_size)
{
	uint16_t i = 0;
    uint32_t time = 0;
    uint32_t latitude = 0;
    uint32_t longitude = 0;

    while(i < buf_size - 71)
    {
        if(buf[i] == '$' && buf[i+1] == 'G' && buf[i+2] == 'N' &&
        		buf[i+3] == 'R' && buf[i+4] == 'M' && buf[i+5] == 'C')
        {

			time = (uint32_t)strtod(&buf[i+7],0);

			g.gnss.time_h = time/10000;
			g.gnss.time_m = time/100%100;
			g.gnss.time_s = time%100;

			if(g.gnss.time_h >= 24)
			{
				g.gnss.time_h = g.gnss.time_h - 24;
			}

			//A indicates that it has been positioned
			//V indicates that there is no positioning.
			if(buf[i+18] == 'A')
			{
				g.gnss.status = TRUE;
				g.flag.gnss_initiated = TRUE;
			}
			else
			{
				g.gnss.status = FALSE;
			}

			latitude = (buf[i+20]-'0') * 10 + (buf[i+21]-'0');

			float fractional_part_lat = strtod(&buf[i+22],0);

			g.gnss.lat = (float)latitude + fractional_part_lat / 60;

			g.gnss.lat_area = buf[i+30];

			longitude = (buf[i+33]-'0') * 10 + (buf[i+34]-'0');

			float fractional_part_lon = strtod(&buf[i+35],0);

			g.gnss.lon = (float)longitude + fractional_part_lon / 60;

			g.gnss.lon_area = buf[i+43];

			g.gnss.knots = strtod(&buf[i+45],0);
			g.gnss.kmh = 1.852 * g.gnss.knots;
			g.gnss.m_s = g.gnss.kmh / 3.6;

			if(g.gnss.knots >= 10)
			{
				//speed was 5 digits
				g.gnss.bearing = strtod(&buf[i+51],0);
			}
			else
			{
				//speed was 4 digits
				g.gnss.bearing = strtod(&buf[i+50],0);
			}

        }
        i++;
    }
}

void gnss_coordinate_deg_to_rad()
{
	g.gnss.lat_rad = g.gnss.lat * PI / 180;
	g.gnss.lon_rad = g.gnss.lon * PI / 180;
	g.gnss.lat_last_rad = g.gnss.lat_last * PI / 180;
	g.gnss.lon_last_rad = g.gnss.lon_last * PI / 180;
}

void gnss_calculate_bearing(float lat_a, float lon_a, float lat_b, float lon_b)
{
	float lon_delta = lon_a - lon_b;
	if(lon_delta != 0.0f)
	{
		float x = cos(lat_b) * sin(lon_delta);
		float y = cos(lat_a) * sin(lat_b) - sin(lat_a) * cos(lat_b) * cos(lon_delta);

		g.gnss.bearing = atan2(x, y) * 180 / PI; //convert to degrees

		if(g.gnss.bearing < 0)
		{
			g.gnss.bearing = g.gnss.bearing + 360;
		}
	}
}
