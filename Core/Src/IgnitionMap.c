#include <IgnitionMap.h>
#include <math.h>


#define COLS  16  // Pressure (x-axis)
#define ROWS  20  // RPM (y-axis)


/* Sampling points of pressure */
//const uint16_t X[COLS] = {25, 50, 75, 100, 125, 150, 175, 200, 225, 250, 275, 300, 325, 350, 375, 400};
const uint16_t X[COLS] = {447, 477, 506, 535, 565, 594, 624, 653, 683, 712, 742, 771, 801, 830, 860, 889};
/* Sampling points of RPM */
const uint16_t Y[ROWS] = {250, 500, 750, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000, 3250, 3500, 3750, 4000, 4250, 4500, 4750, 5000};

/* Actual ignition map */
const uint16_t ignition_map[ROWS][COLS] =
{
	{175, 175, 175, 175, 175, 175, 170, 155, 135, 120, 105, 85, 75, 75, 75, 75},
	{175, 175, 175, 175, 175, 175, 170, 155, 135, 120, 105, 85, 75, 75, 75, 75},
	{175, 175, 175, 175, 175, 175, 170, 155, 135, 120, 105, 85, 75, 75, 75, 75},
	{175, 175, 175, 175, 175, 175, 170, 155, 135, 120, 105, 85, 75, 75, 75, 75},
	{200, 200, 200, 200, 200, 200, 195, 180, 160, 145, 130, 110, 100, 100, 100, 100},
	{220, 220, 220, 220, 220, 220, 215, 200, 180, 165, 150, 130, 120, 120, 120, 120},
	{220, 220, 220, 220, 220, 220, 215, 200, 180, 165, 150, 130, 120, 120, 120, 120},
	{220, 220, 220, 220, 220, 220, 215, 200, 180, 165, 150, 130, 120, 120, 120, 120},
	{230, 230, 230, 230, 230, 230, 225, 210, 190, 175, 160, 140, 130, 130, 130, 130},
	{260, 260, 260, 260, 260, 260, 255, 240, 220, 205, 190, 170, 160, 160, 160, 160},
	{280, 280, 280, 280, 280, 280, 275, 260, 240, 225, 210, 190, 180, 180, 180, 180},
	{300, 300, 300, 300, 300, 300, 295, 280, 260, 245, 230, 210, 200, 200, 200, 200},
	{330, 330, 330, 330, 330, 330, 325, 310, 290, 275, 260, 240, 230, 230, 230, 230},
	{330, 330, 330, 330, 330, 330, 325, 310, 290, 275, 260, 240, 230, 230, 230, 230},
	{330, 330, 330, 330, 330, 330, 325, 310, 290, 275, 260, 240, 230, 230, 230, 230},
	{330, 330, 330, 330, 330, 330, 325, 310, 290, 275, 260, 240, 230, 230, 230, 230},
	{330, 330, 330, 330, 330, 330, 325, 310, 290, 275, 260, 240, 230, 230, 230, 230},
	{330, 330, 330, 330, 330, 330, 325, 310, 290, 275, 260, 240, 230, 230, 230, 230},
	{330, 330, 330, 330, 330, 330, 325, 310, 290, 275, 260, 240, 230, 230, 230, 230},
	{330, 330, 330, 330, 330, 330, 325, 310, 290, 275, 260, 240, 230, 230, 230, 230}
};


/* Minimum and maximum values of the sampling points */
float x_min;
float y_min;
float x_max;
float y_max;


void IgnitionMap_initialize() {
	x_min = X[0];
	y_min = Y[0];
	x_max = X[COLS-1];
	y_max = Y[ROWS-1];
}

uint16_t IgnitionMap_getFiringAngle(uint16_t x_pressure, uint16_t y_rpm) {
	/* Limit pressure (in mv) to x_min < x_pressure < x_max */
	if(x_pressure > x_max)
	{
		x_pressure = x_max;
	}
	else if(x_pressure < x_min)
	{
		x_pressure = x_min;
	}

	/* Limit RPM to y_min < y_rpm < y_max */
	if(y_rpm > y_max)
	{
		y_rpm = y_max;
	}
	else if(y_rpm < y_min)
	{
		y_rpm = y_min;
	}

	/* Do interpolation*/

	/* Row and column indices */
	uint8_t row_idx, col_idx;

	float estimated = 0;
	float fractional_col = 0, fractional_row = 0;

	/* Find integer and fractional part of column index */
	fractional_col = (COLS - 1) * (x_pressure - x_min) / (x_max - x_min);
	col_idx = (uint8_t)fractional_col;
	fractional_col = fractional_col - col_idx;

	/* Find integer and fractional part of row index */
	fractional_row = (ROWS-1) * (y_rpm - y_min) / (y_max - y_min);
	row_idx = (uint8_t)fractional_row;
	fractional_row = fractional_row - row_idx;

	/* Calculate interpolated estimate */
	estimated = (1-fractional_col) * (1-fractional_row) * ignition_map[row_idx][col_idx]
                 + fractional_col * (1-fractional_row) * ignition_map[row_idx][col_idx+1]
				 + (1-fractional_col) * fractional_row * ignition_map[row_idx+1][col_idx]
				 + fractional_col * fractional_row * ignition_map[row_idx+1][col_idx+1];

	/* Rounded to 1 decimal place */
	return (uint16_t)round(estimated);
}

