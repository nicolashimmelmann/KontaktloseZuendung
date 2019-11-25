#include "ConvertValues.h"
#include <math.h>

/**
 * Calculate Engine RPM based on measured ticks from Camshaft Sensor period.
 */
uint16_t calculateEngineRPM(uint16_t ticks) {
	float engine_rpm = (1.0f / ((float)ticks * 38.0f * 2.631E-6f) * 60.0f) * 2.0f;
	uint16_t int_engine_rpm = (uint16_t)round(engine_rpm);
	return int_engine_rpm;
}


/**
 * Calculate pressure from MAP-Sensor.
 *
 * ADC range is changed by voltage divider (from 3.3V max to 3.905V max).
 * --> 3.905 / 1024 = 3.813mV
 */
uint16_t calculateMAPPressure(uint16_t mapVolts) {
	float volts = mapVolts * 0.003813f + 0.5f;    // 3.913mV per ADC step (0 - 1023)
	float pressure = 0.2534f * volts + 0.0931f;   // Convert to pressure according to datasheet
												  // (Linear relation between voltage and pressure
	return (uint16_t)round(pressure * 1000);  // Pressure in mbar
}
