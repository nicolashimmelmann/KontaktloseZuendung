#ifndef SRC_CONVERTVALUES_H_
#define SRC_CONVERTVALUES_H_

#include <stdint.h>

/**
 * Calculate Engine RPM based on measured ticks from Camshaft Sensor period.
 */
uint16_t calculateEngineRPM(uint16_t ticks);

/**
 * Calculate pressure from MAP-Sensor
 */
uint16_t calculateMAPPressure(uint16_t mapVolts);


#endif /* SRC_CONVERTVALUES_H_ */
