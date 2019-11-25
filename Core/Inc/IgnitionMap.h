#ifndef INC_IGNITIONMAP_H_
#define INC_IGNITIONMAP_H_

#include <stdint.h>

/**
 * Initializes the min and max values from the engine map.
 */
void IgnitionMap_initialize();

/**
 *
 */
uint16_t IgnitionMap_getFiringAngle(uint16_t x_pressure, uint16_t y_rpm);


#endif /* INC_IGNITIONMAP_H_ */
