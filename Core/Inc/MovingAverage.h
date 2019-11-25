#ifndef INC_MOVINGAVERAGE_H_
#define INC_MOVINGAVERAGE_H_

#include <stdint.h>

void MovAvg_reset(int max_values);
void MovAvg_update(uint32_t value);
uint32_t MovAvg_getMean();



#endif /* INC_MOVINGAVERAGE_H_ */
