#ifndef _BMP180_H_
#define _BMP180_H_


#include "stm32f4xx_hal.h"

void BMP180_Start (void);

float BMP180_GetTemp (void);

#endif
