/*
 * ventilador.h
 *
 *  Created on: Jun 22, 2025
 *      Author: davim
 */

#ifndef VENTILADOR_H
#define VENTILADOR_H


#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_tim.h"
void ventiladorInit(void);
void ventiladorSetDutyCycle(float duty_cycle_percent);






#endif /* INC_VENTILADOR_H_ */
