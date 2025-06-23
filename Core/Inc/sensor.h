/*
 * sensor.h
 *
 *  Created on: Jun 22, 2025
 *      Author: davim
 */

#ifndef SENSOR_H
#define SENSOR_H

#include "stm32g4xx_hal.h"

#include "stm32g4xx_hal_i2c.h"

#define VL53L0X_DEFAULT_ADDR (0x52 << 1) // Endereço padrão do sensor
extern I2C_HandleTypeDef hi2c1; // Handler externo

void sensorInit(void);


// Lê a distância em mm
HAL_StatusTypeDef sensorReadDistance(I2C_HandleTypeDef *hi2c, uint16_t *distance);


#endif /* INC_SENSOR_H_ */
