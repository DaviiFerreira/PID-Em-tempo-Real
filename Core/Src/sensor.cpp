/*
 * sensor.cpp
 *
 *  Created on: Jun 22, 2025
 *      Author: davim
 */

#include "sensor.h"

#include "qassert.h"
I2C_HandleTypeDef hi2c1; // Definição do handler

void sensorInit(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. Ativar clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    // 2. Configurar PA15 (SCL)
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 3. Configurar PB7 (SDA)
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 4. Configurar I2C1
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00707CBB;  // 400 kHz
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        // Tratamento de erro - ajuste conforme sua aplicação
		while(1){

		}
    }
}


HAL_StatusTypeDef sensorReadDistance(I2C_HandleTypeDef *hi2c, uint16_t *distance) {
    uint8_t reg = 0x1E;  // Registro da distância
    uint8_t data[2];
    HAL_StatusTypeDef status;

    // Lê os 2 bytes de distância
    status = HAL_I2C_Master_Transmit(hi2c, VL53L0X_DEFAULT_ADDR, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    status = HAL_I2C_Master_Receive(hi2c, VL53L0X_DEFAULT_ADDR, data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    *distance = (data[0] << 8) | data[1];
    return HAL_OK;
}
