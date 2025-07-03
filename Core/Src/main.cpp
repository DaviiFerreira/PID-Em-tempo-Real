/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "main.h"
#include <cstdint>
extern "C"
{
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_rcc.h"
#include "semaforo.h"

#include "qassert.h"
#include "stm32g4xx_hal.h"     // Biblioteca da HAL para STM32G4
#include "stm32g4xx_hal_rng.h" // Biblioteca específica do RNG
}
// #include "sensor.h"
#include "stm32g4xx_hal_conf.h"
#include "ventilador.h"
#include "core_cm4.h" // traz as definições de SCB e FPU
#include "miros.h"
/*teste botao*/

rtos :: MySemaphore mutex;
#define VL53L0X_ADDR (0x52) // 7-bit left-shifted

I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
#define VL53L0X_TIMEOUT_MS 1000

uint32_t stack_idleThread[40];
uint32_t stack_LerSensor[128];
uint32_t stack_CalculoPid[128];
uint32_t stack_SetaVelocidade[400];

rtos ::OSPeriodicTask threadLerSensor;
rtos ::OSPeriodicTask threadCalculoPid;
rtos ::OSPeriodicTask threadSetaVelocidade;
// Endereço do VL53L0X

int32_t E = -1;
double ultimoErro = 0;
double erroIntegral = 0;

double testePid(double medida, double setpoint)
{
  double erro = setpoint - medida;

  double proporcional = -0.0001 * erro;
  erroIntegral += erro * 0.050;
  double integral = -0.00001 * erroIntegral;
  double derivativo = -0.00001 * (erro - ultimoErro) / 0.050;

  ultimoErro = erro;
  if (proporcional + integral + derivativo < -0.3)
  {
    return -30;
  }
  if (proporcional + integral + derivativo > 0.3)
  {
    return 30;
  }

  return (proporcional + integral + derivativo) * 100;
}
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

HAL_StatusTypeDef VL53L0X_InitSimple(void)
{
  uint8_t cmd = 0x01;
  HAL_StatusTypeDef ret;
  uint8_t ready;
  // 1) Iniciar medição única (escreve 0x01 em 0x00)
  ret = HAL_I2C_Mem_Write(&hi2c1, VL53L0X_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);
  if (ret != HAL_OK)
    return ret;

  do
  {
    ret = HAL_I2C_Mem_Read(&hi2c1, VL53L0X_ADDR, 0xC0, I2C_MEMADD_SIZE_8BIT, &ready, 1, 100);

  } while ((ready != 0xEE));
  return ret;
}

HAL_StatusTypeDef VL53L0X_ReadSingleSimple(uint16_t *distance)
{
  HAL_StatusTypeDef ret;
  uint8_t cmd = 0x01;
  uint8_t rangeData[2];

  uint8_t ready;

  ret = HAL_I2C_Mem_Write(&hi2c1, VL53L0X_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);
  do
  {
    ret = HAL_I2C_Mem_Read(&hi2c1, VL53L0X_ADDR, 0xC0, I2C_MEMADD_SIZE_8BIT, &ready, 1, 100);

  } while ((ready != 0xEE));

  // 3) Ler distância: 0x1E + 0x1F,,
  ret = HAL_I2C_Mem_Read(&hi2c1, VL53L0X_ADDR, 0x1E, I2C_MEMADD_SIZE_8BIT, rangeData, 2, 100);

  *distance = (rangeData[0] << 8) | rangeData[1];
  return ret;
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* --- Habilita clocks GPIOA e GPIOB --- */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PB7 -> SDA (I2C1_SDA) */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PA15 -> SCL (I2C1_SCL) */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  /* Mode, Pull, Speed e Alternate permanecem iguais */
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}
static void MX_I2C1_Init(void)
{

  __HAL_RCC_I2C1_CLK_ENABLE();

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10C0ECFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

uint16_t setpointGlobal = 0;
double resultPid;
uint16_t distance = 0;
void LerSensor()
{
  //E = distance;
	if(mutex.tryLock()){

   VL53L0X_ReadSingleSimple( &distance);
		mutex.tryUnlock();
	}
}
void SetaVelocidade()
{
  //float e = distance / 100;
  // distance = 6.00;

	if(mutex.tryLock()){

  ventiladorSetDutyCycle(resultPid + 61);
	mutex.tryUnlock();
	}
  // Usa o valor da distância
  // Ex: enviar por UART, acionar algo, etc.
}
void CalculoPid()
{

 if(mutex.tryLock()){
	  uint16_t medida = distance ; // 50 correção leitura do sensor
	 if(distance<20){
		 medida = 20;
	 }
	 if(distance>1000){
	 		 medida = 1000;
	 	 }
	  double erro = setpointGlobal - medida;

	  double proporcional = -0.0001 * erro;
	  erroIntegral += erro * 0.050;
	  double integral = -0.00001 * erroIntegral;
	  double derivativo = -0.00001 * (erro - ultimoErro) / 0.050;

	  ultimoErro = erro;
	  if (proporcional + integral + derivativo < -0.3)
	  {
	    resultPid = -30;

		  mutex.tryUnlock();
	    return;
	  }
	  if (proporcional + integral + derivativo > 0.3)
	  {
	    resultPid = 30;

		  mutex.tryUnlock();
	    return;
	  }

	  resultPid = (proporcional + integral + derivativo) * 100;
	  mutex.tryUnlock();
 }

  // resultPid = testePid(distance-50,setpointGlobal)+61;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	while(!mutex.tryLock()){}

	if (setpointGlobal == 300){
		  setpointGlobal = 200;
	}
	else{
		setpointGlobal = 300;
	}
	mutex.tryUnlock();

}

int main(void)
{
  setpointGlobal = 300;
  SCB->CPACR |= (0xF << 20); // habilita acesso FPU

  // no começo do main(), logo após habilitar CPACR:
  FPU->FPCCR &= ~((1U << 30) | (1U << 31)); // desliga ASPEN e LSPEN

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();

  // Clock para PC13 (botão)
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  // Inicialização do botão PC13 como EXTI
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Habilita interrupção no NVIC
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  // avG_Init();

  // rtos :: OS_onStartup();
  ventiladorInit();
  ventiladorSetDutyCycle(60.00);

  VL53L0X_InitSimple();

  rtos::OS_init(stack_idleThread, sizeof(stack_idleThread));

  rtos::OSPeriodicTask_start(&threadLerSensor,
                             &LerSensor,
                             stack_LerSensor, sizeof(stack_LerSensor), 50u);

  rtos::OSPeriodicTask_start(&threadCalculoPid,
                             &CalculoPid,
                             stack_CalculoPid, sizeof(stack_CalculoPid), 50u);

  rtos::OSPeriodicTask_start(&threadSetaVelocidade,
                             &SetaVelocidade,
                             stack_SetaVelocidade, sizeof(stack_SetaVelocidade), 50u);
  rtos::OS_run();
}
