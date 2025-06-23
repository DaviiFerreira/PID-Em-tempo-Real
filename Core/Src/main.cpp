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
extern "C" {
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_rcc.h"
#include "semaforo.h"

#include "qassert.h"
#include "stm32g4xx_hal.h"  // Biblioteca da HAL para STM32G4
#include "stm32g4xx_hal_rng.h"  // Biblioteca específica do RNG
}
#include "sensor.h"
#include "stm32g4xx_hal_conf.h"
#include "ventilador.h"
/*teste botao*/
volatile int but = 0;  // variável global que será modificada na interrupção

#include "miros.h"
int32_t a = 0;
int32_t pilha[5] = {-1, -1, -1, -1, -1};
int32_t topo = -1;
rtos :: MySemaphore mutex;
RNG_HandleTypeDef hrng;  // Definição da variável global

void RNG_Init() {
    __HAL_RCC_RNG_CLK_ENABLE(); // Liga o clock do RNG
    hrng.Instance = RNG;
    HAL_RNG_Init(&hrng);
}



void avG_Init() {
    __HAL_RCC_RNG_CLK_ENABLE(); // Liga o clock do RNG
    hrng.Instance = RNG;
    HAL_RNG_Init(&hrng);
}

uint32_t get_random_number() {
    uint32_t random_value;
    HAL_RNG_GenerateRandomNumber(&hrng, &random_value);
    return ((random_value % 94) + 33); // Caracteres de 33 a 126
}
/*
	bool push(){
		if(mutex.tryLock()){
			if(topo<4){
				topo++;
				pilha[topo] = get_random_number();
				mutex.tryUnlock();
				return true;
			}
			mutex.tryUnlock();
		}
		return false;

	}
	int32_t pop(){
			if(mutex.tryLock()){
				if(topo>=0){
					uint32_t value = pilha[topo];
					pilha[topo] = -1; // Limpando valor retirado

					topo--;
					mutex.tryUnlock();
					return value;
				}
				mutex.tryUnlock();
			}
			return -1;

		}
*/


uint32_t stack_idleThread[40];
uint32_t stack_ProducerThread[50];
uint32_t stack_ConsumerThread[50];

uint32_t stack_TaskCThread[50];

void inProduce(){
	a=2;
	return;
	}
void outProduce(){
	a=4;
	return;
	}
void inConsume(){
	a=3;
	return;
	}
void outConsume(){
	a=5;
	return;
	}
void inTc(){
	a=10;
	return;
	}
void outTc(){
	a=20;
	return;
	}
void produce(){
 		inProduce();
	//push();
 		for(uint16_t i = 0; i<10; i++){}

	outProduce();

	}
void taskC(){
	inTc();
	for(uint16_t i = 0; i<30; i++){}
	outTc();

}

void consume(){
	/*
	 * Se sizePass fosse declarado dentro da função sem static, sim, ele reiniciaria para 0 toda vez que consume() fosse chamado.

Mas quando a variável é static dentro da função, ela mantém seu valor entre chamadas! Ou seja:

Na primeira chamada, sizePass = 0.

Na segunda chamada, ele continua com o valor que estava na última execução.

Ele só é reinicializado se o microcontrolador for reiniciado.*/



/*	static char password[64];
	static uint32_t sizePass = 0;
	int32_t retorno = pop();



	 * Em microcontroladores ARM Cortex-M (como o STM32), operações em variáveis de 32 bits
	 * são atômicas se alinhadas corretamente (o compilador cuida disso).

Ou seja, uma escrita ou leitura em sizePass não pode ser interrompida pela preempção.*/


	/*if(sizePass >= 64 ){
			(void) password; // Apenas para evitar o warning
			sizePass = 0;

		}
	if(retorno != -1 && sizePass<64){
		password[sizePass]= static_cast<char>(retorno);
		sizePass++;

	}*/
	inConsume();
		for(uint16_t i = 0; i<20; i++){}
	outConsume();


}
rtos::OSPeriodicTask PeriodicA;
rtos::OSPeriodicTask PeriodicB;
rtos::OSPeriodicTask PeriodicC;



uint8_t D = 0;
void inTd(){
D=1;
}
void taskD(){
inTd();
D=3;
}
uint8_t E = 0;
void inTe(){
E=1;
}
void taskE(){
inTe();
E=3;
}
rtos :: OSAperiodicTask e;
rtos :: OSAperiodicTask d;

uint8_t ABC = 0;



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13)
  {
	  rtos :: OSAperiodicTask_start(&d, &taskD);
  }
}



// Main
/*int main(void) {
    HAL_Init();
    rtos :: OS_onStartup() ;// Configure o clock do sistema
    I2C1_Manual_Init();    // Inicializa o I2C1 com PA15/PB7

    while (1) {
        uint16_t distance = VL53L0X_ReadDistance();
        HAL_Delay(100);    // Espera 100ms entre medições
    }
}*/
int main(void)
{
	 HAL_Init();
	// SystemClock_Config();
	    sensorInit();          // Inicializa I2C

	    uint16_t distance;
	    HAL_StatusTypeDef status;

	  __HAL_RCC_GPIOC_CLK_ENABLE(); // Clock para PC13 (botão)
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

	 avG_Init();

	 status = sensorReadDistance(&hi2c1, &distance);

	        if (status == HAL_OK) {
	            // Usa o valor da distância
	            // Ex: enviar por UART, acionar algo, etc.
	        } else {
	            // Tratar erro
	        }
	  rtos::OS_init(stack_idleThread, sizeof(stack_idleThread));


	  rtos::OSPeriodicTask_start(&PeriodicA,
	                 &produce,
					 stack_ProducerThread, sizeof(stack_ProducerThread),200u);

	  rtos:: OSPeriodicTask_start(&PeriodicB,
	                 &consume,
					 stack_ConsumerThread, sizeof(stack_ConsumerThread),400u);


	  /*rtos:: OSPeriodicTask_start(&PeriodicC,
	                 &taskC,
					 stack_TaskCThread, sizeof(stack_TaskCThread),750u);*/
	  rtos :: AperiodicServerStart();

	  /* transfer control to the RTOS to run the threads */
	  rtos::OS_run();
}


