/****************************************************************************
* MInimal Real-time Operating System (MiROS), GNU-ARM port.
*
* This software is a teaching aid to illustrate the concepts underlying
* a Real-Time Operating System (RTOS). The main goal of the software is
* simplicity and clear presentation of the concepts, but without dealing
* with various corner cases, portability, or error handling. For these
* reasons, the software is generally NOT intended or recommended for use
* in commercial applications.
*
* Copyright (C) 2018 Miro Samek. All Rights Reserved.
*
* SPDX-License-Identifier: GPL-3.0-or-later
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <https://www.gnu.org/licenses/>.
*
* Git repo:
* https://github.com/QuantumLeaps/MiROS
****************************************************************************/
#include <cstdint>
#include "miros.h"
#include "qassert.h"
#include "stm32g4xx.h"

#include "semaforo.h"
#include <limits>

Q_DEFINE_THIS_FILE

namespace rtos {

OSThread * volatile OS_curr; /* pointer to the current thread */
OSThread * volatile OS_next; /* pointer to the next thread to run */

OSPeriodicTask * volatile OSPeriodic_curr; /* pointer to the current thread */
OSPeriodicTask * volatile OSPeriodic_next; /* pointer to the next thread to run */

OSPeriodicTask *OSPeriodicTasks[32 + 1]; /* array of PeriodicTask started so far */
uint8_t OS_periodicTaskNum=0; /* number of periodic PeriodicTask started */


OSAperiodicTask *OSAperiodicTasks[32 + 1]; /* array of threads started so far */
uint8_t OS_AperiodicTaskNum = 0;



uint32_t TempoCiclo=0;
uint32_t TempoAtual=0;

OSThread *OS_thread[32 + 1]; /* array of threads started so far */
uint32_t OS_readySet; /* bitmask of threads that are ready to run */


uint8_t OS_threadNum; /* number of threads started */
uint8_t OS_currIdx; /* current thread index for the circular array */


bool AperiodicServerStarted = false;

rtos :: MySemaphore preemptionAllowed;


void desativarPreempcao(){
	preemptionAllowed.tryLock();
}
void reativarPreempcao(){
	preemptionAllowed.tryUnlock();
}

// Função para calcular o Máximo Divisor Comum (GCD)
   uint32_t gcd(uint32_t a, uint32_t b) {
      while (b != 0) {
          uint32_t temp = b;
          b = a % b;
          a = temp;
      }
      return a;
  }

  // Função para calcular o Mínimo Múltiplo Comum (LCM)
   uint32_t lcm(uint32_t a, uint32_t b) {
      return (a / gcd(a, b)) * b;
  }

OSThread idleThread;

void AperiodicServerStart(){
	AperiodicServerStarted = true;
}
void AperiodicServerStop(){
	AperiodicServerStarted = false;
}
void main_idleThread() {
    while (1) {
    	while(OS_AperiodicTaskNum>0 && AperiodicServerStarted)
    	{
    		osAperiodicWrapper();
    	}
        OS_onIdle();
    }
}



void OS_init(void *stkSto, uint32_t stkSize) {
    /* set the PendSV interrupt priority to the lowest level 0xFF */
    *(uint32_t volatile *)0xE000ED20 |= (0xFFU << 16);

    /* start idleThread thread */
    OSThread_start(&idleThread,
                   &main_idleThread,
                   stkSto, stkSize);
}

void osAperiodicWrapper() {
		OSAperiodicTasks[0]->myTask();
		for(uint8_t i =0; i<OS_AperiodicTaskNum-1; i++){
			OSAperiodicTasks[i] = OSAperiodicTasks[i+1];
			 OSAperiodicTasks[i+1] = 0x0;
		}
		OS_AperiodicTaskNum--;
}


void OSAperiodicTask_start(OSAperiodicTask *me,
    OSThreadHandler threadHandler){
	__disable_irq();
	me->myTask = threadHandler;
	OSAperiodicTasks[OS_AperiodicTaskNum] = me;
	OS_AperiodicTaskNum++;
	__enable_irq();
}




void OS_sched(void) {

	 uint32_t minPeriod = 0xFFFFFFFFU;

	 uint8_t periodicIndex= 0u;

	 if (OS_readySet == 0U) {
	        OS_currIdx = 0U; // idle
	 }

	 else {
		for (uint8_t n = 0; n < OS_periodicTaskNum; n++) {
			// Mapear OSPeriodicTasks[n] para OS_thread[n + 1]
			uint8_t threadIdx = OSPeriodicTasks[n]->myThreadIndex;
			if ((OS_readySet & (1U << (threadIdx - 1U)))) {
				OSPeriodicTask *pt = OSPeriodicTasks[n]; // Usar índice correto
				if (pt->Period < minPeriod) {
					minPeriod = pt->Period;
					OS_currIdx = threadIdx;
					periodicIndex = n;
				}
			}
		}
	}

	 if(OS_currIdx!=0){
		OSPeriodic_curr =  OSPeriodicTasks[periodicIndex];
	 }

     OS_next = OS_thread[OS_currIdx];

    /* trigger PendSV, if needed */
    if(OS_next != OS_curr && preemptionAllowed.isAvailable() ){
    	*(uint32_t volatile *)0xE000ED04 = (1U << 28);
    }

}

void OS_run(void) {
    /* callback to configure and start interrupts */
    OS_onStartup();

    __disable_irq();
    OS_sched();
    __enable_irq();

    /* the following code should never execute */
    Q_ERROR();
}


void checkDeadline(uint8_t n){
	  OSPeriodicTask *pt = OSPeriodicTasks[n];
	        uint32_t deadline = pt->lastAtivation + pt->Period;
	        if (deadline <= TempoCiclo) {
				if (TempoAtual >= deadline) {
					if (TempoAtual > deadline) {// && (TempoAtual != TempoCiclo && pt->lastAtivation  != 0)
						Q_ERROR(); // Deadline miss

					}
					else{
						if((OS_readySet & (1U << (pt->myThreadIndex - 1U)))){
							Q_ERROR(); // Deadline miss
						}

						//marca tarefa como pronta
						OS_readySet |= (1U <<  (pt->myThreadIndex - 1U));

							 pt->lastAtivation = TempoAtual;

					}
				}
			}
	                // Lógica para deadlines no próximo ciclo
			else {
				uint32_t adjustedDeadline = deadline - TempoCiclo;
				if (TempoAtual >= adjustedDeadline) {
					if (TempoAtual > adjustedDeadline) {
						Q_ERROR(); // Deadline miss

					}
					else{
						if((OS_readySet & (1U << ( pt->myThreadIndex - 1U)))){
							Q_ERROR(); // Deadline miss

						}
						OS_readySet |= (1U << ( pt->myThreadIndex - 1U));
						pt->lastAtivation = TempoAtual;
					}
				}
			}
}

void OS_tick(void) {
	uint8_t n = 0;
	TempoAtual++;
	if(TempoAtual>TempoCiclo){
		TempoAtual = 0;
	}
	for(n=1U;n<OS_threadNum; n++){ 				/* cycle through every thread but the idle */
		if(OS_thread[n]->timeout != 0U){
			OS_thread[n]->timeout--;			/* decrease the timeout */
			if(OS_thread[n]->timeout == 0U){
				OS_readySet |= (1U << (n-1U));	/* if the thread is ready mask the corresponding bit */
			}
		}
	}
	if (TempoCiclo == 0) return;
    for (uint8_t n = 0; n < OS_periodicTaskNum; n++) {
    	checkDeadline(n);
	}
 }



void OS_delay(uint32_t ticks) {
    __asm volatile ("cpsid i");

    /* never call OS_delay from the idleThread */
    Q_REQUIRE(OS_curr != OS_thread[0]);

    OS_curr->timeout = ticks;
    OS_readySet &= ~(1U << (OS_currIdx - 1U));
    OS_sched();
    __asm volatile ("cpsie i");
 }


void osPeriodicWrapper() {
	while (1) {
		if (OSPeriodic_curr && OSPeriodic_curr->myTask
				&& (OS_readySet & (1U << ((OSPeriodic_curr->myThreadIndex) - 1U)))) {
			OSPeriodicTask *me = OSPeriodic_curr;
			me->myTask();


			//essa parte deve ser atomica?? esta certo falar atomica??
			uint32_t deadline = me->lastAtivation + me->Period;
			if (deadline <= TempoCiclo) {
				if (me->lastAtivation != TempoAtual && TempoAtual > deadline) { // || (TempoAtual == TempoCiclo && me->lastAtivation  != 0)
					Q_ERROR(); // Deadline miss

				}
			}
			// Lógica para deadlines no próximo ciclo
			else {

				uint32_t adjustedDeadline = deadline - TempoCiclo;
				if (me->lastAtivation != TempoAtual && TempoAtual > adjustedDeadline  ) {
					Q_ERROR(); // Deadline miss

				}
			}
			OS_readySet &= ~(1U << (OSPeriodic_curr->myThreadIndex - 1U));

			// se fosse atomica é até aqui???


		}
		__disable_irq();
		OS_sched();
		__enable_irq();

	}
}
void OSPeriodicTask_start(OSPeriodicTask *me,
    OSThreadHandler threadHandler,
    void *stkSto, uint32_t stkSize, uint32_t period){
	  Q_REQUIRE(period != 0);

	__disable_irq();

	me->myTask = threadHandler;

	me->Period = period;

	me->myThreadIndex = OSThread_start(&(me->my_Thread), osPeriodicWrapper, stkSto, stkSize);

	me->myPeriodicTaskIndex = OS_periodicTaskNum;

	OSPeriodicTasks[OS_periodicTaskNum] = me;

	OS_periodicTaskNum++;

	me->lastAtivation = TempoAtual;

	if (TempoCiclo != 0) {
		uint32_t new_lcm = lcm(TempoCiclo, period);
		if (new_lcm == 0 ) { // Verificação correta
			Q_ERROR();
		}
		TempoCiclo = new_lcm;
	} else {
		TempoCiclo = period;
	}

  __enable_irq();
}

uint8_t OSThread_start(
    OSThread *me,
    OSThreadHandler threadHandler,
    void *stkSto, uint32_t stkSize)
{
    /* round down the stack top to the 8-byte boundary
    * NOTE: ARM Cortex-M stack grows down from hi -> low memory
    */
    uint32_t *sp = (uint32_t *)((((uint32_t)stkSto + stkSize) / 8) * 8);
    uint32_t *stk_limit;

    /* thread number must be in ragne
    * and must be unused
    */
    Q_REQUIRE((OS_threadNum < Q_DIM(OS_thread)) && (OS_thread[OS_threadNum] == (OSThread *)0));

    *(--sp) = (1U << 24);  /* xPSR */
    *(--sp) = (uint32_t)threadHandler; /* PC */
    *(--sp) = 0x0000000EU; /* LR  */
    *(--sp) = 0x0000000CU; /* R12 */
    *(--sp) = 0x00000003U; /* R3  */
    *(--sp) = 0x00000002U; /* R2  */
    *(--sp) = 0x00000001U; /* R1  */
    *(--sp) = 0x00000000U; /* R0  */
    /* additionally, fake registers R4-R11 */
    *(--sp) = 0x0000000BU; /* R11 */
    *(--sp) = 0x0000000AU; /* R10 */
    *(--sp) = 0x00000009U; /* R9 */
    *(--sp) = 0x00000008U; /* R8 */
    *(--sp) = 0x00000007U; /* R7 */
    *(--sp) = 0x00000006U; /* R6 */
    *(--sp) = 0x00000005U; /* R5 */
    *(--sp) = 0x00000004U; /* R4 */

    /* save the top of the stack in the thread's attibute */
    me->sp = sp;

    /* round up the bottom of the stack to the 8-byte boundary */
    stk_limit = (uint32_t *)(((((uint32_t)stkSto - 1U) / 8) + 1U) * 8);

    /* pre-fill the unused part of the stack with 0xDEADBEEF */
    for (sp = sp - 1U; sp >= stk_limit; --sp) {
        *sp = 0xDEADBEEFU;
    }

    /* register the thread with the OS */
    OS_thread[OS_threadNum] = me;
    /* make the thread ready to run */
    if (OS_threadNum > 0U) {
        OS_readySet |= (1U << (OS_threadNum - 1U));
    }
    OS_threadNum++;
    return OS_threadNum-1;
}
/***********************************************/
void OS_onStartup(void) {
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / TICKS_PER_SEC);

    /* set the SysTick interrupt priority (highest) */
    NVIC_SetPriority(SysTick_IRQn, 0U);

}

void OS_onIdle(void) {
#ifdef NDBEBUG
    __WFI(); /* stop the CPU and Wait for Interrupt */
#endif
}

}//fim namespace

void Q_onAssert(char const *module, int loc) {
    /* TBD: damage control */
    (void)module; /* avoid the "unused parameter" compiler warning */
    (void)loc;    /* avoid the "unused parameter" compiler warning */
    NVIC_SystemReset();
}

/***********************************************/

/*
*/
__attribute__ ((naked, optimize("-fno-stack-protector")))
void PendSV_Handler(void) {

__asm volatile (

    /* __disable_irq(); */
    "  CPSID         I                 \n"

    /* if (OS_curr != (OSThread *)0) { */
    "  LDR           r1,=_ZN4rtos7OS_currE       \n"
    "  LDR           r1,[r1,#0x00]     \n"
    "  CBZ           r1,PendSV_restore \n"

	     // Salva FPU (se usada)
	        "  MRS    r0, CONTROL           \n"
	        "  TST    r0, #0x10             \n" // Bit FPCA
	        "  BEQ    save_no_fpu          \n"
	        "  SUB    sp, sp, #(17*4)      \n" // Reserva espaço: 16 S regs + FPSCR
	        "  VSTMDB sp!, {s16-s31}       \n" // Salva s16-s31
	        "  VMRS   r1, FPSCR            \n"
	        "  STR    r1, [sp, #(16*4)]    \n"
	        "save_no_fpu:                  \n"
    /*     push registers r4-r11 on the stack */
    "  PUSH          {r4-r11}          \n"

    /*     OS_curr->sp = sp; */
    "  LDR           r1,=_ZN4rtos7OS_currE       \n"
    "  LDR           r1,[r1,#0x00]     \n"
    "  STR           sp,[r1,#0x00]     \n"
    /* } */

    "PendSV_restore:                   \n"
    /* sp = OS_next->sp; */
    "  LDR           r1,=_ZN4rtos7OS_nextE       \n"
    "  LDR           r1,[r1,#0x00]     \n"
    "  LDR           sp,[r1,#0x00]     \n"

    /* OS_curr = OS_next; */
    "  LDR           r1,=_ZN4rtos7OS_nextE       \n"
    "  LDR           r1,[r1,#0x00]     \n"
    "  LDR           r2,=_ZN4rtos7OS_currE       \n"
    "  STR           r1,[r2,#0x00]     \n"

    /* pop registers r4-r11 */
    "  POP           {r4-r11}          \n"

		  // Restaura FPU (se usada)
		        "  MRS    r0, CONTROL          \n"
		        "  TST    r0, #0x10            \n"
		        "  BEQ    restore_no_fpu      \n"
		        "  VLDMIA sp!, {s16-s31}      \n"
		        "  LDR    r1, [sp], #(16*4)   \n"
		        "  VMSR   FPSCR, r1           \n"
		        "restore_no_fpu:              \n"

    /* __enable_irq(); */
    "  CPSIE         I                 \n"

    /* return to the next thread */
    "  BX            lr                \n"
    );
}
