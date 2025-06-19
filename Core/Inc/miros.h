/*
 * miros.h
 *
 *  Created on: Feb 6, 2025
 *      Author: guilh
 */

#ifndef INC_MIROS_H_
#define INC_MIROS_H_

namespace rtos {



/* Thread Control Block (TCB) */
typedef struct {
    void *sp; /* stack pointer */
    uint32_t timeout; /* timeout delay down-counter */
    /* ... other attributes associated with a thread */
} OSThread;
typedef void (*OSThreadHandler)();

typedef struct {
	OSThread my_Thread;
	uint32_t Period;
	uint32_t lastAtivation;
	uint8_t myThreadIndex;
	uint8_t myPeriodicTaskIndex;
	OSThreadHandler myTask;

} OSPeriodicTask;

typedef struct {
	OSThreadHandler myTask;

} OSAperiodicTask;
void osAperiodicWrapper();
void desativarPreempcao();
void reativarPreempcao();


void AperiodicServerStart();

void AperiodicServerStop();

 uint32_t gcd(uint32_t a, uint32_t b);
 uint32_t lcm(uint32_t a, uint32_t b);
const uint16_t TICKS_PER_SEC = 100U;

void OSPeriodicTask_start(OSPeriodicTask *me,
	    OSThreadHandler threadHandler,
	    void *stkSto, uint32_t stkSize, uint32_t period);

void OSAperiodicTask_start(OSAperiodicTask *me,
    OSThreadHandler threadHandler);



void OS_init(void *stkSto, uint32_t stkSize);

/* callback to handle the idle condition */
void OS_onIdle(void);

/* this function must be called with interrupts DISABLED */
void OS_sched(void);

/* transfer control to the RTOS to run the threads */
void OS_run(void);

/* blocking delay */
void OS_delay(uint32_t ticks);

/* process all timeouts */
void OS_tick(void);

/* callback to configure and start interrupts */
void OS_onStartup(void);

uint8_t OSThread_start(
    OSThread *me,
    OSThreadHandler threadHandler,
    void *stkSto, uint32_t stkSize);

}

#endif /* INC_MIROS_H_ */
