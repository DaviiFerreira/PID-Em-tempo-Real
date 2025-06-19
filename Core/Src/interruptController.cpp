#include <cstdint>
#include "miros.h"
#include "qassert.h"
#include "stm32g4xx.h"
#include "interruptController.h"

namespace rtos {


void STM32InterruptController::disableInterrupts() {
    __disable_irq();
}

void STM32InterruptController::enableInterrupts() {
    __enable_irq();
}

STM32InterruptController& STM32InterruptController::getInstance() {
    static STM32InterruptController instance;
    return instance;
}



}
