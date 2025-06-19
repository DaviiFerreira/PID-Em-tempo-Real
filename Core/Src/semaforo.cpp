#include "semaforo.h"
#include <cstdint>

#include "stm32g4xx_it.h"
#include "miros.h"

namespace rtos{


void MySemaphore::lock() {
    ocupado = true;
}

void MySemaphore::unlock() {
    ocupado = false;
}
/**
 * @class MySemaphore
 * @brief Classe que implementa um semáforo básico para controle de acesso.
 *
 * Permite bloqueio e desbloqueio de recursos com suporte opcional a um controlador
 * de interrupções externo.
 */
MySemaphore::MySemaphore(InterruptController* interruptCtrl)
    : ocupado(false),
      ControleInterupcao(interruptCtrl ? interruptCtrl : &STM32InterruptController::getInstance()) {}


/**
   * @brief Tenta devolver o semáforo.
   *
   * @return true se o semáforo foi liberado com sucesso, false se já estava ocupado.
   * se seu codigo entrar em loop infinito no hardfault, voce esta tentando desbloquear
   * um semaforo que voce nao tem
   *
   */
bool MySemaphore::tryUnlock() {
    ControleInterupcao->disableInterrupts();

    if (isLocked()) {
        unlock();
        ControleInterupcao->enableInterrupts();
        return true;
    }
    HardFault_Handler();
    ControleInterupcao->enableInterrupts();
    return false;
}

/**
 * @brief Verifica se o semaforo esta bloqueado
 *
 * @return true se bloqueado, false se livre
 */
bool MySemaphore::isLocked() {
    return ocupado;
}


/**
 * @brief Verifica se o semaforo esta liberado
 *
 * @return true se livre, false se ocupado
 */
bool MySemaphore::isAvailable() {
   return !ocupado;
}
/**
   * @brief Tenta adquirir o semáforo.
   *
   * @return true se o semáforo foi adquirido com sucesso, false se já estava ocupado.
   */
bool MySemaphore::tryLock() {
    ControleInterupcao->disableInterrupts();
    if (isAvailable()) {
        lock();
        ControleInterupcao->enableInterrupts();
        return true;
    }
    OS_sched();
    ControleInterupcao->enableInterrupts();
    return false;
}
}
