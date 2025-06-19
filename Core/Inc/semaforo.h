/*
 * semaforo.h
 *
 *  Created on: Apr 15, 2025
 *      Author: davim
 */

#ifndef SEMAFORO_H
#define SEMAFORO_H


#include "interruptController.h"

namespace rtos{

/**
 * @class MySemaphore
 * @brief Classe que implementa um semáforo básico para controle de acesso.
 *
 * Permite bloqueio e desbloqueio de recursos com suporte opcional a um controlador
 * de interrupções externo.
 */
class MySemaphore {
public:
	/**
	 * @class MySemaphore
	 * @brief Classe que implementa um semáforo básico para controle de acesso.
	 *
	 * Permite bloqueio e desbloqueio de recursos com suporte opcional a um controlador
	 * de interrupções externo.
	 */
    explicit MySemaphore(InterruptController* interruptCtrl = nullptr);


    /**
     * @brief Tenta adquirir o semáforo.
     *
     * @return true se o semáforo foi adquirido com sucesso, false se já estava ocupado.
     */
    bool tryLock();

    /**
     * @brief Tenta devolver o semáforo.
     *
     * @return true se o semáforo foi liberado com sucesso, false se já estava ocupado.
     */
    bool tryUnlock();
    /**
     * @brief Verifica se o semaforo esta bloqueado
     *
     * @return true se bloqueado, false se livre
     */
    bool isLocked();

    /**
     * @brief Verifica se o semaforo esta liberado
     *
     * @return true se livre, false se ocupado
     */
    bool isAvailable();

private:
    volatile bool ocupado;
    InterruptController* const ControleInterupcao;


    /*
     * @brief bloqueia o semaforo sem verificações
     */
    void lock();
    /*
    * @brief desbloqueia o semaforo sem verificações
     */
    void unlock();
};

}

#endif /* INC_SEMAFORO_H_ */
