# RTOS MiROS – README

Este projeto implementa um sistema operacional de tempo real mínimo (MiROS) para ARM Cortex-M4 (STM32G4), suportando:

- **Tarefas Periódicas**: executadas a cada período predefinido.  
- **Tarefas Aperiódicas**: enfileiradas e servidas em segundo plano (background scheduling).  
- **Protocolo Não-Preemptivo**: preempção ativada/desativada via semáforo.

---

## Sumário
1. [Visão Geral](#visão-geral)  
2. [Agendador de Tarefas Periódicas](#agendador-de-tarefas-periódicas)  
3. [Servidor Aperiódico (Background Scheduling)](#servidor-aperiódico-background-scheduling)  
4. [Protocolo Não-Preemptivo](#protocolo-não-preemptivo)  


---

### Visão Geral
O MiROS fornece:  
- **Tick periódica** via SysTick para controlar tempo e deadlines  
- **Escalonamento por prioridades de período** (Rate-Monotonic)  
- **Serviço de tarefas aperiódicas** quando a CPU está ociosa  
- **Controle de preempção** que pode ser pausado para seções críticas  

---
### Servidor Aperiódico (Background Scheduling)

- O servidor aperiódico é controlado por duas funções:
  - `AperiodicServerStart()`: **habilita** o processamento de tarefas aperiódicas na idle thread.
  - `AperiodicServerStop()`: **desabilita** o processamento de tarefas aperiódicas, mesmo se houver tarefas na fila.

- Definido pela *idle thread* (índice 0). Em `main_idleThread()`:
  1. Enquanto houver tarefas aperiódicas enfileiradas (`OS_AperiodicTaskNum > 0`) e o servidor estiver ativo (`AperiodicServerStarted == true`), chama `osAperiodicWrapper()`.
  2. Caso contrário, chama `OS_onIdle()`. Dentro dessa função é executada a instrução `__WFI()` (_Wait For Interrupt_): a CPU entra em **modo de baixo consumo de energia**, aguardando a próxima interrupção para retomar a execução.

- `OSAperiodicTask_start(...)`: enfileira uma nova tarefa aperiódica para ser executada futuramente.
- `osAperiodicWrapper()`: executa a tarefa aperiódica no topo da fila e decrementa o contador (`OS_AperiodicTaskNum--`).

**Modelo utilizado:** background scheduling — tarefas aperiódicas são executadas **somente quando não há tarefas periódicas prontas** e **o servidor aperiódico estiver iniciado**.

- No `main()` chame `AperiodicServerStart()` antes de `OS_run()` para ativar o servidor aperiódico:

---

### Protocolo Não-Preemptivo
- **Semáforo** `MySemaphore preemptionAllowed` controla se a preempção está ativa.  
- **Funções**:  
  - `desativarPreempcao()`: bloqueia preempção (`tryLock()`).  
  - `reativarPreempcao()`: libera preempção (`tryUnlock()`).  
- No `OS_sched()`, ao comparar `OS_next != OS_curr`, só dispara PendSV se `preemptionAllowed.isAvailable()` for true.  
- Permite proteger seções críticas sem desativar globalmente todas as interrupções.  

