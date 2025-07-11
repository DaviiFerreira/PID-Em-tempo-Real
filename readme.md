# Demonstrador Final - Sistemas de Tempo Real

Este projeto implementa um sistema operacional de tempo real mínimo (MiROS) para gerenciar tarefas que controlam a altura de uma bolinha de ping pong suspensa pelo fluxo de ar de um ventilador. É usando um sensor de distância VL53L0X para feedback e roda em um microcontrolador ARM Cortex-M4 (STM32G4).

---
## Funcionamento

O projeto utiliza de controle PID para variar a velocidade de um ventilador conforme o sinal de um sensor, alterando assim a altura de uma bolinha de ping pong suspensa pelo fluxo de ar. O código também permite que a altura da bolinha seja alterada ao pressionar um botão.
Seu funcionamento faz uso de:

- **Tarefas Periódicas**: executadas a cada período predefinido com protocolo de escalonamento RM (Rate Monotonic), com FIFO (First In First Out) para tarefas de prioridade igual.  As tarefas são:

  1. **Leitura do Sensor**: Mede a distância da bolinha periodicamente via I2C;
  2. **Calculo do PID**: Calcula a saída do controlador PID com base no erro entre a altura medida e o setpoint;
  3. **Controle da velocidade do ventilador**: Ajusta o duty cycle do PWM para o ventilador, controlando a velocidade do ar e consequentemente a altura da bolinha.

- **Tarefas Aperiódicas**: Gerenciadas por um servidor de tarefas aperiódicas em segundo plano (background scheduling).  A tarefa aperiódica é responsavel por fazer a altura da bolinha alterar de um setpoint para outro quando o botão é pressionado.

- **Protocolo Não-Preemptivo**: preempção ativada/desativada via semáforo para impedir que mais de uma tarefa altere as variáveis ao mesmo tempo.

---
### Ambiente de Desenvolvimento

O projeto foi realizado no ambiente de desenvolvimento **STM32CubeIDE**, utilizando a placa de desenvolvimento **Placa STM32G474RE**

---
### Hardware

  - Microcontrolador STM32G4.
  - Sensor de distância VL53L0X, conectado via I2C.
  - Ventilador Delta Electronics controlado por PWM.
  - Tubo de acrílico vertical de 950mm que guia a bolinha de ping pong, permitindo que ela suba ou desça com o fluxo de ar.

---
### Visão Geral Detalhada

O sistema é dividido em:

1. **MiROS (miros.cpp e miros.h)**: um RTOS educativo que implementa tarefas periódicas e aperiódicas, escalonamento Rate Monotonic (RM), gerenciamento de deadlines, servidor aperiódico e preempção controlada.
  - Tarefas são registradas por meio de `OSPeriodicTask_start` para periódicas e `OSAperiodicTask_start` para aperiódicas.
  - As deadlines são verificadas a cada tick do sistema para detectar misses.
  - Tarefas prontas são gerenciadas por um bitmask (`OS_readySet`) e uma lista de TCBs (Thread Control Blocks).

2. **Controle de concorrência (semaforo.cpp e semaforo.h)**: implementa a classe `MySemaphore`, com métodos `tryLock`, `tryUnlock`, `isAvailable`, protegendo regiões críticas do código.

3. **Ventilador (ventilador.cpp e ventilador.h)**: controla o duty cycle do PWM via `ventiladorSetDutyCycle()` e inicializa o hardware com `ventiladorInit()`. O PWM é gerado no canal TIM1_CH3.

4. **Sensor VL53L0X (VL53L0X.c)**: realiza a comunicação I2C com o sensor para medir a distância e retorna valores em milímetros.
  - Contém funções como `VL53L0X_init` e `VL53L0X_ReadSingleSimple` para configuração e leitura.

5. **Gerenciamento de interrupções (interruptController.cpp e interruptController.h)**: abstrai habilitar/desabilitar interrupções com métodos seguros (`enableInterrupts`, `disableInterrupts`).

6. **main.cpp**: inicializa o sistema, cria as tarefas e define o comportamento geral.

  - Três tarefas periódicas: `LerSensor`, `CalculoPid`, `SetaVelocidade`.
  - Uma tarefa aperiódica: mudança do setpoint via botão.
  - PID é calculado na função `testePid(double medida, double setpoint)` com termos proporcional, integral e derivativo.
  - Sistema inicializa stacks, GPIOs, I2C e timers com funções `MX_GPIO_Init()`, `MX_I2C1_Init()`, `ventiladorInit()`.

## Autoria e Créditos

- **Bruna Campos Santana**,
- **Davi Ferrera**,
- **Daniel Augusto Santana Porto**,
- **Gabriel Andrea Carvalho**,
- **Maria Eduarda Favero Razia**.

Estudantes de Engenharia Mecatrônica – UFSC  

Este projeto foi desenvolvido como parte de uma atividade acadêmica com base em códigos fornecidos pelo professor **Gian Ricardo Berkenbrock**, no contexto da disciplina de Sistemas de Tempo Real.

---
