#ifndef interruptController_h
#define interruptController_h
namespace rtos {

class InterruptController {
public:
    virtual void disableInterrupts() = 0;
    virtual void enableInterrupts() = 0;
    virtual ~InterruptController() = default;
};

class STM32InterruptController : public InterruptController {
public:
    void disableInterrupts() override;
    void enableInterrupts() override;
    static STM32InterruptController& getInstance();
};
}

#endif
