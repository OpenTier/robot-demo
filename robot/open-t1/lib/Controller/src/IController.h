#pragma once

#include "Robot.h"

class IController {
public:
    virtual ~IController() = default;

    // A typical controller might have to init hardware/communication:
    virtual esp_err_t init() = 0; 

    // A method to be called periodically (e.g., in your main loop):
    virtual void update() = 0; 

    // Optionally, you might want to explicitly handle commands. 
    // But that depends on your design.
};
