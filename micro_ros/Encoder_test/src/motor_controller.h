#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "motor.h"
#include "pid.h"
#include "encoder.h" // Asumimos que la velocidad actual provendrá del encoder
#include "low_filter.h"

class MotorController {
private:

public:
    Motor motor;     // Referencia al motor
    Encoder encoder; // Referencia al encoder
    PID pid;         // Controlador PID
    float setpoint;   // Velocidad deseada

    // Constructor
    MotorController(int pwm_pin, int dir_pin, int enable_pin, 
                    int encoder_pin_a, int encoder_pin_b, 
                    float kp, float ki, float kd);

    // Métodos públicos
    void setTargetSpeed(float speed); // Configurar la velocidad deseada
    void update();                    // Actualizar el control del motor
    void init();
    LowPassFilter* lowPassFilter;
};

#endif // MOTOR_CONTROLLER_H
