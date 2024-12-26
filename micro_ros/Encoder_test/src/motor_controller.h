#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "motor.h"
#include "pid.h"
#include "encoder.h" // Asumimos que la velocidad actual provendrá del encoder

#include "functional"
#include "memory"
#include "map"

class MotorController {
private:
    bool enable_control_;    // flag para habilitar el control
    float setpoint;   // Velocidad seteado
    float rate;   // Velocidad seteado

public:
    Motor motor;     // Referencia al motor
    Encoder encoder; // Referencia al encoder
    PID pid;         // Referencia PID
    
    
    // Constructor
    MotorController(int pwm_pin, int dir_pin, int enable_pin, int channel,
                    int encoder_pin_a, int encoder_pin_b, 
                    float kp, float ki, float kd);
    // Métodos públicos
    void init();  
    void reset();  
    void enableControl();
    void disableControl();

    void setTargetSpeed(float speed); // Setear un setpoint
    void update();                    // Actualizar el control del motor
                      


};

#endif // MOTOR_CONTROLLER_H
