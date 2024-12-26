#ifndef MOTOR_H
#define MOTOR_H

class Motor {
private:
    int pin_pwm_;    // Pin para señal PWM

    int pin_dir1_;    // Pin para dirección
    int pin_dir2_; // Pin para habilitar/deshabilitar el motor
    float pwm_speed_; // Velocidad actual del motor (0-100%)
    int channel_;
public:
    // Constructor
    Motor(int pwm_pin, int pin_dir1, int pin_dir2, int channel);

    void stop();
    void init();
    void reset();
    
    // Métodos públicos
    void setPwm(float speed); // Setea un pwm 
};

#endif // MOTOR_H
