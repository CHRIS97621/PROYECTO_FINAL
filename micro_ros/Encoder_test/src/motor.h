#ifndef MOTOR_H
#define MOTOR_H

class Motor {
private:
    int pin_pwm_;    // Pin para señal PWM

    int pin_dir1_;    // Pin para dirección
    int pin_dir2_; // Pin para habilitar/deshabilitar el motor
    float pwm_speed_; // Velocidad actual del motor (0-100%)

public:
    bool enable_motor_;    // Pin para señal PWM
    // Constructor
    Motor(int pwm_pin, int pin_dir1_, int pin_dir2_);
    void stop();
    void init();
    // Métodos públicos
    void setPwm(float speed); // Establece la velocidad del motor
    void enable();              // Habilita el motor
    void disable();             // Deshabilita el motor
    float getSpeed() const;     // Obtiene la velocidad actual
    void clean();
};

#endif // MOTOR_H
