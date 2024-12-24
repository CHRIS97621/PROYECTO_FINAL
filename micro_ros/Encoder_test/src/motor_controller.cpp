#include "motor_controller.h"

// Constructor
MotorController::MotorController(int pwm_pin, int dir_pin, int enable_pin, 
                                 int encoder_pin_a, int encoder_pin_b, 
                                 float kp, float ki, float kd)
    : motor(pwm_pin, dir_pin, enable_pin), 
      encoder(encoder_pin_a, encoder_pin_b), 
      pid(kp, ki, kd), 
      setpoint(0),lowPassFilter(nullptr) {

    lowPassFilter = new LowPassFilter(0.5); 

}

// Configura la velocidad deseada
void MotorController::setTargetSpeed(float speed) {
    setpoint = speed;
    pid.setSetpoint(speed);
}
void MotorController::init() {
    encoder.init();
    motor.init();
}

// Actualiza el control del motor
void MotorController::update() {
    float current_speed = encoder.calculateSpeed(); // Obtén la velocidad actual del encoder
    float current_speed_filtered = lowPassFilter->update(current_speed);
    float control_signal = pid.compute(current_speed_filtered); // Calcula la señal de control

    // Controla la velocidad del motor usando la señal del PID
    motor.setPwm(control_signal);
}
