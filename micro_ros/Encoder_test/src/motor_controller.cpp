#include "motor_controller.h"

// Constructor
MotorController::MotorController(int dir_pin1, int dir_pin2, int channel1, int channel2,
                                 int encoder_pin_a, int encoder_pin_b, 
                                 float kp, float ki, float kd)
    : motor(dir_pin1, dir_pin2,channel1,channel2), 
      encoder(encoder_pin_a, encoder_pin_b), 
      pid(kp, ki, kd){
    
    rate = 0.02;
    setpoint = 0.0;
    enable_control_ = false;
}

//Inicializo el motor controller
void MotorController::init() {

    
    encoder.init();
    motor.init();
    pid.init();
}

//Inicializo el motor controller
void MotorController::reset() {

    encoder.reset();
    motor.reset();
    pid.reset();
}



// Configura la velocidad deseada
void MotorController::setTargetSpeed(float speed) {
    setpoint = speed;
    // le paso el setpoint a l pid
    pid.setSetpoint(speed);
}


// Actualiza el control del motor
void MotorController::update() {

    float current_speed = encoder.calculateSpeed(); // Obtén la velocidad actual del encoder
    float current_speed_filtered = encoder.calculateSpeedFiltered(current_speed);
    if(enable_control_ ){
        float control_signal = pid.compute(current_speed_filtered,rate); // Calcula la señal de control
        motor.setPwm(control_signal);
    }    
}

// Actualiza el control del motor
void MotorController::enableControl() {
    MotorController::reset();
    enable_control_ = true;
}

// Actualiza el control del motor
void MotorController::disableControl() {
    motor.stop();
    enable_control_ = false;
}
