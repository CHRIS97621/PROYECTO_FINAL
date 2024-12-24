#include <Arduino.h>
#include "pid.h"

// Constructor
PID::PID(float kp, float ki, float kd) 
    : kp(kp), ki(ki), kd(kd), setpoint(0), integral(0), previous_error(0), signal_control_(0), signal_error_(0) {}

// Configura los parámetros del PID
void PID::setTunings(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

// Establece el valor deseado
void PID::setSetpoint(float setpoint) {
    this->setpoint = setpoint;
}


// Establece el valor deseado
void PID::clean() {
    integral = 0.0;
    previous_error = 0.0;
    signal_control_ = 0.0;
    signal_error_ = 0.0;
}

// Calcula la salida del PID
float PID::compute(float current_value) {
    float error = setpoint - current_value;

    signal_error_ =  error;
    integral += error;
    float derivative = error - previous_error;

    float output = (kp * error) + (ki * integral) + (kd * derivative);
    signal_control_ = output;
    previous_error = error;

    output = constrain(output, -100.0, 100.0); // Limitar entre -100% y 100%
    return output; // Retorna la señal de control
}

float PID::getCurrentSignalError(){

    return signal_error_;
}
float PID::getCurrentSignalControl(){

    return signal_control_;
}