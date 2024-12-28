#include <Arduino.h>
#include "pid.h"

// Constructor
PID::PID(float kp, float ki, float kd) 
    : kp(kp), ki(ki), kd(kd){}

// Configura los parámetros del PID
void PID::setTunings(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    Serial.print("PID seteado con ");
    Serial.print("kp: [");
    Serial.print(kp);
    Serial.print("] ki: [");
    Serial.print(ki);
    Serial.print("] kd: [");
    Serial.print(kd);
    Serial.println("]");
}

// Establece el valor deseado
void PID::setSetpoint(float setpoint) {
    this->setpoint_ = setpoint;
}


// Establece el valor deseado
void PID::init() {
    setpoint_ = 0.0;
    integral_ = 0.0;
    previous_error_ = 0.0;

    signal_control_ = 0.0;
    signal_error_ = 0.0;
    Serial.print("PID inicilizado con ");
    Serial.print("kp: [");
    Serial.print(kp);
    Serial.print("] ki: [");
    Serial.print(ki);
    Serial.print("] kd: [");
    Serial.print(kd);
    Serial.println("]");
}

// reseteo los valores de PID
void PID::reset() {
    setpoint_ = 0.0;
    integral_ = 0.0;
    previous_error_ = 0.0;
    
    signal_control_ = 0.0;
    signal_error_ = 0.0;
}


// Calcula la salida del PID
float PID::compute(float current_value, float delta_time) {
    float error = setpoint_ - current_value;

    if(abs(error)<0.5 && setpoint_ == 0){
        return 0;
    }

    signal_error_ =  error;
    integral_ += error;
    float derivative = error - previous_error_;

    float output = (kp * error) + (ki * integral_)* delta_time + (kd * derivative)/delta_time;
    signal_control_ = output;
    previous_error_ = error;

    output = constrain(output, -4095.0, 4095.0); // Limitar entre -100% y 100%
    return output; // Retorna la señal de control
}

float PID::getCurrentSignalError(){

    return signal_error_;
}
float PID::getCurrentSignalControl(){

    return signal_control_;
}

float PID::getSetpoint(){

    return setpoint_;
}