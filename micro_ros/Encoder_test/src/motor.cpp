#include "motor.h"
#include <Arduino.h>

// Constructor
Motor::Motor(int pin_pwm, int pin_dir1, int pin_dir2, int channel) 
    : pin_pwm_(pin_pwm), pin_dir1_(pin_dir1), pin_dir2_(pin_dir2), pwm_speed_(0), channel_(channel){


}

void Motor::init(){

    const int ledChannel = channel_;    // Canal PWM, puede ser de 0 a 15
    const int frequency = 5000;  // Frecuencia en Hz
    const int resolution = 12;    // Resolución en bits (de 1 a 15)

    pinMode(pin_dir1_, OUTPUT);
    pinMode(pin_dir2_, OUTPUT);

    ledcSetup(ledChannel, frequency, resolution);
    ledcAttachPin(pin_pwm_, ledChannel);

    Serial.println("Motor Started");
    Serial.println(channel_);
}

void Motor::reset(){

    pwm_speed_ = 0.0;
}

// Establece la velocidad del motor
void Motor::setPwm(float speed) {
    pwm_speed_ = constrain(speed, -4095.0, 4095.0); // Limitar entre -100% y 100%
    Serial.print("PWM seteado ");
    Serial.println(pwm_speed_);

    // Determinar dirección
    if (pwm_speed_ > 0) {
        digitalWrite(pin_dir1_, HIGH); // Dirección hacia adelante
        digitalWrite(pin_dir2_, LOW); // Dirección hacia adelante
    }
    else if(pwm_speed_ == 0){
        digitalWrite(pin_dir1_, HIGH); // Dirección hacia adelante
        digitalWrite(pin_dir2_, HIGH); // Dirección hacia adelante
    } else {
        digitalWrite(pin_dir1_, LOW); // Dirección hacia adelante
        digitalWrite(pin_dir2_, HIGH); // Dirección hacia adelante
    }

    // Aplicar PWM
    ledcWrite(channel_, abs(pwm_speed_)); 
}

// Habilita el motor
void Motor::stop() {        
    digitalWrite(pin_dir1_, HIGH); // Dirección hacia adelante
    digitalWrite(pin_dir2_, HIGH); // Dirección hacia adelante
    ledcWrite(channel_, 0);
}
