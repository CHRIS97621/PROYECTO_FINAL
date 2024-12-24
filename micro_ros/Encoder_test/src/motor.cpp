#include "motor.h"
#include <Arduino.h>

// Constructor
Motor::Motor(int pin_pwm_, int pin_dir1_, int pin_dir2_) 
    : pin_pwm_(pin_pwm_), pin_dir1_(pin_dir1_), pin_dir2_(pin_dir2_), pwm_speed_(0), enable_motor_(false) {


}

void Motor::init(){

    const int ledChannel = 0;    // Canal PWM, puede ser de 0 a 15
    const int frequency = 5000;  // Frecuencia en Hz
    const int resolution = 12;    // Resolución en bits (de 1 a 15)


    pinMode(pin_dir1_, OUTPUT);
    pinMode(pin_dir2_, OUTPUT);

    ledcSetup(ledChannel, frequency, resolution);
    ledcAttachPin(pin_pwm_, ledChannel);
    Serial.println("Motor Started");
}

// Establece la velocidad del motor
void Motor::setPwm(float speed) {
    pwm_speed_ = constrain(speed, -100.0, 100.0); // Limitar entre -100% y 100%
    
    if(!enable_motor_){
        
    }
    else{
        // Determinar dirección
        if (pwm_speed_ > 0) {
            digitalWrite(pin_dir1_, HIGH); // Dirección hacia adelante
            digitalWrite(pin_dir2_, LOW); // Dirección hacia adelante
        }
        else if(pwm_speed_ == 0){
            digitalWrite(pin_dir1_, LOW); // Dirección hacia adelante
            digitalWrite(pin_dir2_, LOW); // Dirección hacia adelante
        } else {
            digitalWrite(pin_dir1_, LOW); // Dirección hacia adelante
            digitalWrite(pin_dir2_, HIGH); // Dirección hacia adelante
        }
    }
    

    // Aplicar PWM
    analogWrite(pin_pwm_, abs(pwm_speed_) * 2.55); // Convertir % a valor PWM (0-255)
}

// Habilita el motor
void Motor::stop() {        
    digitalWrite(pin_dir1_, LOW); // Dirección hacia adelante
    digitalWrite(pin_dir2_, LOW); // Dirección hacia adelante
    ledcWrite(pin_pwm_, 0);
}

// Habilita el motor
void Motor::enable() {
    stop();
   
    enable_motor_=true;
}

// Deshabilita el motor
void Motor::disable() {
    clean();
    stop();
    enable_motor_=false;
}


// Reiniciar valores
void Motor::clean() {
    enable_motor_ = false;
    pwm_speed_ = 0.0;
}

// Obtiene la velocidad actual
float Motor::getSpeed() const {
    return pwm_speed_;
}
