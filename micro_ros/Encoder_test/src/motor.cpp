#include "motor.h"
#include <Arduino.h>

// Constructor
Motor::Motor(int pin_dir1, int pin_dir2, int channel1,int channel2) 
    : pin_dir1_(pin_dir1), pin_dir2_(pin_dir2), channel1_(channel1), channel2_(channel2){

  pwm_speed_ = 0.0;
}

void Motor::init(){

    const int ledChannel1 = channel1_;    // Canal PWM, puede ser de 0 a 15
    const int ledChannel2 = channel2_;    // Canal PWM, puede ser de 0 a 15
    const int frequency = 5000;  // Frecuencia en Hz
    const int resolution = 12;    // Resolución en bits (de 1 a 15)

    ledcSetup(ledChannel1, frequency, resolution);
    ledcSetup(ledChannel2, frequency, resolution);

    ledcAttachPin(pin_dir1_, ledChannel1);
    ledcAttachPin(pin_dir2_, ledChannel2);

    Serial.println("Motor Started");

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
        ledcWrite(channel1_, abs(pwm_speed_)); 
        ledcWrite(channel2_, 0); 
    }
    else if(pwm_speed_ == 0){
        ledcWrite(channel1_, 0); 
        ledcWrite(channel2_, 0); 
    } else {
        ledcWrite(channel1_, 0); 
        ledcWrite(channel2_, abs(pwm_speed_)); 
    }
}

// Habilita el motor
void Motor::stop() {        
    ledcWrite(channel1_, 0); 
    ledcWrite(channel2_, 0); 
}
