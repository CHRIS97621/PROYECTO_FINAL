#include "encoder.h"
#include "map"


// Constructor
Encoder::Encoder(int pinA, int pinB)
    : pinA(pinA), pinB(pinB), pulseCount_(0), positionMotor_(0.0), speed_(0.0), lastTime_(0) {

        PPR = 1320;
        lowPassFilter = std::make_shared<LowPassFilter>(0.2);
    }

// Inicializa el encoder
void Encoder::init() {

    Serial.println("Encoder initialized");
    lowPassFilter->init();
}



// Maneja los cambios en el pin A
void Encoder::updateA() {
    int stateA = digitalRead(pinA);
    int stateB = digitalRead(pinB);

    if (stateA == stateB) {
        pulseCount_++;
    } else {
        pulseCount_--;
    }
}

// Maneja los cambios en el pin B
void Encoder::updateB() {
    int stateA = digitalRead(pinA);
    int stateB = digitalRead(pinB);

    if (stateA != stateB) {
        pulseCount_++;

    } else {
        pulseCount_--;
    }
}

// Calcula la velocidad (llamar periódicamente en el loop)
float Encoder::calculateSpeed() {
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - lastTime_;

    // Hallo la velocidad
    speed_ = (pulseCount_ - lastPulseCount_)*2.0*M_PI*1000.0 / (deltaTime*PPR);

    //actualizo valores
    lastTime_ = currentTime;
    lastPulseCount_ = pulseCount_;

    //calculo la posición actual
    positionMotor_ = pulseCount_*2*M_PI/1320.0;

    return speed_;
}

float Encoder::calculateSpeedFiltered(float speed) {

    float current_speed_filtered = lowPassFilter->update(speed);
    speed_filtered_ = current_speed_filtered;
    return current_speed_filtered;
}

// Devuelve la posición actual (basado en pulsos acumulados)
float Encoder::getCount() {
    return pulseCount_;
}

// Devuelve la velocidad actual del robot
float Encoder::getSpeed() {
    return speed_;
}

// Devuelve la velocidad actual del robot
float Encoder::getPosition() {
    return positionMotor_;
}

// Devuelve la velocidad filtrada actual del robot
float Encoder::getSpeedFiltered() {
    return speed_filtered_;
}

// Reinicia los valores del encoder
void Encoder::reset() {
    lowPassFilter->reset();
    pulseCount_ = 0;
    lastTime_ = 0;
    lastPulseCount_ = 0;
    speed_ = 0.0;
    positionMotor_ = 0.0;
}
