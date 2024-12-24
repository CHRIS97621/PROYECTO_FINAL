#include "encoder.h"

Encoder* instance = nullptr;
// Constructor
Encoder::Encoder(int pinA, int pinB)
    : pinA(pinA), pinB(pinB), pulseCount_(0), position(0.0), speed(0.0), lastTime_(0) {

        instance = this;
        PPR = 1320;
    }

// Inicializa el encoder
void Encoder::init() {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    attachInterrupt(pinA, isrA, CHANGE); 
    attachInterrupt(pinB, isrB, CHANGE);

    Serial.println("Encoder initialized");
}

// Función de interrupción para actualizar el contador de pulsos
void IRAM_ATTR Encoder::isrA() {
    if (instance) {
        instance->updateA();
    }
}

// ISR estática para el pin B
void IRAM_ATTR Encoder::isrB() {
    if (instance) {
        instance->updateB();
    }
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

    Serial.print("El delta de tiempo es ");
    Serial.println(deltaTime);
    speed = (pulseCount_ - lastPulseCount_)*2*M_PI / (deltaTime*1.320); // Pulsos por segundo
    //actualizo valores
    lastTime_ = currentTime;
    lastPulseCount_ = pulseCount_;
    
    return speed;
}

// Devuelve la posición actual (basado en pulsos acumulados)
float Encoder::getPosition() {
    return pulseCount_;
}

// Reinicia los valores del encoder
void Encoder::reset() {
    pulseCount_ = 0;
    lastTime_ = 0;
    lastPulseCount_ = 0;
    position = 0.0;
    speed = 0.0;
}
