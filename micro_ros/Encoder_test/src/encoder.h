#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
//#include "ESP32Encoder.h"

#include "low_filter.h"
#include "memory"

class Encoder {

private: 
    int PPR;

    float speed_; // Velocidad calculada (en unidades de tu elección)
    float speed_filtered_; // Velocidad calculada (en unidades de tu elección)

    int pinA; // Pin para la señal A del encoder
    int pinB; // Pin para la señal B del encoder

    volatile long pulseCount_; // Contador de pulsos
    long lastPulseCount_;   // Ultimo contador de pulsos
    
    unsigned long lastTime_; // Último tiempo para cálculo de velocidad
    float positionMotor_; // Posición calculada en radianes7

public:

    std::shared_ptr<LowPassFilter> lowPassFilter;   // Referencia al filtro

    // Constructor
    Encoder(int pinA, int pinB);
    // Métodos
    void init(); // Configura los pines y el interrupt
    void reset(); // Reinicia el contador de pulsos y posición

    float calculateSpeed(); // Calcula la velocidad en base a los pulsos
    float calculateSpeedFiltered(float speed); // Calcula la velocidad en base a los pulsos

    float getCount(); // Devuelve la posición
    float getSpeed();
    float getSpeedFiltered();
    float getPosition();


    void updateA();
    void updateB();

};

#endif 