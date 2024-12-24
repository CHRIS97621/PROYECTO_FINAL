#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {

public:
    int pinA; // Pin para la señal A del encoder
    int pinB; // Pin para la señal B del encoder
    volatile long pulseCount_; // Contador de pulsos
    long lastPulseCount_;

    int PPR;
    unsigned long lastTime_; // Último tiempo para cálculo de velocidad

    float position; // Posición calculada (en unidades de tu elección)
    float speed; // Velocidad calculada (en unidades de tu elección)
    // Constructor
    Encoder(int pinA, int pinB);

    // Métodos
    void init(); // Configura los pines y el interrupt
    void update(); // Función de interrupción para contar pulsos
    float calculateSpeed(); // Calcula la velocidad en base a los pulsos
    float getPosition(); // Devuelve la posición
    void updateA();
    void updateB();
    void reset(); // Reinicia el contador de pulsos y posición
    static void IRAM_ATTR isrA();
    static void IRAM_ATTR isrB();

};

#endif 