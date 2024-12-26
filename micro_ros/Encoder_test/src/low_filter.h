#include "Arduino.h"
#ifndef LOW_FILTER_H
#define LOW_FILTER_H

class LowPassFilter {
private:
    float alpha_;             // Factor de suavizado
    float filteredValue;     // Último valor filtrado
    bool firstUpdate;        // Indica si es la primera muestra

public:
    // Constructor
    LowPassFilter(float alpha) : alpha_(alpha) {
    }

    // Constructor
    void init() {
        filteredValue = 0.0;
        firstUpdate = true;
    }
    void setParam(float alpha) {
        alpha_ = alpha;
    }
    // Constructor
    void reset() {
        filteredValue = 0.0;
        firstUpdate = true;
    }

    // Método para actualizar el filtro con un nuevo valor
    float update(float newValue) {
        if (firstUpdate) {
            // La primera vez usamos directamente el valor medido
            filteredValue = newValue;
            firstUpdate = false;
        } else {
            // Aplicar el filtro de paso bajo
            filteredValue = alpha_ * newValue + (1 - alpha_) * filteredValue;
        }
        return filteredValue;
    }
};

#endif 