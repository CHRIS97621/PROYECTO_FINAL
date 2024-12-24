#ifndef LOW_FILTER_H
#define LOW_FILTER_H


class LowPassFilter {
private:
    float alpha;             // Factor de suavizado
    float filteredValue;     // Último valor filtrado
    bool firstUpdate;        // Indica si es la primera muestra

public:
    // Constructor
    LowPassFilter(float alpha) : alpha(alpha), filteredValue(0.0), firstUpdate(true) {}

    // Método para actualizar el filtro con un nuevo valor
    float update(float newValue) {
        if (firstUpdate) {
            // La primera vez usamos directamente el valor medido
            filteredValue = newValue;
            firstUpdate = false;
        } else {
            // Aplicar el filtro de paso bajo
            filteredValue = alpha * newValue + (1 - alpha) * filteredValue;
        }
        return filteredValue;
    }
};

#endif 