#ifndef PID_H
#define PID_H

class PID {
private:
    float kp, ki, kd; // Constantes del controlador
    float setpoint;   // Valor deseado
    float integral;   // Componente integral
    float previous_error; // Error en el paso anterior

    float signal_control_;
    float signal_error_;

public:
    // Constructor
    PID(float kp, float ki, float kd);

    // Métodos públicos
    void setTunings(float kp, float ki, float kd); // Configurar los parámetros
    void setSetpoint(float setpoint);             // Establecer el valor objetivo
    float compute(float current_value);           // Calcular la salida del PID
    void clean();
    float getCurrentSignalControl();
    float getCurrentSignalError();
};

#endif // PID_H
