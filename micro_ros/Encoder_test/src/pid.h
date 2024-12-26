#ifndef PID_H
#define PID_H

class PID {
private:
    float kp, ki, kd; // Constantes del controlador
    
    float setpoint_;   // Valor deseado
    float integral_;   // Componente integral
    float previous_error_; // Error en el paso anterior

    float signal_control_;
    float signal_error_;

public:
    // Constructor
    PID(float kp, float ki, float kd);

    void init();
    void reset();

    // Métodos públicos
    void setTunings(float kp, float ki, float kd); // Configurar los parámetros
    void setSetpoint(float setpoint);             // Establecer el valor objetivo
    float compute(float current_value, float delta_time);           // Calcular la salida del PID
    
    float getCurrentSignalControl();
    float getCurrentSignalError();
    float getSetpoint();
};

#endif // PID_H
