#include "ros_communication.h"
#include "Arduino.h"


rcl_subscription_t led_sub;
rcl_subscription_t set_vel_motors_sub;
rcl_subscription_t set_switch_control_sub;
rcl_subscription_t set_pwm_motors_sub;
rcl_subscription_t config_params_sub;

rcl_publisher_t current_pulses_counts_pub;
rcl_publisher_t wheels_state_pub;
rcl_publisher_t control_state_pub;


std_msgs__msg__Bool led_msg;


agro_interfaces__msg__Motors pulse_counts_msg;
agro_interfaces__msg__Motors set_pwm_motors_msg;


std_msgs__msg__Bool set_switch_control_msg;
agro_interfaces__msg__ControlStatus motor_status_control_msg;
agro_interfaces__msg__ConfigParams config_params_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

rcl_node_t node;
rcl_timer_t timer;
rcl_timer_t timer_control;


//joint_states
sensor_msgs__msg__JointState wheels_state_msg;
std_msgs__msg__Float32MultiArray wheels_command_msg;

//#define ledPin 32
RosCommunication* instance_ros_comunication = nullptr; 
MotorController* instanceFL = nullptr;
MotorController* instanceFR = nullptr;
MotorController* instanceNL = nullptr;
MotorController* instanceNR = nullptr;

// Encoders pines
struct MotorPin{
  int encoder_A;
  int encoder_B;
  int pwm_IN1; 
  int pwm_IN2; 
  int channel_IN1; 
  int channel_IN2;
};

MotorPin motorFL;
MotorPin motorFR;
MotorPin motorNL;
MotorPin motorNR;

RosCommunication::RosCommunication(): motorControllerFL(nullptr), motorControllerFR(nullptr),motorControllerNL(nullptr),motorControllerNR(nullptr){
    Serial.println("RosCommunication instance created.");
    instance_ros_comunication = this;

}

void RosCommunication::initialize(){
    Serial.begin(115200);
    Serial.println("ROS Communication node started");


    // Adding Wifi
    IPAddress agent_ip(192, 168, 1,62);
    size_t agent_port = 8888;
    char ssid[] = "LAURA";
    char psk[]= "47854643";

    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

    delay(2000);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "firmware", "", &support);



    motorFL.encoder_A = 25;
    motorFL.encoder_B = 26;
    motorFL.pwm_IN1 = 16; 
    motorFL.pwm_IN2 = 27; 
    motorFL.channel_IN1 = 0; 
    motorFL.channel_IN2 = 1; 

    motorFR.encoder_A = 35;
    motorFR.encoder_B = 34;
    motorFR.pwm_IN1 = 19; 
    motorFR.pwm_IN2 = 21; 
    motorFR.channel_IN1 = 2; 
    motorFR.channel_IN2 = 3;

    motorNL.encoder_A = 32;
    motorNL.encoder_B = 33;
    motorNL.pwm_IN1 = 17; 
    motorNL.pwm_IN2 = 18; 
    motorNL.channel_IN1 = 4; 
    motorNL.channel_IN2 = 5; 

    motorNR.encoder_A = 36;
    motorNR.encoder_B = 39;
    motorNR.pwm_IN1 = 23; 
    motorNR.pwm_IN2 = 22; 
    motorNR.channel_IN1 = 6; 
    motorNR.channel_IN2 = 7; 

    /*MotorController(           int pwm_IN1, int pwm_IN1, 
                                 int channel_IN1, int channel_IN2, 
                                 int encoder_pin_a, int encoder_pin_b, 
                                 float kp, float ki, float kd)*/
    // motor front left
    motorControllerFL = new MotorController(motorFL.pwm_IN1,     motorFL.pwm_IN2,
                                            motorFL.channel_IN1, motorFL.channel_IN2,   
                                            motorFL.encoder_A,   motorFL.encoder_B,    
                                            200.0, 70.0, 1.0); 
    // motor front rigth
    motorControllerFR = new MotorController(motorFR.pwm_IN1, motorFR.pwm_IN2,
                                            motorFR.channel_IN1, motorFR.channel_IN2,   
                                            motorFR.encoder_A, motorFR.encoder_B,    
                                           200.0, 70.0, 1.0); 
    // motor near left
    motorControllerNL = new MotorController(motorNL.pwm_IN1,     motorNL.pwm_IN2,
                                            motorNL.channel_IN1, motorNL.channel_IN2,   
                                            motorNL.encoder_A,   motorNL.encoder_B,     
                                           200.0, 70.0, 1.0);
    // motor near right
    motorControllerNR = new MotorController(motorNR.pwm_IN1,     motorNR.pwm_IN2,
                                            motorNR.channel_IN1, motorNR.channel_IN2,   
                                            motorNR.encoder_A,   motorNR.encoder_B,      
                                           200.0, 70.0, 1.0); 




    pinMode(motorFL.encoder_A, INPUT_PULLUP);
    pinMode(motorFL.encoder_B, INPUT_PULLUP);
    pinMode(motorFR.encoder_A, INPUT_PULLUP);
    pinMode(motorFR.encoder_B, INPUT_PULLUP);
    pinMode(motorNL.encoder_A, INPUT_PULLUP);
    pinMode(motorNL.encoder_B, INPUT_PULLUP);
    pinMode(motorNR.encoder_A, INPUT_PULLUP);
    pinMode(motorNR.encoder_B, INPUT_PULLUP);

    attachInterrupt(motorFL.encoder_A, isrA1, CHANGE); 
    attachInterrupt(motorFL.encoder_B, isrB1, CHANGE);

    attachInterrupt(motorFR.encoder_A, isrB2, CHANGE); 
    attachInterrupt(motorFR.encoder_B, isrA2, CHANGE);

    attachInterrupt(motorNL.encoder_A, isrA3, CHANGE); 
    attachInterrupt(motorNL.encoder_B, isrB3, CHANGE);

    attachInterrupt(motorNR.encoder_A, isrA4, CHANGE); 
    attachInterrupt(motorNR.encoder_B, isrB4, CHANGE);


    Serial.println("Encoder initialized");

    motorControllerFL->init();
    motorControllerFR->init();
    motorControllerNL->init();
    motorControllerNR->init();
    instanceFL = motorControllerFL;
    instanceFR = motorControllerFR;
    instanceNL = motorControllerNL;
    instanceNR = motorControllerNR;
}
// Función de interrupción para actualizar el contador de pulsos
// MOTOR FL
void IRAM_ATTR RosCommunication::isrA1() {
    if (instanceFL) {
        instanceFL->encoder.updateA();
    }
}
void IRAM_ATTR RosCommunication::isrB1() {
    if (instanceFL) {
        instanceFL->encoder.updateB();
    }
}
// MOTOR FR
void IRAM_ATTR RosCommunication::isrA2() {
    if (instanceFR) {
        instanceFR->encoder.updateA();
    }
}
void IRAM_ATTR RosCommunication::isrB2() {
    if (instanceFR) {
        instanceFR->encoder.updateB();
    }
}
// MOTOR NL
void IRAM_ATTR RosCommunication::isrA3() {
    if (instanceNL) {
        instanceNL->encoder.updateA();
    }
}
void IRAM_ATTR RosCommunication::isrB3() {
    if (instanceNL) {
        instanceNL->encoder.updateB();
    }
}
// MOTOR NR
void IRAM_ATTR RosCommunication::isrA4() {
    if (instanceNR) {
        instanceNR->encoder.updateA();
    }
}
void IRAM_ATTR RosCommunication::isrB4() {
    if (instanceNR) {
        instanceNR->encoder.updateB();
    }
}

void RosCommunication::executors_start(){
  // Crear ejecutor
  RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &led_sub, &led_msg,   RosCommunication::led_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &set_vel_motors_sub, &wheels_command_msg,   RosCommunication::set_vel_motors_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &set_switch_control_sub, &set_switch_control_msg,   RosCommunication::set_switch_control_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &set_pwm_motors_sub, &set_pwm_motors_msg,   RosCommunication::set_pwm_motors_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &config_params_sub, &config_params_msg,   RosCommunication::config_params_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_control));

  instance_ros_comunication->fill_wheels_state_msg(&wheels_state_msg);
  instance_ros_comunication->fill_wheels_command_msg(&wheels_command_msg);
  Serial.println("Executors Started");
}


void RosCommunication::publishers_define() {
    // Publicador de la cantidad de pulsos
    RCCHECK(rclc_publisher_init_default(
        &current_pulses_counts_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(agro_interfaces, msg, Motors),
        "current_pulses_counts"));

    // Publicador para la información de control
    RCCHECK(rclc_publisher_init_default(
        &control_state_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(agro_interfaces, msg, ControlStatus),
        "control_info"));
    // Publicador para la información de control
    RCCHECK(rclc_publisher_init_default(
        &wheels_state_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "_motors_response"));
}

void RosCommunication::subscribers_define() {
    // Crear suscriptor led
    RCCHECK(rclc_subscription_init_default(
        &led_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/led_state"));

    // Crear subscriptor para atender las velocidad enviadas
    RCCHECK(rclc_subscription_init_default(
        &set_vel_motors_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32MultiArray),
        "/_motors_cmd"));

    RCCHECK(rclc_subscription_init_default(
        &set_pwm_motors_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(agro_interfaces, msg, Motors),
        "/set_pwm_motors"));

    RCCHECK(rclc_subscription_init_default(
        &set_switch_control_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/set_switch_control"));

    RCCHECK(rclc_subscription_init_default(
        &config_params_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(agro_interfaces, msg, ConfigParams),
        "/config_params"));
}
void RosCommunication::timers_define() {
    // Crear temporizador
    const unsigned int timer_timeout = 1000; // Tiempo en milisegundos
    const unsigned int timer_timeout_control = 20; // Tiempo en milisegundos
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        &RosCommunication::timer_callback);
        //,true);

    rclc_timer_init_default(
        &timer_control,
        &support,
        RCL_MS_TO_NS(timer_timeout_control),
     &RosCommunication::timer_callback_control);
        //,true);
        
}



void RosCommunication::led_callback(const void *msg_recv) {
    const std_msgs__msg__Bool *data = (const std_msgs__msg__Bool *)msg_recv;

    Serial.print("El estado del LED es: ");
    Serial.println(data->data ? "Encendido" : "Apagado");

    //digitalWrite(ledPin,data->data);
}

void RosCommunication::set_vel_motors_callback(const void *msg_recv) {
    const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msg_recv;
    Serial.print("Recibiendo comandos de velocidad");
    if (msg != NULL and msg->data.size == MOTORS_COUNT)
    {
        instance_ros_comunication->motorControllerFL->setTargetSpeed(msg->data.data[motor_left_front]);
        instance_ros_comunication->motorControllerFR->setTargetSpeed(msg->data.data[motor_right_front]);
        instance_ros_comunication->motorControllerNL->setTargetSpeed(msg->data.data[motor_left_rear]);
        instance_ros_comunication->motorControllerNR->setTargetSpeed(msg->data.data[motor_right_rear]);
    }
    Serial.print("Velocidad seteada motor left front ");
    Serial.println(msg->data.data[motor_left_front]);
    Serial.print("Velocidad seteada motor right front ");
    Serial.println(msg->data.data[motor_right_front]);
    Serial.print("Velocidad seteada motor left near ");
    Serial.println(msg->data.data[motor_left_rear]);
    Serial.print("Velocidad seteada motor right near ");
    Serial.println(msg->data.data[motor_right_rear]);
}

void RosCommunication::set_pwm_motors_callback(const void *msg_recv) {
    const agro_interfaces__msg__Motors *data = (const agro_interfaces__msg__Motors *)msg_recv;

    instance_ros_comunication->motorControllerFL->motor.setPwm(data->motor_fl);
    instance_ros_comunication->motorControllerFR->motor.setPwm(data->motor_fr);
    instance_ros_comunication->motorControllerNL->motor.setPwm(data->motor_nl);
    instance_ros_comunication->motorControllerNR->motor.setPwm(data->motor_nr);
    Serial.print("Pwm seteado motor FL ");
    Serial.println(data->motor_fl);
    Serial.print("Pwm seteado motor FR ");
    Serial.println(data->motor_fr);
    Serial.print("Pwm seteado motor NL ");
    Serial.println(data->motor_nl);
    Serial.print("Pwm seteado motor NR ");
    Serial.println(data->motor_nr);
}

void RosCommunication::config_params_callback(const void *msg_recv) {
    const agro_interfaces__msg__ConfigParams *data = (const agro_interfaces__msg__ConfigParams *)msg_recv;

    float kp = data->kp;
    float ki = data->ki;
    float kd = data->kd;

    instance_ros_comunication->motorControllerFL->pid.setTunings(kp,ki,kd);
    instance_ros_comunication->motorControllerFL->encoder.lowPassFilter->setParam(data->alpha);
    instance_ros_comunication->motorControllerFR->pid.setTunings(kp,ki,kd);
    instance_ros_comunication->motorControllerFR->encoder.lowPassFilter->setParam(data->alpha);
    instance_ros_comunication->motorControllerNL->pid.setTunings(kp,ki,kd);
    instance_ros_comunication->motorControllerNL->encoder.lowPassFilter->setParam(data->alpha);
    instance_ros_comunication->motorControllerNR->pid.setTunings(kp,ki,kd);
    instance_ros_comunication->motorControllerNR->encoder.lowPassFilter->setParam(data->alpha);

    Serial.print("Low pass filter set in ");
    Serial.println(data->alpha);
}



void RosCommunication::set_switch_control_callback(const void *msg_recv) {
    const std_msgs__msg__Float32 *data = (const std_msgs__msg__Float32 *)msg_recv;
    if(!data->data){

        instance_ros_comunication->motorControllerFL->disableControl();
        instance_ros_comunication->motorControllerFR->disableControl();
        instance_ros_comunication->motorControllerNL->disableControl();
        instance_ros_comunication->motorControllerNR->disableControl();
        Serial.println("Control desabilitado ");
    }
    else{
        instance_ros_comunication->motorControllerFL->enableControl();
        instance_ros_comunication->motorControllerFR->enableControl();
        instance_ros_comunication->motorControllerNL->enableControl();
        instance_ros_comunication->motorControllerNR->enableControl();
        Serial.println("Control habilitado ");
    }

}

void  RosCommunication::timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        pulse_counts_msg.motor_fl = instance_ros_comunication->motorControllerFL->encoder.getCount();
        pulse_counts_msg.motor_fr = instance_ros_comunication->motorControllerFR->encoder.getCount();
        pulse_counts_msg.motor_nl = instance_ros_comunication->motorControllerNL->encoder.getCount();
        pulse_counts_msg.motor_nr = instance_ros_comunication->motorControllerNR->encoder.getCount();
        RCSOFTCHECK(rcl_publish(&current_pulses_counts_pub, &pulse_counts_msg, NULL));
    }

}

void  RosCommunication::timer_callback_control(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);

    
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - instance_ros_comunication->lastTime_;
    
    instance_ros_comunication->lastTime_ = currentTime;

    //Serial.print("El delta de tiempo del control es ");
    //Serial.println(deltaTime);

    instance_ros_comunication->motorControllerFL->update();
    instance_ros_comunication->motorControllerFR->update();
    instance_ros_comunication->motorControllerNL->update();
    instance_ros_comunication->motorControllerNR->update();

    //motor FL
    float current_vel_FL =  instance_ros_comunication->motorControllerFL->encoder.getSpeed();
    float current_vel_filtered_FL =  instance_ros_comunication->motorControllerFL->encoder.getSpeedFiltered();
    float current_control_FL =  instance_ros_comunication->motorControllerFL->pid.getCurrentSignalControl();
    float current_error_FL  = instance_ros_comunication->motorControllerFL->pid.getCurrentSignalError();
    float setpoint_FL= instance_ros_comunication->motorControllerFL->pid.getSetpoint();

    motor_status_control_msg.current_speed.motor_fl = current_vel_FL;
    motor_status_control_msg.current_control.motor_fl = current_control_FL;
    motor_status_control_msg.current_error.motor_fl = current_error_FL;
    motor_status_control_msg.current_speed_filtered.motor_fl = current_vel_filtered_FL;
    motor_status_control_msg.setpoint.motor_fl= setpoint_FL;

    //motor FR
    float current_vel_FR =  instance_ros_comunication->motorControllerFR->encoder.getSpeed();
    float current_vel_filtered_FR =  instance_ros_comunication->motorControllerFR->encoder.getSpeedFiltered();
    float current_control_FR =  instance_ros_comunication->motorControllerFR->pid.getCurrentSignalControl();
    float current_error_FR  = instance_ros_comunication->motorControllerFR->pid.getCurrentSignalError();
    float setpoint_FR= instance_ros_comunication->motorControllerFR->pid.getSetpoint();

    motor_status_control_msg.current_speed.motor_fr = current_vel_FR;
    motor_status_control_msg.current_control.motor_fr = current_control_FR;
    motor_status_control_msg.current_error.motor_fr = current_error_FR;
    motor_status_control_msg.current_speed_filtered.motor_fr = current_vel_filtered_FR;
    motor_status_control_msg.setpoint.motor_fr = setpoint_FR;

    //motor NL
    float current_vel_NL =  instance_ros_comunication->motorControllerNL->encoder.getSpeed();
    float current_vel_filtered_NL =  instance_ros_comunication->motorControllerNL->encoder.getSpeedFiltered();
    float current_control_NL =  instance_ros_comunication->motorControllerNL->pid.getCurrentSignalControl();
    float current_error_NL  = instance_ros_comunication->motorControllerNL->pid.getCurrentSignalError();
    float setpoint_NL= instance_ros_comunication->motorControllerNL->pid.getSetpoint();

    motor_status_control_msg.current_speed.motor_nl = current_vel_NL;
    motor_status_control_msg.current_control.motor_nl = current_control_NL;
    motor_status_control_msg.current_error.motor_nl = current_error_NL;
    motor_status_control_msg.current_speed_filtered.motor_nl = current_vel_filtered_NL;
    motor_status_control_msg.setpoint.motor_nl= setpoint_NL;

    //motor NR
    float current_vel_NR =  instance_ros_comunication->motorControllerNR->encoder.getSpeed();
    float current_vel_filtered_NR =  instance_ros_comunication->motorControllerNR->encoder.getSpeedFiltered();
    float current_control_NR =  instance_ros_comunication->motorControllerNR->pid.getCurrentSignalControl();
    float current_error_NR  = instance_ros_comunication->motorControllerNR->pid.getCurrentSignalError();
    float setpoint_NR= instance_ros_comunication->motorControllerNR->pid.getSetpoint();

    motor_status_control_msg.current_speed.motor_nr = current_vel_NR;
    motor_status_control_msg.current_control.motor_nr = current_control_NR;
    motor_status_control_msg.current_error.motor_nr = current_error_NR;
    motor_status_control_msg.current_speed_filtered.motor_nr = current_vel_filtered_NR;
    motor_status_control_msg.setpoint.motor_nr = setpoint_NR;

    RCSOFTCHECK(rcl_publish(&control_state_pub, &motor_status_control_msg, NULL));
    instance_ros_comunication->updatWheelsStates();
    RCSOFTCHECK(rcl_publish(&wheels_state_pub, &wheels_state_msg, NULL));
}


void RosCommunication::start_receiving_msgs(){
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}

void RosCommunication::error_loop() {
    while (1) {
        Serial.println("Error");
        delay(100);
    }
}


void RosCommunication::updatWheelsStates(){
    float current_position[MOTORS_COUNT];
    current_position[motor_left_front] = instance_ros_comunication->motorControllerFL->encoder.getPosition();
    current_position[motor_right_front] = instance_ros_comunication->motorControllerFR->encoder.getPosition();
    current_position[motor_left_rear] = instance_ros_comunication->motorControllerNL->encoder.getPosition();
    current_position[motor_right_rear] = instance_ros_comunication->motorControllerNR->encoder.getPosition();
    float current_velocity[MOTORS_COUNT];
    current_velocity[motor_left_front] = instance_ros_comunication->motorControllerFL->encoder.getSpeedFiltered();
    current_velocity[motor_right_front] = instance_ros_comunication->motorControllerFR->encoder.getSpeedFiltered();
    current_velocity[motor_left_rear] = instance_ros_comunication->motorControllerNL->encoder.getSpeedFiltered();
    current_velocity[motor_right_rear] = instance_ros_comunication->motorControllerNR->encoder.getSpeedFiltered();
    for (auto i = 0u; i < MOTORS_COUNT; ++i)
    {   
        wheels_state_msg.position.data[i] = current_position[i];
        wheels_state_msg.velocity.data[i] = current_velocity[i];
    }
};

void RosCommunication::fill_wheels_state_msg(sensor_msgs__msg__JointState* msg)
{
  static double msg_data_tab[MOTORS_STATE_COUNT][MOTORS_COUNT];
  static rosidl_runtime_c__String msg_name_tab[MOTORS_COUNT];
  msg->header.frame_id = micro_ros_string_utilities_set(msg->header.frame_id, "wheels_state");

  msg->position.data = msg_data_tab[motor_state_position];
  msg->position.capacity = msg->position.size = MOTORS_COUNT;
  msg->velocity.data = msg_data_tab[motor_state_velocity];
  msg->velocity.capacity = msg->velocity.size = MOTORS_COUNT;
  msg->effort.data = msg_data_tab[motor_state_effort];
  msg->effort.capacity = msg->effort.size = MOTORS_COUNT;

  msg_name_tab->capacity = msg_name_tab->size = MOTORS_COUNT;
  msg_name_tab[motor_left_front].data = (char*)FRONT_LEFT_MOTOR_NAME;
  msg_name_tab[motor_right_front].data = (char*)FRONT_RIGHT_MOTOR_NAME;
  msg_name_tab[motor_left_rear].data = (char*)REAR_LEFT_MOTOR_NAME;
  msg_name_tab[motor_right_rear].data = (char*)REAR_RIGHT_MOTOR_NAME;
  for (uint8_t i = 0; i < MOTORS_COUNT; i++)
  {
    msg_name_tab[i].capacity = msg_name_tab[i].size = strlen(msg_name_tab[i].data);
  }
  msg->name.capacity = msg->name.size = MOTORS_COUNT;
  msg->name.data = msg_name_tab;

  if (rmw_uros_epoch_synchronized())
  {
    msg->header.stamp.sec = (int32_t)(rmw_uros_epoch_nanos() / 1000000000);
    msg->header.stamp.nanosec = (uint32_t)(rmw_uros_epoch_nanos() % 1000000000);
  }
}

void RosCommunication::fill_wheels_command_msg(std_msgs__msg__Float32MultiArray* msg)
{
  static float data[MOTORS_COUNT] = { 0, 0, 0, 0 };
  msg->data.capacity = MOTORS_COUNT;
  msg->data.size = 0;
  msg->data.data = (float*)data;
}