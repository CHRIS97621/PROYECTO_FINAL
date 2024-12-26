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
MotorController* instance1 = nullptr;
MotorController* instance2 = nullptr;

RosCommunication::RosCommunication(): motorController1(nullptr), motorController2(nullptr){
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

    //pinMode(ledPin,OUTPUT);
    /*MotorController(int pwm_pin, int dir_pin, int enable_pin, 
                                 int encoder_pin_a, int encoder_pin_b, 
                                 float kp, float ki, float kd)*/

    motorController1 = new MotorController(17, 4, 16, 0,   
                                           18, 19,    
                                           200.0, 70.0, 1.0); 
    motorController2 = new MotorController(13, 32, 33, 1,  
                                           26, 27,    
                                           200.0, 70.0, 1.0); 



    pinMode(18, INPUT_PULLUP);
    pinMode(19, INPUT_PULLUP);
    pinMode(26, INPUT_PULLUP);
    pinMode(27, INPUT_PULLUP);
    attachInterrupt(18, isrA, CHANGE); 
    attachInterrupt(19, isrB, CHANGE);
    attachInterrupt(26, isrC, CHANGE); 
    attachInterrupt(27, isrD, CHANGE);


    Serial.println("Encoder initialized");

    motorController1->init();
    instance1 = motorController1;
    motorController2->init();
    instance2 = motorController2;
}
// Función de interrupción para actualizar el contador de pulsos

void IRAM_ATTR RosCommunication::isrA() {
    if (instance1) {
        instance1->encoder.updateA();
    }
}

// ISR estática para el pin B
void IRAM_ATTR RosCommunication::isrB() {
    if (instance1) {
        instance1->encoder.updateB();
    }
}

void IRAM_ATTR RosCommunication::isrC() {
    if (instance2) {
        instance2->encoder.updateB();
    }
}

// ISR estática para el pin B
void IRAM_ATTR RosCommunication::isrD() {
    if (instance2) {
        instance2->encoder.updateA();
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
   
        instance_ros_comunication->motorController1->setTargetSpeed(msg->data.data[motor_left_front]);
        instance_ros_comunication->motorController2->setTargetSpeed(msg->data.data[motor_right_front]);
        instance_ros_comunication->motorController1->setTargetSpeed(msg->data.data[motor_left_rear]);
        instance_ros_comunication->motorController2->setTargetSpeed(msg->data.data[motor_right_rear]);
    }
    Serial.print("Velocidad seteada motor left_front ");
    Serial.println(msg->data.data[motor_left_front]);
    Serial.print("Velocidad seteada motor right front ");
    Serial.println(msg->data.data[motor_right_front]);
}

void RosCommunication::set_pwm_motors_callback(const void *msg_recv) {
    const agro_interfaces__msg__Motors *data = (const agro_interfaces__msg__Motors *)msg_recv;

    instance_ros_comunication->motorController1->motor.setPwm(data->motor1);
    instance_ros_comunication->motorController2->motor.setPwm(data->motor2);
    Serial.print("Pwm seteado motor1 ");
    Serial.println(data->motor1);
    Serial.print("Pwm seteado motor2 ");
    Serial.println(data->motor2);
}

void RosCommunication::config_params_callback(const void *msg_recv) {
    const agro_interfaces__msg__ConfigParams *data = (const agro_interfaces__msg__ConfigParams *)msg_recv;

    float kp = data->kp;
    float ki = data->ki;
    float kd = data->kd;
    instance_ros_comunication->motorController1->pid.setTunings(kp,ki,kd);
    instance_ros_comunication->motorController1->encoder.lowPassFilter->setParam(data->alpha);
    
    instance_ros_comunication->motorController2->pid.setTunings(kp,ki,kd);
    instance_ros_comunication->motorController2->encoder.lowPassFilter->setParam(data->alpha);

    Serial.print("Low pass filter set in ");
    Serial.println(data->alpha);
}



void RosCommunication::set_switch_control_callback(const void *msg_recv) {
    const std_msgs__msg__Float32 *data = (const std_msgs__msg__Float32 *)msg_recv;
    if(!data->data){

        instance_ros_comunication->motorController1->disableControl();
        instance_ros_comunication->motorController2->disableControl();
        Serial.println("Control desabilitado ");
    }
    else{
        instance_ros_comunication->motorController1->enableControl();
        instance_ros_comunication->motorController2->enableControl();
        Serial.println("Control habilitado ");
    }

}

void  RosCommunication::timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        pulse_counts_msg.motor1 = instance_ros_comunication->motorController1->encoder.getCount();
        pulse_counts_msg.motor2 = instance_ros_comunication->motorController2->encoder.getCount();
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

    //motor1
    instance_ros_comunication->motorController1->update();
    instance_ros_comunication->motorController2->update();

    float current_vel1 =  instance_ros_comunication->motorController1->encoder.getSpeed();
    float current_vel_filtered1 =  instance_ros_comunication->motorController1->encoder.getSpeedFiltered();
    float current_control1 =  instance_ros_comunication->motorController1->pid.getCurrentSignalControl();
    float current_error1  = instance_ros_comunication->motorController1->pid.getCurrentSignalError();
    float setpoint1= instance_ros_comunication->motorController1->pid.getSetpoint();

    motor_status_control_msg.current_speed.motor1 = current_vel1;
    motor_status_control_msg.current_control.motor1 = current_control1;
    motor_status_control_msg.current_error.motor1 = current_error1;
    motor_status_control_msg.current_speed_filtered.motor1 = current_vel_filtered1;
    motor_status_control_msg.setpoint.motor1 = setpoint1;

    //motor2
    float current_vel2 =  instance_ros_comunication->motorController2->encoder.getSpeed();
    float current_vel_filtered2 =  instance_ros_comunication->motorController2->encoder.getSpeedFiltered();
    float current_control2 =  instance_ros_comunication->motorController2->pid.getCurrentSignalControl();
    float current_error2  = instance_ros_comunication->motorController2->pid.getCurrentSignalError();
    float setpoint2= instance_ros_comunication->motorController2->pid.getSetpoint();

    motor_status_control_msg.current_speed.motor2 = current_vel2;
    motor_status_control_msg.current_control.motor2 = current_control2;
    motor_status_control_msg.current_error.motor2 = current_error2;
    motor_status_control_msg.current_speed_filtered.motor2 = current_vel_filtered2;
    motor_status_control_msg.setpoint.motor2 = setpoint2;

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
    current_position[motor_left_front] = instance_ros_comunication->motorController1->encoder.getPosition();
    current_position[motor_right_front] = instance_ros_comunication->motorController2->encoder.getPosition();
    current_position[motor_left_rear] = instance_ros_comunication->motorController1->encoder.getPosition();
    current_position[motor_right_rear] = instance_ros_comunication->motorController2->encoder.getPosition();
    float current_velocity[MOTORS_COUNT];
    current_velocity[motor_left_front] = instance_ros_comunication->motorController1->encoder.getSpeedFiltered();
    current_velocity[motor_right_front] = instance_ros_comunication->motorController2->encoder.getSpeedFiltered();
    current_velocity[motor_left_rear] = instance_ros_comunication->motorController1->encoder.getSpeedFiltered();
    current_velocity[motor_right_rear] = instance_ros_comunication->motorController2->encoder.getSpeedFiltered();
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