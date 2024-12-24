#include "ros_communication.h"

rcl_subscription_t led_sub;
rcl_subscription_t vel_motor1_sub;
rcl_subscription_t state_motor_sub;

rcl_publisher_t count_pub;
rcl_publisher_t motor_control_pub;

std_msgs__msg__Bool led_msg;
std_msgs__msg__Int32 count_msg;
std_msgs__msg__Float32 vel_msg;
std_msgs__msg__Float32 state_msg;
geometry_msgs__msg__Point motor_control_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

rcl_node_t node;
rcl_timer_t timer;
rcl_timer_t timer_control;


#define ledPin 32



RosCommunication* instance_ros_comunication = nullptr; 

RosCommunication::RosCommunication(): motorController(nullptr){
    // Mensaje de depuración
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
    rclc_node_init_default(&node, "cmd_vel_sub", "", &support);

    pinMode(ledPin,OUTPUT);
    /*MotorController(int pwm_pin, int dir_pin, int enable_pin, 
                                 int encoder_pin_a, int encoder_pin_b, 
                                 float kp, float ki, float kd)*/

    motorController = new MotorController(17, 4, 16,   
                                           18, 19,    
                                           2.0, 0.5, 1.0); 

    
    motorController->init();
}


void RosCommunication::executors_start(){
  // Crear ejecutor
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &led_sub, &led_msg,   RosCommunication::led_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &vel_motor1_sub, &vel_msg,   RosCommunication::vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &state_motor_sub, &state_msg,   RosCommunication::state_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_control));

  Serial.println("Executors Started");
}


void RosCommunication::publishers_define() {
    // Crear publicador
    RCCHECK(rclc_publisher_init_default(
        &count_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "publish_count"));
    // Crear publicador
    RCCHECK(rclc_publisher_init_default(
        &motor_control_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point),
        "control_info"));

}

void RosCommunication::subscribers_define() {
    // Crear suscriptor led
    RCCHECK(rclc_subscription_init_default(
        &led_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/led_state"));

    // Crear suscriptor vel
    RCCHECK(rclc_subscription_init_default(
        &vel_motor1_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/vel_motor1"));
    RCCHECK(rclc_subscription_init_default(
        &state_motor_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/set_motor_state"));
}
void RosCommunication::timers_define() {
    // Crear temporizador
    Serial.println("5");
    const unsigned int timer_timeout = 1000; // Tiempo en milisegundos
    const unsigned int timer_timeout_control = 20; // Tiempo en milisegundos
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        &RosCommunication::timer_callback));
    Serial.println("6");

    RCCHECK(rclc_timer_init_default(
        &timer_control,
        &support,
        RCL_MS_TO_NS(timer_timeout_control),
        &RosCommunication::timer_callback_control));
        
}



void RosCommunication::led_callback(const void *msg_recv) {
    const std_msgs__msg__Bool *data = (const std_msgs__msg__Bool *)msg_recv;

    Serial.print("El estado del LED es: ");
    Serial.println(data->data ? "Encendido" : "Apagado");

    digitalWrite(ledPin,data->data);
}

void RosCommunication::vel_callback(const void *msg_recv) {
    const std_msgs__msg__Float32 *data = (const std_msgs__msg__Float32 *)msg_recv;
    instance_ros_comunication->motorController->setTargetSpeed(data->data);
    Serial.print("Velocidad seteada ");
    Serial.println(data->data);
    
}

void RosCommunication::state_callback(const void *msg_recv) {
    const std_msgs__msg__Float32 *data = (const std_msgs__msg__Float32 *)msg_recv;
    if(!data->data){
        instance_ros_comunication->motorController->motor.disable();
        instance_ros_comunication->motorController->setTargetSpeed(0.0);
        Serial.println("Motor desabilitado ");
    }
    else{
        instance_ros_comunication->motorController->motor.enable();
        Serial.println("Motor habilitado ");
    }

    
}

void RosCommunication::timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&count_pub, &count_msg, NULL));
        count_msg.data++;
    }
}

void RosCommunication::timer_callback_control(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);

    
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - instance_ros_comunication->lastTime_;
    
    instance_ros_comunication->lastTime_ = currentTime;

    Serial.print("El delta de tiempo es ");
    Serial.println(deltaTime);

    if(instance_ros_comunication->motorController->motor.enable_motor_){
        if (timer != NULL) {
            instance_ros_comunication->motorController->update();
            float current_vel = instance_ros_comunication->motorController->encoder.calculateSpeed();
            float current_control = instance_ros_comunication->motorController->pid.getCurrentSignalControl();
            float current_error = instance_ros_comunication->motorController->pid.getCurrentSignalError();

            motor_control_msg.x = current_vel;
            motor_control_msg.y = current_control;
            motor_control_msg.z = current_error;
            RCSOFTCHECK(rcl_publish(&motor_control_pub, &motor_control_msg, NULL));

        }
    }
    else{

        instance_ros_comunication->motorController->pid.clean();    
    }
}


void RosCommunication::start_receiving_msgs(){
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}

void RosCommunication::error_loop() {
    while (1) {
        Serial.println("Error");
        delay(100);
    }
}
