#ifndef ROS_COMMUNICATION_H
#define ROS_COMMUNICATION_H

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/point.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "encoder.h"
#include "pid.h"
#include "motor.h"
#include "motor_controller.h"

#include "agro_interfaces/msg/control_status.h"
#include "agro_interfaces/msg/config_params.h"
#include "agro_interfaces/msg/motors.h"

#include "vector"

#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) { error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) {} }

// Motors setup
#define MOTOR_FR MOTOR1
#define MOTOR_FL MOTOR4
#define MOTOR_RR MOTOR2
#define MOTOR_RL MOTOR3

constexpr const char* FRONT_LEFT_MOTOR_NAME = "front_left_joint";
constexpr const char* FRONT_RIGHT_MOTOR_NAME = "front_right_joint";
constexpr const char* REAR_LEFT_MOTOR_NAME = "near_left_joint";
constexpr const char* REAR_RIGHT_MOTOR_NAME = "near_right_joint";

enum Motors
{
  motor_right_rear,
  motor_left_rear,
  motor_right_front,
  motor_left_front,
  MOTORS_COUNT
};

enum MotorsState
{
  motor_state_position,
  motor_state_velocity,
  motor_state_effort,
  MOTORS_STATE_COUNT
};



class RosCommunication{
    public:
        RosCommunication(); // Constructor que acepta los parámetros del Encoder
        //RosCommunication();
        void initialize();

        void subscribers_define();
        void publishers_define();
        void timers_define();

        static void led_callback(const void *msg_recv);
        static void set_vel_motors_callback(const void *msg_recv);
        static void set_switch_control_callback(const void *msg_recv);
        static void set_pwm_motors_callback(const void *msg_recv);
        static void config_params_callback(const void *msg_recv);

        static void  timer_callback(rcl_timer_t *timer, int64_t last_call_time);
        static void  timer_callback_control(rcl_timer_t *timer, int64_t last_call_time);

        unsigned long lastTime_; // Último tiempo para cálculo de velocidad
        void start_receiving_msgs();
        void executors_start();

        void error_loop();

        MotorController* motorControllerFL;
        MotorController* motorControllerFR;
        MotorController* motorControllerNL;
        MotorController* motorControllerNR;

        static void IRAM_ATTR isrA1();
        static void IRAM_ATTR isrB1();
        static void IRAM_ATTR isrA2();
        static void IRAM_ATTR isrB2();
        static void IRAM_ATTR isrA3();
        static void IRAM_ATTR isrB3();
        static void IRAM_ATTR isrA4();
        static void IRAM_ATTR isrB4();

        void updatWheelsStates();
        void fill_wheels_state_msg(sensor_msgs__msg__JointState* msg);
        void fill_wheels_command_msg(std_msgs__msg__Float32MultiArray* msg);
    private:

    static float linear_vel;
    static float angular_vel;

};

// Error handle loop

#endif