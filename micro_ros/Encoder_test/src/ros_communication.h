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


#include "encoder.h"
#include "pid.h"
#include "motor.h"
#include "motor_controller.h"




#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) { error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) {} }

class RosCommunication{
    public:
        RosCommunication(); // Constructor que acepta los parámetros del Encoder
        //RosCommunication();
        void initialize();

        void subscribers_define();
        void publishers_define();
        void timers_define();

        static void led_callback(const void *msg_recv);
        static void vel_callback(const void *msg_recv);
        static void state_callback(const void *msg_recv);

        static void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
        static void timer_callback_control(rcl_timer_t *timer, int64_t last_call_time);

        unsigned long lastTime_; // Último tiempo para cálculo de velocidad
        void start_receiving_msgs();
        void executors_start();

        void error_loop();

        MotorController* motorController;
    private:

    static float linear_vel;
    static float angular_vel;

};

// Error handle loop

#endif