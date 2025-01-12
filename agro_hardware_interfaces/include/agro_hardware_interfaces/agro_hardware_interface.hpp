#ifndef AGRO_HARDWARE_INTERFACE_HPP
#define AGRO_HARDWARE_INTERFACE_HPP

#include "agro_hardware_interfaces/visibility_control.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/executor.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/handle.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"



namespace agro_hardware_interfaces{
using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

using Float32MultiArray = std_msgs::msg::Float32MultiArray;
using JointState = sensor_msgs::msg::JointState;

class AgroSystem: public hardware_interface::SystemInterface{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(AgroSystem)

    AGRO_HARDWARE_INTERFACES_PUBLIC 
    CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;

    AGRO_HARDWARE_INTERFACES_PUBLIC
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    AGRO_HARDWARE_INTERFACES_PUBLIC
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    AGRO_HARDWARE_INTERFACES_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    AGRO_HARDWARE_INTERFACES_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    AGRO_HARDWARE_INTERFACES_PUBLIC
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

    AGRO_HARDWARE_INTERFACES_PUBLIC
    CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;


    AGRO_HARDWARE_INTERFACES_PUBLIC
    std::vector<StateInterface> export_state_interfaces() override;

    AGRO_HARDWARE_INTERFACES_PUBLIC
    std::vector<CommandInterface> export_command_interfaces() override;

    AGRO_HARDWARE_INTERFACES_PUBLIC
    return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    AGRO_HARDWARE_INTERFACES_PUBLIC
    return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;


protected:

    std::map<std::string, double> vel_commands_;
    std::map<std::string, double> vel_state_;
    std::map<std::string, double> pos_state_;
    uint connection_timeout_ms_;
    uint connection_check_period_ms_;

    std::vector<std::string> velocity_command_joint_order_;

    std::shared_ptr<rclcpp::Node> node_;
    
    rclcpp::executors::MultiThreadedExecutor executor_;
    std::unique_ptr<std::thread> executor_thread_;

    std::shared_ptr<rclcpp::Publisher<Float32MultiArray>> motor_command_publisher_ = nullptr;
    std::shared_ptr<realtime_tools::RealtimePublisher<Float32MultiArray>> realtime_motor_command_publisher_ = nullptr;
    realtime_tools::RealtimeBox<std::shared_ptr<JointState>> received_motor_state_msg_ptr_{ nullptr };
    rclcpp::Subscription<JointState>::SharedPtr motor_state_subscriber_= nullptr;


    void motor_state_cb(const std::shared_ptr<JointState> msg);
    void cleanup_node();

};

}
#endif