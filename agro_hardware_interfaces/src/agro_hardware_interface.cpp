#include "agro_hardware_interfaces/agro_hardware_interface.hpp"

#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/logging.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"


namespace agro_hardware_interfaces{
CallbackReturn AgroSystem::on_init(const hardware_interface::HardwareInfo& hardware_info)
{
  RCLCPP_INFO(rclcpp::get_logger("AgroSystem"), "Iniciando");
  //inserto el hardware info dentro de info_
  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS){

    return CallbackReturn::ERROR;
  }
  // extraigo todas las articulaciones del ahrdware info
  for(hardware_interface::ComponentInfo &joint : info_.joints){

    // Valido que cada joint tenga solo un comando de control
    if(joint.command_interfaces.size() != 1){
      RCLCPP_FATAL(rclcpp::get_logger("AgroSystem"), "Joint '%s' has %zu command interfaces found. 1 expected.",
                joint.name.c_str(), joint.command_interfaces.size());

      return CallbackReturn::ERROR; 
    }
    //valido ue la interface de control sea por velocidad
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("AgroSystem"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);

      return CallbackReturn::ERROR;
    }

    //valido ue la interface de estado sean 2
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(rclcpp::get_logger("AgroSystem"), "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());

      return CallbackReturn::ERROR;
    }

    //valido que la primera interfaz de estado sea la posicion
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger("AgroSystem"), "Joint '%s' have '%s' as first state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                   
      return CallbackReturn::ERROR;
    }

     //valido que la primera interfaz de estado sea la velocidad
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("AgroSystem"), "Joint '%s' have '%s' as second state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);

      return CallbackReturn::ERROR;
    }

  }
  for(auto& j : info_.joints){
    
    RCLCPP_INFO(rclcpp::get_logger("AgroSystem"), "Joint '%s' found", j.name.c_str());

    pos_state_[j.name] = 0.0;
    vel_state_[j.name] = 0.0;
    vel_commands_[j.name] = 0.0;
  }
  
  // tomo los parámetros definidos en el URDF
  connection_timeout_ms_ = std::stoul(info_.hardware_parameters["connection_timeout_ms"]);
  connection_check_period_ms_ = std::stoul(info_.hardware_parameters["connection_check_period_ms"]);


  std::string velocity_command_joint_order_raw = info_.hardware_parameters["velocity_command_joint_order"];

  // quito los espacios en blanco
  velocity_command_joint_order_raw.erase(
      std::remove_if(velocity_command_joint_order_raw.begin(), velocity_command_joint_order_raw.end(),
                     [](char c) { return std::isspace(static_cast<unsigned char>(c)); }),
      velocity_command_joint_order_raw.end());
  std::stringstream velocity_command_joint_order_stream(velocity_command_joint_order_raw);

  std::string joint_name;

  
  // inserto los comandos en un vector, respetando el orden defnido en los parámetros
  while (getline(velocity_command_joint_order_stream, joint_name, ','))
  {
    velocity_command_joint_order_.push_back(joint_name);
  }
  // Verifico el tamaño del vector con el tamaño del los joints definidos en URDF
  if (velocity_command_joint_order_.size() != info_.joints.size())
  {
    RCLCPP_FATAL(rclcpp::get_logger("AgroSystem"), "Joint order size is invalid");

    return CallbackReturn::ERROR;
  }


  for (auto& j : info_.joints)
  {
    if (std::find(velocity_command_joint_order_.begin(), velocity_command_joint_order_.end(), j.name) ==
        velocity_command_joint_order_.end())
    {
      RCLCPP_FATAL(rclcpp::get_logger("AgroSystem"), "Joint '%s' missing from velocity command joint order",
                   j.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  node_ = std::make_shared<rclcpp::Node>("agro_system_node");
  executor_.add_node(node_);

  executor_thread_ =
      std::make_unique<std::thread>(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor_));

  return CallbackReturn::SUCCESS;

}

CallbackReturn AgroSystem::on_configure(const rclcpp_lifecycle::State&){
  RCLCPP_INFO(rclcpp::get_logger("AgroSystem"), "Configuring");
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn AgroSystem::on_cleanup(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("AgroSystem"), "Cleaning up");

  return CallbackReturn::SUCCESS;
}

CallbackReturn AgroSystem::on_activate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("AgroSystem"), "Activating");
  
  // reinicio los valores 
  for (const auto& x : pos_state_)
  {
    pos_state_[x.first] = 0.0;
    vel_state_[x.first] = 0.0;
    vel_commands_[x.first] = 0.0;
  }

  motor_command_publisher_ = node_->create_publisher<Float32MultiArray>("~/motors_cmd",10);
  realtime_motor_command_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<Float32MultiArray>>(motor_command_publisher_);

  motor_state_subscriber_ =
      node_->create_subscription<JointState>("~/motors_response", rclcpp::SensorDataQoS(),
                                             std::bind(&AgroSystem::motor_state_cb, this, std::placeholders::_1));

  std::shared_ptr<JointState> motor_state;

  //reviso la conexión, esto se hace median la subscripción y espero el tiempo definido en hardware info
  for (uint wait_time = 0; wait_time <= connection_timeout_ms_; wait_time += connection_check_period_ms_){
    if (!rclcpp::ok())
    {
      return CallbackReturn::ERROR;
    }

    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("RosbotSystem"), *node_->get_clock(), 5000, "Feedback message from motors wasn't received yet");

    received_motor_state_msg_ptr_.get(motor_state);

    if (motor_state)
    {
      RCLCPP_DEBUG(node_->get_logger(), "Subscriber and publisher are now active.");

      return CallbackReturn::SUCCESS;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(connection_check_period_ms_));
  }

  RCLCPP_FATAL(node_->get_logger(), "Activation failed, timeout reached while waiting for feedback from motors");

  return CallbackReturn::ERROR;
}

// Funcion apra desativar el nodo
CallbackReturn AgroSystem::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("AgroSystem"), "Deactivating");
  cleanup_node();
  received_motor_state_msg_ptr_.set(nullptr);

  return CallbackReturn::SUCCESS;
}

CallbackReturn AgroSystem::on_shutdown(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Shutting down");
  cleanup_node();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AgroSystem::on_error(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("AgroSystem"), "Handling error");
  cleanup_node();
  return CallbackReturn::SUCCESS;
}

// exporto las interfaces de estado
std::vector<StateInterface> AgroSystem::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_state_[info_.joints[i].name]));
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_state_[info_.joints[i].name]));
  }

  return state_interfaces;
}

// exporto las interfaces de comando, le paso el puntero de mis Atributos a state_interface
std::vector<CommandInterface> AgroSystem::export_command_interfaces()
{
  std::vector<CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_commands_[info_.joints[i].name]));
  }

  return command_interfaces;
}

//libero los recursos
void AgroSystem::cleanup_node()
{
  motor_state_subscriber_.reset();
  realtime_motor_command_publisher_.reset();
  motor_command_publisher_.reset();
}

// callback para recibir el estado de los motores
void AgroSystem::motor_state_cb(const std::shared_ptr<JointState> msg)
{
  RCLCPP_DEBUG(node_->get_logger(), "Received motors response");
  received_motor_state_msg_ptr_.set(std::move(msg));
}

//funcion que almacena los estados de los motores
return_type AgroSystem::read(const rclcpp::Time&, const rclcpp::Duration&)
{
  std::shared_ptr<JointState> motor_state;
  received_motor_state_msg_ptr_.get(motor_state);

  RCLCPP_DEBUG(rclcpp::get_logger("AgroSystem"), "Reading motors state");

  if (!motor_state)
  {
    RCLCPP_ERROR(rclcpp::get_logger("AgroSystem"), "Feedback message from motors wasn't received");
    return return_type::ERROR;
  }

  for (auto i = 0u; i < motor_state->name.size(); i++)
  {
    if (pos_state_.find(motor_state->name[i]) == pos_state_.end() ||
        vel_state_.find(motor_state->name[i]) == vel_state_.end())
    {
      RCLCPP_ERROR(rclcpp::get_logger("AgroSystem"), "Position or velocity feedback not found for joint %s",
                   motor_state->name[i].c_str());
      return return_type::ERROR;
    }

    pos_state_[motor_state->name[i]] = motor_state->position[i];
    vel_state_[motor_state->name[i]] = motor_state->velocity[i];

    RCLCPP_DEBUG(rclcpp::get_logger("AgroSystem"), "Position feedback: %f, velocity feedback: %f",
                 pos_state_[motor_state->name[i]], vel_state_[motor_state->name[i]]);
  }
  return return_type::OK;
}

return_type AgroSystem::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  if (realtime_motor_command_publisher_->trylock())
  {
    auto& motor_command = realtime_motor_command_publisher_->msg_;
    motor_command.data.clear();

    RCLCPP_DEBUG(rclcpp::get_logger("AgroSystem"), "Wrtiting motors cmd message");

    for (auto const& joint : velocity_command_joint_order_)
    {
      motor_command.data.push_back(vel_commands_[joint]);
    }

    realtime_motor_command_publisher_->unlockAndPublish();
  }

  return return_type::OK;
}


}


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(agro_hardware_interfaces::AgroSystem, hardware_interface::SystemInterface)