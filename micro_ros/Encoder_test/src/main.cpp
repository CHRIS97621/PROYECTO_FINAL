
#include "ros_communication.h"


RosCommunication ros_communication; // Definición global


void setup() {
  ros_communication.initialize();

  ros_communication.subscribers_define();
  ros_communication.publishers_define();
  ros_communication.timers_define();

  ros_communication.executors_start();
}

void loop() {
  ros_communication.start_receiving_msgs();
}
