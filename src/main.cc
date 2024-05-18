#include "serial_communicator.h"

int main() {
  SerialCommunicator::Parameters parameters;
  parameters.port_name = "/dev/ttyACM0";
  parameters.baud_rate = 921600;
  parameters.packet_type =
      SerialCommunicator::Parameters::PacketType::kFrameWithChecksum;
  SerialCommunicator serial_comm(parameters);

  while (true) {
    sleep(1);
  }
  return 0;
}