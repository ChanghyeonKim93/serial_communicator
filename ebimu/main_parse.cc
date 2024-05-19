#include <string>

#include "../src/serial_communicator.h"

void Setting(SerialCommunicator& serial_comm) {
  int delay_in_us = 1000000;
  std::string message;
  // message = "<sb7>";  // 460800
  message.clear();
  message = "<stop>";
  serial_comm.SendPacket(message);
  usleep(delay_in_us);

  message.clear();
  message = "<ver>";
  serial_comm.SendPacket(message);
  usleep(delay_in_us);

  message.clear();
  message = "<cfg>";
  serial_comm.SendPacket(message);
  usleep(delay_in_us);

  message.clear();
  message = "<sor2>";
  serial_comm.SendPacket(message);
  usleep(delay_in_us);

  message.clear();
  message = "<sof2>";
  serial_comm.SendPacket(message);
  usleep(delay_in_us);

  message.clear();
  message = "<ssg4>";
  serial_comm.SendPacket(message);
  usleep(delay_in_us);

  message.clear();
  message = "<ssa4>";
  serial_comm.SendPacket(message);
  usleep(delay_in_us);

  message.clear();
  message = "<pons1>";
  serial_comm.SendPacket(message);
  usleep(delay_in_us);

  message.clear();
  message = "<sog1>";
  serial_comm.SendPacket(message);
  usleep(delay_in_us);

  message.clear();
  message = "<soa1>";
  serial_comm.SendPacket(message);
  usleep(delay_in_us);

  message.clear();
  message = "<som1>";
  serial_comm.SendPacket(message);
  usleep(delay_in_us);

  message.clear();
  message = "<start>";
  serial_comm.SendPacket(message);
  usleep(delay_in_us);
}

int main() {
  SerialCommunicator::Parameters parameters;
  // parameters.port_name = "/dev/ttyACM0";
  // parameters.baud_rate = 921600;
  // parameters.packet_type =
  //     SerialCommunicator::Parameters::PacketType::kFrameWithChecksum;

  parameters.port_name = "/dev/ttyUSB0";
  parameters.baud_rate = 460800;
  parameters.packet_type = SerialCommunicator::Parameters::PacketType::kRaw;
  SerialCommunicator serial_comm(parameters);

  // Setting(serial_comm);

  while (true) {
    auto str = serial_comm.GetRawPacket();
    if (!str.empty()) std::cerr << str;
    usleep(1000);
  }
  return 0;
}