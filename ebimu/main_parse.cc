#include <chrono>
#include <queue>
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

class EbimuParser {
 public:
  struct ImuData {
    double time{0.0};
    double quaternion[4] = {1.0, 0.0, 0.0, 0.0};
    double angular_vel[3] = {0.0};
    double linear_acc[3] = {0.0};
    double mag[3] = {0.0};
  };

  EbimuParser() {}
  ~EbimuParser() {}

  void AppendRawMessage(const std::string& str) {
    static std::stringstream ss;
    static bool is_stx_found = false;
    for (const auto& c : str) {
      if (c == '*') {
        ss.str("");
        is_stx_found = true;
      } else if (c == '\r') {
        is_stx_found = false;
        auto imu_data = ParseString(ss.str());
        std::cerr << "time: " << imu_data.time << " / "
                  << imu_data.quaternion[0] << ", " << imu_data.quaternion[1]
                  << ", " << imu_data.quaternion[2] << ", "
                  << imu_data.quaternion[3] << " / " << imu_data.linear_acc[0]
                  << ", " << imu_data.linear_acc[1] << ", "
                  << imu_data.linear_acc[2] << " / " << imu_data.angular_vel[0]
                  << ", " << imu_data.angular_vel[1] << ", "
                  << imu_data.angular_vel[2] << " / " << imu_data.mag[0] << ", "
                  << imu_data.mag[1] << ", " << imu_data.mag[2] << "\n";

        if (imu_data_queue_.size() > 2) {
          auto it = imu_data_queue_.rbegin();
          std::cerr << "timediff : " << it->time - (++it)->time << std::endl;
        }
      } else {
        if (is_stx_found) ss << c;
      }
    }
  }

 private:
  ImuData ParseString(const std::string& str_data) {
    static constexpr char kDelimiter = ',';
    const char* str = str_data.data();
    const char* begin = str;
    std::vector<double> vals;
    do {
      begin = str;
      while (*str != kDelimiter && *str) str++;
      try {
        vals.push_back(std::stod(std::string(begin, str)));
      } catch (std::exception&) {
        return {};
      }
    } while (0 != *str++);

    ImuData imu;
    imu.time =
        std::chrono::steady_clock::now().time_since_epoch().count() * 1e-9;
    imu.quaternion[0] = vals[0];
    imu.quaternion[1] = vals[1];
    imu.quaternion[2] = vals[2];
    imu.quaternion[3] = vals[3];
    imu.angular_vel[0] = vals[4];
    imu.angular_vel[1] = vals[5];
    imu.angular_vel[2] = vals[6];
    imu.linear_acc[0] = vals[7];
    imu.linear_acc[1] = vals[8];
    imu.linear_acc[2] = vals[9];
    imu.mag[0] = vals[10];
    imu.mag[1] = vals[11];
    imu.mag[2] = vals[12];

    imu_data_queue_.push_back(imu);

    return imu;
  }

 private:
  std::deque<ImuData> imu_data_queue_;
};

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

  EbimuParser parser;

  // Setting(serial_comm);

  while (true) {
    auto str = serial_comm.GetRawPacket();
    parser.AppendRawMessage(str);
    // if (!str.empty()) std::cerr << str;
    usleep(1000);
  }
  return 0;
}