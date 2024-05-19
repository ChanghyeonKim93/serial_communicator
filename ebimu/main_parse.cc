#include <chrono>
#include <queue>
#include <string>

#include "Eigen/Dense"

#include "../src/numerical_integrator/numerical_integrator.h"
#include "../src/serial_communicator.h"

#define D2R 3.14159265358979323846 / 180.0

struct ImuState {
  double time;
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  Eigen::Matrix<double, 10, 1> GetVector() {
    Eigen::Matrix<double, 10, 1> vec{Eigen::Matrix<double, 10, 1>::Zero()};
    vec.block<3, 1>(0, 0) = p;
    vec.block<3, 1>(3, 0) = v;
    vec(6, 0) = q.w();
    vec(7, 0) = q.x();
    vec(8, 0) = q.y();
    vec(9, 0) = q.z();
    return vec;
  }
};

ImuState GetDerivative(const ImuState& current_state,
                       const Eigen::Vector3d& linear_acc,
                       const Eigen::Vector3d& angular_vel) {
  // x(t+dt) = x(t) + dt * f(t, x(t));
  ImuState derivative;
  derivative.p = current_state.v;
  derivative.v = current_state.q * linear_acc + Eigen::Vector3d(0, 0, -9.81);
  Eigen::Quaterniond two_dq =
      current_state.q * Eigen::Quaterniond(0.0, angular_vel.x(),
                                           angular_vel.y(), angular_vel.z());
  derivative.q.w() = 0.5 * two_dq.w();
  derivative.q.x() = 0.5 * two_dq.x();
  derivative.q.y() = 0.5 * two_dq.y();
  derivative.q.z() = 0.5 * two_dq.z();

  return derivative;
}

ImuState IntegrateEuler(const ImuState& current_state,
                        const Eigen::Vector3d& linear_acc,
                        const Eigen::Vector3d& angular_vel, const double dt) {
  ImuState state;
  auto k1 = GetDerivative(current_state, linear_acc, angular_vel);

  state.p = current_state.p + dt * k1.p;
  state.v = current_state.v + dt * k1.v;

  // Eigen::Quaterniond dq(k1.q.w() * dt, k1.q.x() * dt, k1.q.y() * dt,
  // k1.q.z() * dt);
  state.q = Eigen::Quaterniond(
      current_state.q.w() + k1.q.w() * dt, current_state.q.x() + k1.q.x() * dt,
      current_state.q.y() + k1.q.y() * dt, current_state.q.z() + k1.q.z() * dt);
  state.q.normalize();

  return state;
}

Eigen::Matrix<double, 10, 1> GetDerivativeMidpoint(
    const ImuState& current_state, const Eigen::Vector3d& linear_acc,
    const Eigen::Vector3d& angular_vel, const double dt) {
  // k1 = f(t,x(t));
  // x(t+dt) = x(t) + dt * f(t+1/2*dt, x(t)+1/2*dt*k1);
  (void)current_state;
  (void)linear_acc;
  (void)angular_vel;
  (void)dt;
  // Eigen::Matrix3d rmat = current_state.q.toRotationMatrix();

  // Eigen::Matrix<double, 10, 1> derivative;
  // derivative.block<3, 1>(0, 0) = current_state.v;
  // derivative.block<3, 1>(3, 0) =
  //     rmat * linear_acc + Eigen::Vector3d(0, 0, 9.81);
  // Eigen::Quaterniond two_dq =
  //     current_state.q * Eigen::Quaterniond(0.0, angular_vel.x(),
  //                                          angular_vel.y(), angular_vel.z());
  // derivative.block<4, 1>(6, 0) =
  //     0.5 * Eigen::Vector4d(two_dq.w(), two_dq.x(), two_dq.y(), two_dq.z());

  return {};
}

Eigen::Matrix<double, 10, 1> GetDerivativeRK4(
    const ImuState& current_state, const Eigen::Vector3d& linear_acc,
    const Eigen::Vector3d& angular_vel, const double dt) {
  // k1 = f(t         , x(t));
  // k2 = f(t + 1/2*dt, x(t) + 1/2*dt*k1);
  // k3 = f(t + 1/2*dt, x(t) + 1/2*dt*k2);
  // k4 = f(t +     dt, x(t) +     dt*k3);
  // x(t+dt) = x(t) + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
  (void)current_state;
  (void)linear_acc;
  (void)angular_vel;
  (void)dt;
  // Eigen::Matrix3d rmat = current_state.q.toRotationMatrix();

  // Eigen::Matrix<double, 10, 1> derivative;
  // derivative.block<3, 1>(0, 0) = current_state.v;
  // derivative.block<3, 1>(3, 0) =
  //     rmat * linear_acc + Eigen::Vector3d(0, 0, 9.81);
  // Eigen::Quaterniond two_dq =
  //     current_state.q * Eigen::Quaterniond(0.0, angular_vel.x(),
  //                                          angular_vel.y(), angular_vel.z());
  // derivative.block<4, 1>(6, 0) =
  //     0.5 * Eigen::Vector4d(two_dq.w(), two_dq.x(), two_dq.y(), two_dq.z());

  return {};
}

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
    static bool init = false;
    for (const auto& c : str) {
      if (c == '*') {
        ss.str("");
        is_stx_found = true;
      } else if (c == '\r') {
        is_stx_found = false;
        auto imu_data = ParseString(ss.str());
        if (imu_data.time <= 0.0001) continue;
        std::cerr << "time: " << imu_data.time
                  << " /q: " << imu_data.quaternion[0] << ", "
                  << imu_data.quaternion[1] << ", " << imu_data.quaternion[2]
                  << ", " << imu_data.quaternion[3]
                  << " /a: " << imu_data.linear_acc[0] << ", "
                  << imu_data.linear_acc[1] << ", " << imu_data.linear_acc[2]
                  << " /w: " << imu_data.angular_vel[0] << ", "
                  << imu_data.angular_vel[1] << ", " << imu_data.angular_vel[2]
                  << " /m: " << imu_data.mag[0] << ", " << imu_data.mag[1]
                  << ", " << imu_data.mag[2] << "\n";
        std::cerr << "anorm: "
                  << std::sqrt(imu_data.linear_acc[0] * imu_data.linear_acc[0] +
                               imu_data.linear_acc[1] * imu_data.linear_acc[1] +
                               imu_data.linear_acc[2] * imu_data.linear_acc[2])
                  << "\n";
        imu_data_queue_.push_back(imu_data);

        if (!init) {
          imu_state_.time = imu_data.time;
          imu_state_.q = Eigen::Quaterniond(
              imu_data.quaternion[0], imu_data.quaternion[1],
              imu_data.quaternion[2], imu_data.quaternion[3]);
          std::cerr << "q: " << imu_data.quaternion[1] << ","
                    << imu_data.quaternion[2] << "," << imu_data.quaternion[3]
                    << "," << imu_data.quaternion[0] << " / "
                    << imu_state_.q.coeffs().transpose() << std::endl;

          init = true;
        } else {
          if (imu_data_queue_.size() > 1) {
            auto it = imu_data_queue_.rbegin();
            const double dt = it->time - (++it)->time;
            imu_state_ =
                IntegrateEuler(imu_state_, Eigen::Vector3d(imu_data.linear_acc),
                               Eigen::Vector3d(imu_data.angular_vel), dt);
            std::cerr << "p: " << imu_state_.p.transpose()
                      << " / v: " << imu_state_.v.transpose()
                      << " / q: " << imu_state_.q.coeffs().transpose()
                      << std::endl;
          }
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
    imu.angular_vel[0] = vals[4] * D2R;
    imu.angular_vel[1] = vals[5] * D2R;
    imu.angular_vel[2] = vals[6] * D2R;
    imu.linear_acc[0] = vals[7] * 9.81;
    imu.linear_acc[1] = vals[8] * 9.81;
    imu.linear_acc[2] = vals[9] * 9.81;
    imu.mag[0] = vals[10];
    imu.mag[1] = vals[11];
    imu.mag[2] = vals[12];

    return imu;
  }

 private:
  std::deque<ImuData> imu_data_queue_;

  ImuState imu_state_;
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