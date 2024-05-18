#include "serial_communicator.h"

#include <set>
#include <sstream>
#include <string>

namespace {

inline void PrintInfo(const std::string& str) {
  static std::stringstream ss;
  ss.str("");
  ss << "[INFO]: " << str << std::endl;
  std::cerr << std::string(ss.str());
}

inline void PrintWarn(const std::string& str) {
  static std::stringstream ss;
  ss.str("");
  ss << "\033[0;33m" << "[WARN]: " + str + "\033[0m" << std::endl;
  std::cerr << std::string(ss.str());
}

}  // namespace

SerialCommunicator::SerialCommunicator(const std::string& portname,
                                       const int baud_rate)
    : io_service_(),
      timeout_(io_service_),
      seq_recv_(0),
      len_packet_recv_(0),
      flag_recv_packet_ready_(false),
      seq_send_(0),
      len_packet_send_(0),
      ready_to_send_(false) {
  // initialize the portname
  port_name_ = portname;

  // Check whether this baud rate is valid
  CheckSupportedBaudRate(baud_rate);

  // Try to open serial port.
  OpenSerialPort();
  PrintInfo("SerialCommunicator - port [" + port_name_ + "] is open.");

  // Run TX RX threads
  terminate_future_ = terminate_promise_.get_future();
  mutex_rx_ = std::make_shared<std::mutex>();
  mutex_tx_ = std::make_shared<std::mutex>();
  thread_rx_ = std::thread([this]() { ProcessRx(); });
  thread_tx_ = std::thread([this]() { ProcessTx(); });
}

// deconstructor
SerialCommunicator::~SerialCommunicator() {
  // Terminate signal .
  PrintInfo("SerialCommunicator - terminate signal is published.");
  terminate_promise_.set_value();

  // wait for TX & RX threads to terminate ...
  PrintInfo(" - waiting 1 second to join TX / RX threads ...");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  if (thread_rx_.joinable()) thread_rx_.join();
  PrintInfo(" -  RX thread joins successfully.");

  if (thread_tx_.joinable()) thread_tx_.join();
  PrintInfo(" -  TX thread joins successfully.");

  // Close the serial port.
  CloseSerialPort();
  PrintInfo(" - Program is terminated.");
}

bool SerialCommunicator::IsNewPacketReceived() {
  return flag_recv_packet_ready_;
}

int SerialCommunicator::GetPacket(unsigned char* data) {
  int len = 0;
  mutex_rx_->lock();
  if (len_packet_recv_ > 0) {
    len = len_packet_recv_;
    for (int i = 0; i < len; ++i) data[i] = packet_recv_[i];
    len_packet_recv_ = 0;
    flag_recv_packet_ready_ = false;
  }
  mutex_rx_->unlock();

  return len;
}

bool SerialCommunicator::SendPacket(unsigned char* data, int length) {
  if (length == 0) return false;

  // update message & length
  mutex_tx_->lock();
  len_packet_send_ = length;
  for (int i = 0; i < length; ++i) packet_send_[i] = data[i];
  ready_to_send_ = true;  // Flag up!
  mutex_tx_->unlock();
  return true;
}

SerialCommunicator::RxStatistics SerialCommunicator::GetRxStatistics() {
  rx_stats_.num_total = seq_recv_;
  return rx_stats_;
}

SerialCommunicator::TxStatistics SerialCommunicator::GetTxStatistics() {
  tx_stats_.num_total = seq_send_;
  return tx_stats_;
}

void SerialCommunicator::SetBaudRate(const int baud_rate) {
  CheckSupportedBaudRate(baud_rate);

  boost::asio::serial_port_base::baud_rate baud_rate_option2(baud_rate);
  serial_port_->set_option(baud_rate_option2);
  boost::asio::serial_port_base::baud_rate baud_rate_option3;
  serial_port_->get_option(baud_rate_option3);
  std::cout
      << "SerialCommunicator - baudrate is changed from 115200 (default) to "
      << baud_rate_option3.value() << std::endl;
}

void SerialCommunicator::CheckSupportedBaudRate(const int baud_rate) {
  static std::set<int> supported_baud_rate_set{
      57600,   115200,  230400,  460800,  500000,  576000,  921600, 1000000,
      1152000, 1500000, 2000000, 2500000, 3000000, 3500000, 4000000};
  if (supported_baud_rate_set.find(baud_rate) != supported_baud_rate_set.end())
    baud_rate_ = baud_rate;
  else
    throw std::runtime_error("SerialCommunicator - Unsupported baudrate...");
}

void SerialCommunicator::OpenSerialPort() {
  PrintInfo("SerialCommunicator - opening the serial port...");
  serial_port_ = new boost::asio::serial_port(io_service_);

  try {
    serial_port_->open(port_name_);
  } catch (boost::system::system_error& error) {
    PrintInfo("SerialCommunicator - port [" + port_name_ +
              "] cannot be opened. Error message:" + error.what());
    throw std::runtime_error(error.what());
  }

  // If serial port cannot be opened,
  if (!serial_port_->is_open()) {
    PrintInfo("SerialCommunicator - [" + port_name_ +
              "] is not opened. terminate the node.");
    throw std::runtime_error("terminate the node");
  }

  // Set serial port spec.
  // No flow control, 8bit / no parity / stop bit 1
  boost::asio::serial_port_base::baud_rate baud_rate_option(baud_rate_);
  boost::asio::serial_port_base::flow_control flow_control(
      boost::asio::serial_port_base::flow_control::none);
  boost::asio::serial_port_base::parity parity(
      boost::asio::serial_port_base::parity::none);
  boost::asio::serial_port_base::stop_bits stop_bits(
      boost::asio::serial_port_base::stop_bits::one);

  serial_port_->set_option(baud_rate_option);
  serial_port_->set_option(flow_control);
  serial_port_->set_option(parity);
  serial_port_->set_option(stop_bits);
}

void SerialCommunicator::CloseSerialPort() {
  serial_port_->close();
  PrintInfo("SerialCommunicator - portname [" + port_name_ + +"] is closed...");
}

void SerialCommunicator::ProcessRx() {
  // Initialize
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  stacked_length_ = 0;
  boost::system::error_code error_code;
  while (true) {
    // Try to read serial port
    const int received_packet_length = serial_port_->read_some(
        boost::asio::buffer(buffer_recv_, BUF_SIZE), error_code);
    if (error_code == boost::system::errc::interrupted) {
      continue;  // error code 4. Interrupted... 괜찮을걸? ㅋㅋ
    } else if (error_code == boost::system::errc::no_such_file_or_directory) {
      PrintWarn(" - serial port might be disconnected. error code : " +
                std::to_string(error_code.value()));
    } else if (error_code) {
      // Error code는 boost::system::errc 에서 찾으면 된다.
      PrintWarn(" - serial_->read_some(): error code : " +
                std::to_string(error_code.value()));
    }

    bool do_stack = false;
    bool is_DLE_found = false;
    if (received_packet_length > 0) {  // There is data
      for (int i = 0; i < received_packet_length; ++i) {
        unsigned char c = buffer_recv_[i];

        if (do_stack) {        // 현재 Packet stack 중...
          if (is_DLE_found) {  // 1) DLE+DLE / 2) DLE+ETX
            if (c == DLE) {    // 1) DLE+DLE --> 실제데이터가 DLE
              is_DLE_found = false;

              packet_stack_[stacked_length_] = c;
              ++stacked_length_;
            } else if (c == ETX) {  // 2) DLE+ETX
              do_stack = false;
              is_DLE_found = false;

              // Check CRC16-CCITT
              Union<unsigned short> crc16_calc;
              crc16_calc.value =
                  GetChecksumCRC16CCITT(packet_stack_, stacked_length_ - 2);

              Union<unsigned short> crc16_recv;
              crc16_recv.bytes[0] = packet_stack_[stacked_length_ - 2];
              crc16_recv.bytes[1] = packet_stack_[stacked_length_ - 1];

              if (crc16_calc.value == crc16_recv.value) {
                ++seq_recv_;
                // Packet END. Copy the packet.
                mutex_rx_->lock();
                len_packet_recv_ = stacked_length_ - 2;
                for (int j = 0; j < len_packet_recv_; ++j)
                  packet_recv_[j] = packet_stack_[j];
                mutex_rx_->unlock();
                flag_recv_packet_ready_ = true;
              } else {
                ++rx_stats_.num_crc_error;
                PrintWarn(" - CRC ERROR ! seq: " + std::to_string(seq_recv_) +
                          ", crc: " + std::to_string(rx_stats_.num_crc_error) +
                          ", ofl: " + std::to_string(rx_stats_.num_overflow) +
                          ", ect:" + std::to_string(rx_stats_.num_exception));
                len_packet_recv_ = 0;
                flag_recv_packet_ready_ = false;
              }
              stacked_length_ = 0;
            } else {
              ++rx_stats_.num_exception;
              do_stack = false;
              is_DLE_found = false;
              stacked_length_ = 0;
              PrintWarn(" - Exception ! DLE but no ETX! seq: " +
                        std::to_string(seq_recv_) +
                        ", crc: " + std::to_string(rx_stats_.num_crc_error) +
                        ", ofl: " + std::to_string(rx_stats_.num_overflow) +
                        ", ect: " + std::to_string(rx_stats_.num_exception));
            }
          } else {           // 이전에 DLE가 발견되지 않았다.
            if (c == DLE) {  // DLE발견
              is_DLE_found = true;
            } else {  // 스택.
              packet_stack_[stacked_length_++] = c;
              if (stacked_length_ >= 256) {  // wierd error...
                for (int kk = 0; kk < stacked_length_; ++kk)
                  std::cout << (int)packet_stack_[kk] << " ";
                std::cout << "\n";
                do_stack = false;
                is_DLE_found = false;
                stacked_length_ = 0;
                ++rx_stats_.num_overflow;
                PrintWarn(" - RX STACK OVERFLOW   ! seq: " +
                          std::to_string(seq_recv_) +
                          ", crc: " + std::to_string(rx_stats_.num_crc_error) +
                          ", ofl: " + std::to_string(rx_stats_.num_overflow) +
                          ", ect:" + std::to_string(rx_stats_.num_exception));
              }
              // std::cout << (int)buf_recv_[i] << std::endl;
            }
          }
        } else {  // flagStacking == false    아직 STX를 발견하지 못함. (Stack
                  // 하지않음)
          if (is_DLE_found) {  // 이전에 DLE나옴.
            if (c == STX) {  // STX 찾음, 새로운 packet을 stack 시작함.
              do_stack = true;
              stacked_length_ = 0;
            }
            is_DLE_found = false;
          } else {
            is_DLE_found = (c == DLE) ? true : false;
          }
        }
      }
    }

    std::future_status terminate_status =
        terminate_future_.wait_for(std::chrono::microseconds(10));
    if (terminate_status == std::future_status::ready) break;
  }
  PrintInfo("SerialCommunicator - RX thread receives termination signal.");
}

void SerialCommunicator::ProcessTx() {
  while (true) {
    if (ready_to_send_) {
      if (mutex_tx_->try_lock()) {
        ready_to_send_ = false;

        SendPacketWithChecksum(packet_send_, len_packet_send_);
        len_packet_send_ = 0;
        ++seq_send_;
        mutex_tx_->unlock();
      }
    }

    std::future_status terminate_status =
        terminate_future_.wait_for(std::chrono::microseconds(50));
    if (terminate_status == std::future_status::ready) {
      // Zero send.
      Union<unsigned short> pwm;
      pwm.value = 0;

      PrintInfo(
          "=============== Catch terminate signal in TX thread "
          "===============");
      PrintInfo("    Send the zero signals to all PWM channels for safety.");

      mutex_tx_->lock();
      for (int i = 0; i < 8; ++i) {
        packet_send_[2 * i] = pwm.bytes[0];
        packet_send_[2 * i + 1] = pwm.bytes[1];
      }
      mutex_tx_->unlock();

      PrintInfo(" Send zero signal ... trial 1...");
      mutex_tx_->lock();
      SendPacketWithChecksum(packet_send_, 16);
      mutex_tx_->unlock();
      PrintInfo("  Wait 1 second...");
      sleep(1);

      PrintInfo(" Send zero signal ... trial 2...");
      mutex_tx_->lock();
      SendPacketWithChecksum(packet_send_, 16);
      mutex_tx_->unlock();

      PrintInfo("Zero signal done.");
      break;
    }
  }
  PrintInfo("SerialCommunicator - TX thread receives termination signal.");
}

void SerialCommunicator::SendPacketWithChecksum(const unsigned char* data,
                                                int len) {
  Union<unsigned short> crc16_calc;
  crc16_calc.value = CalculateChecksumCRC16CCITT(data, len);

  int idx = 2;
  buffer_send_[0] = DLE;
  buffer_send_[1] = STX;  // DLE, STX --> start of the packet
  for (int i = 0; i < len; ++i) {
    if (data[i] == DLE) {
      buffer_send_[idx++] = DLE;
      buffer_send_[idx++] = DLE;
    } else {
      buffer_send_[idx++] = data[i];
    }
  }

  // len = idx - 2;
  if (crc16_calc.bytes[0] == DLE) {
    buffer_send_[idx++] = DLE;
    buffer_send_[idx++] = DLE;
  } else {
    buffer_send_[idx++] = crc16_calc.bytes[0];
  }

  if (crc16_calc.bytes[1] == DLE) {
    buffer_send_[idx++] = DLE;
    buffer_send_[idx++] = DLE;
  } else {
    buffer_send_[idx++] = crc16_calc.bytes[1];
  }

  buffer_send_[idx++] = DLE;
  buffer_send_[idx++] = ETX;  // DLE, ETX --> end of the packet.

  // std::cout << "send data: ";
  // for(int i = 0; i < idx; ++i){
  //     std::cout << (int)buf_send_[i] <<" ";
  // }
  // std::cout << "\n";
  serial_port_->write_some(boost::asio::buffer(buffer_send_, idx));
  // boost::asio::write(*serial_, boost::asio::buffer(buf_send_, idx));
  // std::cout << "write length : " << len +6 << std::endl;
}

unsigned short SerialCommunicator::GetChecksumCRC16CCITT(
    const unsigned char* data, const int length) {
  return CalculateChecksumCRC16CCITT(data, length);
}

// static std::string MakeRed(const std::string& str) {
//   return {"\033[0;31m" + str + "\033[0m"};
// }

// static std::string MakeGreen(const std::string& str) {
//   return {"\033[0;32m" + str + "\033[0m"};
// }

// static std::string MakeYellow(const std::string& str) {
//   return {"\033[0;33m" + str + "\033[0m"};
// }

// static std::string MakeBlue(const std::string& str) {
//   return {"\033[0;34m" + str + "\033[0m"};
// }

// static std::string MakeMagenta(const std::string& str) {
//   return {"\033[0;35m" + str + "\033[0m"};
// }

// static std::string MakeCyan(const std::string& str) {
//   return {"\033[0;36m" + str + "\033[0m"};
// }