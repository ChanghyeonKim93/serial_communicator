#include "serial_communicator.h"

#include <set>
#include <sstream>
#include <string>

namespace {

inline void PrintInfo(const std::string& str) {
  static std::stringstream ss;
  ss.flush();
  ss << "[INFO]: " << str;
  std::cerr << std::string(ss.str() + "\n");
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
      flag_ready_to_send_(false),
      seq_recv_crc_error_(0),
      seq_recv_overflow_(0),
      seq_recv_exception_(0) {
  // initialize mutex
  mutex_rx_ = std::make_shared<std::mutex>();
  mutex_tx_ = std::make_shared<std::mutex>();

  // initialize the portname
  SetPortName(portname);

  // Check whether this baud rate is valid
  CheckSupportedBaudRate(baud_rate);

  // Try to open serial port.
  OpenSerialPort();

  // Run TX RX threads
  terminate_future_ = terminate_promise_.get_future();
  RunTxThread();
  RunRxThread();

  PrintInfo("SerialCommunicator - port [" + portname_ + "] is open.\n");
}

// deconstructor
SerialCommunicator::~SerialCommunicator() {
  // Terminate signal .
  PrintInfo("SerialCommunicator - terminate signal published...");
  terminate_promise_.set_value();

  // wait for TX & RX threads to terminate ...
  PrintInfo(
      "                   - waiting 1 second to join TX / RX threads ...");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  if (thread_rx_.joinable()) thread_rx_.join();
  PrintInfo("                   -   RX thread joins (turns off) successfully.");

  if (thread_tx_.joinable()) thread_tx_.join();
  PrintInfo("                   -   TX thread joins (turns off) successfully.");

  // Close the serial port.
  CloseSerialPort();
  PrintInfo("SerialCommunicator - terminated.");
}

bool SerialCommunicator::IsPacketReady() {
  bool res = false;
  mutex_rx_->lock();
  res = flag_recv_packet_ready_;
  mutex_rx_->unlock();
  return res;
}

int SerialCommunicator::GetPacket(unsigned char* buf) {
  int len = 0;
  mutex_rx_->lock();
  if (len_packet_recv_ > 0) {
    len = len_packet_recv_;
    for (int i = 0; i < len; ++i) buf[i] = packet_recv_[i];

    // Initialize the flag
    len_packet_recv_ = 0;
    flag_recv_packet_ready_ = false;
  }
  mutex_rx_->unlock();

  // return len > 0 when data ready.
  // else 0.
  return len;
}

bool SerialCommunicator::SendPacket(unsigned char* buf, int len) {
  bool isOK = len > 0;

  // update message & length
  mutex_tx_->lock();

  len_packet_send_ = len;
  for (int i = 0; i < len; ++i) packet_send_[i] = buf[i];

  flag_ready_to_send_ = true;  // Flag up!
  mutex_tx_->unlock();
  return isOK;
}

void SerialCommunicator::GetRxStatistics(uint32_t& seq, uint32_t& seq_crcerr,
                                         uint32_t& seq_overflowerr,
                                         uint32_t& seq_exceptionerr) {
  seq = seq_recv_;
  seq_crcerr = seq_recv_crc_error_;
  seq_overflowerr = seq_recv_overflow_;
  seq_exceptionerr = seq_recv_exception_;
}

void SerialCommunicator::GetTxStatistics(uint32_t& seq) { seq = seq_send_; }

void SerialCommunicator::SetPortName(const std::string& portname) {
  portname_ = portname;
}

void SerialCommunicator::SetBaudRate(const int baud_rate) {
  CheckSupportedBaudRate(baud_rate);

  boost::asio::serial_port_base::baud_rate baud_rate_option2(baud_rate);
  serial_->set_option(baud_rate_option2);
  boost::asio::serial_port_base::baud_rate baud_rate_option3;
  serial_->get_option(baud_rate_option3);
  std::cout
      << "SerialCommunicator - baudrate is changed from 115200 (default) to "
      << baud_rate_option3.value() << std::endl;
}

void SerialCommunicator::CheckSupportedBaudRate(const int baud_rate) {
  static std::set<int> supported_baud_rate_set{
      57600,   115200,  230400,  460800,  500000,  576000,  921600, 1000000,
      1152000, 1500000, 2000000, 2500000, 3000000, 3500000, 4000000};
  if (supported_baud_rate_set.find(baud_rate) !=
      supported_baud_rate_set.end()) {
    baud_rate_ = baud_rate;
  } else {
    throw std::runtime_error("SerialCommunicator - Unsupported baudrate...");
  }
}

void SerialCommunicator::OpenSerialPort() {
  PrintInfo("SerialCommunicator - opening the serial port...");
  serial_ = new boost::asio::serial_port(io_service_);

  // Try to open the serial port
  try {
    serial_->open(portname_);
  } catch (boost::system::system_error& error) {
    PrintInfo("SerialCommunicator - port [" + portname_ +
              "] cannot be opened. Error message:" + error.what());
    throw std::runtime_error(error.what());
  }

  // If serial port cannot be opened,
  if (!serial_->is_open()) {
    PrintInfo("SerialCommunicator - [" + portname_ +
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

  serial_->set_option(baud_rate_option);
  serial_->set_option(flow_control);
  serial_->set_option(parity);
  serial_->set_option(stop_bits);
}

void SerialCommunicator::CloseSerialPort() {
  serial_->close();
  PrintInfo("SerialCommunicator - portname [" + portname_ + +"] is closed...");
}

void SerialCommunicator::RunRxThread() {
  thread_rx_ = std::thread([this]() { processRX(terminate_future_); });
}

void SerialCommunicator::RunTxThread() {
  thread_tx_ = std::thread([this]() { processTX(terminate_future_); });
}

void SerialCommunicator::processRX(std::shared_future<void> terminate_signal) {
  // Initialize
  bool flagStacking = false;
  bool flagDLEFound = false;

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  len_stack_ = 0;
  timer::tic();
  boost::system::error_code ec;
  while (true) {
    // Try to read serial port
    int len_read =
        serial_->read_some(boost::asio::buffer(buf_recv_, BUF_SIZE), ec);
    if (ec == boost::system::errc::interrupted) {
      // error code 4. Interrupted... 괜찮을걸? ㅋㅋ
      continue;
    } else if (ec == boost::system::errc::no_such_file_or_directory) {
      std::cout
          << "WARNING    ! - serial port might be disconnected... error code : "
          << ec << std::endl;
    } else if (ec) {
      // Error code는 boost::system::errc 에서 찾으면 된다.
      std::cout << "WARNING    ! - serial_->read_some(): error code : " << ec
                << std::endl;
    }

    if (len_read > 0) {  // There is data
      // std::cout << "get new : " << len_read << std::endl;
      for (int i = 0; i < len_read; ++i) {
        unsigned char c = buf_recv_[i];
        // std::cout << "stkidx : " << len_stack_ <<", char:"<< (int)c <<
        // std::endl;

        if (flagStacking) {    // 현재 Packet stack 중...
          if (flagDLEFound) {  // 1) DLE, DLE / 2) DLE, ETX
            if (c == DLE) {    // 1) DLE, DLE --> 실제데이터가 DLE
              flagDLEFound = false;

              packet_stack_[len_stack_] = c;
              ++len_stack_;
            } else if (c == ETX) {  // 2) DLE, ETX
              flagStacking = false;
              flagDLEFound = false;

              // Check CRC16-CCITT
              USHORT_UNION crc16_calc;
              crc16_calc.ushort_ =
                  stringChecksumCRC16_CCITT(packet_stack_, 0, len_stack_ - 3);

              USHORT_UNION crc16_recv;
              crc16_recv.bytes_[0] = packet_stack_[len_stack_ - 2];
              crc16_recv.bytes_[1] = packet_stack_[len_stack_ - 1];

              // std::cout << " crc check: " << crc16_recv.ushort_ <<", " <<
              // crc16_calc.ushort_ << std::endl;

              if (crc16_calc.ushort_ == crc16_recv.ushort_) {
                // CRC test OK!
                //  std::cout <<" CRC OK!!!  success: " << seq_recv_ <<", crc
                //  err: " << seq_recv_crc_error_ <<", overflow err: " <<
                //  seq_recv_overflow_ <<"\n";
                ++seq_recv_;

                // Packet END. Copy the packet.
                // std::cout << "== " << timer::toc(0) << "ms, ETX found. seq: "
                // << seq_recv_ << ", length: " << len_stack_-2 << std::endl;
                mutex_rx_->lock();
                len_packet_recv_ = len_stack_ - 2;
                // std::cout << " recved contents: ";
                for (int j = 0; j < len_packet_recv_; ++j)
                  packet_recv_[j] = packet_stack_[j];
                // std::cout << std::endl;

                mutex_rx_->unlock();
                flag_recv_packet_ready_ = true;
                // std::cout << " flag packet ready?: " <<
                // flag_recv_packet_ready_ << std::endl;
              } else {
                std::cout << "WARNING    ! - CRC ERROR !\n" << std::endl;
                std::cout << "CRC ERR    ! seq: " << seq_recv_
                          << ", crc: " << seq_recv_crc_error_
                          << ", ofl: " << seq_recv_overflow_
                          << ", ect:" << seq_recv_exception_ << "\n";
                for (int j = 0; j < len_stack_; ++j)
                  std::cout << (int)packet_stack_[j] << " ";
                std::cout << std::endl;

                ++seq_recv_crc_error_;
                len_packet_recv_ = 0;
                flag_recv_packet_ready_ = false;
              }

              len_stack_ = 0;
            } else {
              // exceptional case
              ++seq_recv_exception_;
              flagStacking = false;
              flagDLEFound = false;
              len_stack_ = 0;

              PrintInfo(
                  "WARNING    ! - While stacking, DLE is found, but "
                  "there is no ETX.");
              PrintInfo("DLE,no ETX ! seq: " + std::to_string(seq_recv_) +
                        ", crc: " + std::to_string(seq_recv_crc_error_) +
                        ", ofl: " + std::to_string(seq_recv_overflow_) +
                        ", ect:" + std::to_string(seq_recv_exception_));
            }
          } else {           // 이전에 DLE가 발견되지 않았다.
            if (c == DLE) {  // DLE발견
              flagDLEFound = true;
            } else {  // 스택.
              packet_stack_[len_stack_] = c;
              ++len_stack_;
              if (len_stack_ >= 64) {  // wierd error...
                for (int kk = 0; kk < len_stack_; ++kk)
                  std::cout << (int)packet_stack_[kk] << " ";
                std::cout << "\n";
                flagStacking = false;
                flagDLEFound = false;
                len_stack_ = 0;
                ++seq_recv_overflow_;
                std::cout << "WARNING    ! - RX STACK OVER FLOW!\n"
                          << std::endl;
                std::cout << "OVERFLOW   ! seq: " << seq_recv_
                          << ", crc: " << seq_recv_crc_error_
                          << ", ofl: " << seq_recv_overflow_
                          << ", ect:" << seq_recv_exception_ << "\n";
              }
              // std::cout << (int)buf_recv_[i] << std::endl;
            }
          }
        } else {  // flagStacking == false    아직 STX를 발견하지 못함. (Stack
                  // 하지않음)
          if (flagDLEFound) {  // 이전에 DLE나옴.
            if (c == STX) {  // STX 찾음, 새로운 packet을 stack 시작함.
              // std::cout << "== " << " found STX!\n";

              flagStacking = true;
              len_stack_ = 0;

              // flag_recv_packet_ready_ = false;
            }
            flagDLEFound = false;  // STX 이든 아니든...
          } else {                 //
            // std::cout << " not found STX. DLE found. \n";
            if (c == DLE) flagDLEFound = true;
          }
        }
      }
    }

    std::future_status terminate_status =
        terminate_signal.wait_for(std::chrono::microseconds(10));
    if (terminate_status == std::future_status::ready) break;
  }
  PrintInfo("SerialCommunicator - RX thread receives termination signal.");
}

void SerialCommunicator::processTX(std::shared_future<void> terminate_signal) {
  while (true) {
    if (flag_ready_to_send_) {
      if (mutex_tx_->try_lock()) {
        flag_ready_to_send_ = false;

        SendPacketWithChecksum(packet_send_, len_packet_send_);
        len_packet_send_ = 0;
        ++seq_send_;
        mutex_tx_->unlock();
      }
    }

    std::future_status terminate_status =
        terminate_signal.wait_for(std::chrono::microseconds(50));
    if (terminate_status == std::future_status::ready) {
      // Zero send.
      USHORT_UNION pwm;
      pwm.ushort_ = 0;

      PrintInfo(
          "=============== Catch terminate signal in TX thread "
          "===============");
      PrintInfo("    Send the zero signals to all PWM channels for safety.");

      mutex_tx_->lock();
      for (int i = 0; i < 8; ++i) {
        packet_send_[2 * i] = pwm.bytes_[0];
        packet_send_[2 * i + 1] = pwm.bytes_[1];
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
  USHORT_UNION crc16_calc;
  crc16_calc.ushort_ = CalculateChecksumCRC16CCITT(data, 0, len - 1);

  int idx = 2;
  buf_send_[0] = DLE;
  buf_send_[1] = STX;  // DLE, STX --> start of the packet
  for (int i = 0; i < len; ++i) {
    if (data[i] == DLE) {
      buf_send_[idx++] = DLE;
      buf_send_[idx++] = DLE;
    } else {
      buf_send_[idx++] = data[i];
    }
  }

  // len = idx - 2;
  if (crc16_calc.bytes_[0] == DLE) {
    buf_send_[idx++] = DLE;
    buf_send_[idx++] = DLE;
  } else {
    buf_send_[idx++] = crc16_calc.bytes_[0];
  }

  if (crc16_calc.bytes_[1] == DLE) {
    buf_send_[idx++] = DLE;
    buf_send_[idx++] = DLE;
  } else {
    buf_send_[idx++] = crc16_calc.bytes_[1];
  }

  buf_send_[idx++] = DLE;
  buf_send_[idx++] = ETX;  // DLE, ETX --> end of the packet.

  // std::cout << "send data: ";
  // for(int i = 0; i < idx; ++i){
  //     std::cout << (int)buf_send_[i] <<" ";
  // }
  // std::cout << "\n";
  serial_->write_some(boost::asio::buffer(buf_send_, idx));
  // boost::asio::write(*serial_, boost::asio::buffer(buf_send_, idx));
  // std::cout << "write length : " << len +6 << std::endl;
}

unsigned short SerialCommunicator::stringChecksumCRC16_CCITT(
    const unsigned char* s, int idx_start, int idx_end) {
  return CalculateChecksumCRC16CCITT(s, idx_start, idx_end);
}