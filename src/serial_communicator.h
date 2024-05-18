#ifndef SERIAL_COMMUNICATOR_H_
#define SERIAL_COMMUNICATOR_H_

#define BUF_SIZE 1024

#include <chrono>
#include <cstring>
#include <exception>
#include <future>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

#include "crc16.h"  // CRC16 checksum test
#include "timer.h"
#include "union_struct.h"

#define DLE 0x10
#define STX 0x02
#define ETX 0x03

class SerialCommunicator {
 public:
  SerialCommunicator(const std::string& portname, const int baud_rate);
  ~SerialCommunicator();

  // Get packet
  bool IsPacketReady();
  int GetPacket(unsigned char* buffer);

  // Send packet
  bool SendPacket(unsigned char* buffer, int len);

  // Statistics
  void GetRxStatistics(uint32_t& seq, uint32_t& seq_crcerr,
                       uint32_t& seq_overflowerr, uint32_t& seq_exceptionerr);
  void GetTxStatistics(uint32_t& seq);

 private:
  void RunRxThread();
  void RunTxThread();

  void processRX(std::shared_future<void> terminate_signal);
  void processTX(std::shared_future<void> terminate_signal);

 private:
  void SetPortName(const std::string& portname);
  void SetBaudRate(const int baud_rate);
  void CheckSupportedBaudRate(const int baud_rate);

  // Port settings
 private:
  void OpenSerialPort();
  void CloseSerialPort();

 private:
  void SendPacketWithChecksum(const unsigned char* data, int len);

 private:
  unsigned short stringChecksumCRC16_CCITT(const unsigned char* s,
                                           int idx_start, int idx_end);

  // Serial port related (boost::asio::serial )
 private:
  std::string portname_;
  int baud_rate_;

  boost::asio::serial_port* serial_;
  boost::asio::io_service io_service_;
  boost::asio::deadline_timer timeout_;

  int len_stack_;
  unsigned char packet_stack_[BUF_SIZE];

  // Related to RX (Nucleo -> PC)
 private:
  int seq_recv_;
  unsigned char buf_recv_[BUF_SIZE];

  int len_packet_recv_;
  unsigned char packet_recv_[BUF_SIZE];

  std::atomic<bool> flag_recv_packet_ready_;

  // Related to TX (PC -> Nucleo)
 private:
  int seq_send_{0};
  unsigned char buf_send_[BUF_SIZE];

  int len_packet_send_{0};
  unsigned char packet_send_[BUF_SIZE];

  std::atomic<bool> flag_ready_to_send_;

  // For debug
 private:
  int seq_recv_crc_error_;
  int seq_recv_overflow_;
  int seq_recv_exception_;

  // Variables to elegantly terminate TX & RX threads
 private:
  std::shared_future<void> terminate_future_;
  std::promise<void> terminate_promise_;

  // TX & RX threads and their mutex
 private:
  std::thread thread_tx_;
  std::thread thread_rx_;

  std::shared_ptr<std::mutex> mutex_tx_;
  std::shared_ptr<std::mutex> mutex_rx_;
};

#endif  // SERIAL_COMMUNICATOR_H_