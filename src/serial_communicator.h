#ifndef SERIAL_COMMUNICATOR_H_
#define SERIAL_COMMUNICATOR_H_

#define kBufferSize 1024

#include <chrono>
#include <cstring>
#include <exception>
#include <future>
#include <iostream>
#include <numeric>
#include <queue>
#include <string>
#include <thread>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

#include "crc16.h"  // CRC16 checksum test

#define DLE 0x10  // data link escape
#define STX 0x02  // start of text
#define ETX 0x03  // end of text

class SerialCommunicator {
 public:
  struct Parameters {
    enum class PacketType : uint8_t { kRaw = 0, kFrameWithChecksum };
    std::string port_name;
    int baud_rate;
    PacketType packet_type;
  };

  template <typename T>
  union Union {
    T value;
    unsigned char bytes[sizeof(T)];
  };

  struct RxStatistics {
    int num_total{0};
    int num_crc_error{0};
    int num_overflow{0};
    int num_exception{0};
  };

  struct TxStatistics {
    int num_total{0};
  };

 public:
  SerialCommunicator(const Parameters& parameters);
  ~SerialCommunicator();

  // Get packet
  bool IsNewPacketReceived();
  int GetPacket(unsigned char* data);
  std::string GetRawPacket();

  // Send packet
  bool SendPacket(const std::string& data);
  bool SendPacket(const unsigned char* data, int len);

  RxStatistics GetRxStatistics();
  TxStatistics GetTxStatistics();

 private:
  void ProcessRx();
  void ProcessTx();

  void SetBaudRate(const int baud_rate);
  void CheckSupportedBaudRate(const int baud_rate);

 private:
  void OpenSerialPort();
  void CloseSerialPort();

  void ParseRawPacket(const int received_length);
  void ParseFramedPacketWithChecksum(const int received_length);

  void SendRawPacket(const unsigned char* data, int len);
  void SendFramedPacketWithChecksum(const unsigned char* data, int len);
  unsigned short GetChecksumCRC16CCITT(const unsigned char* data,
                                       const int length);

  // Serial port related (boost::asio::serial )
 private:
  const Parameters parameters_;

  boost::asio::serial_port* serial_port_;
  boost::asio::io_service io_service_;
  boost::asio::deadline_timer timeout_;

  int index_;
  unsigned char packet_stack_[kBufferSize];

  // Related to RX (Nucleo -> PC)
 private:
  int seq_recv_;
  unsigned char buffer_recv_[kBufferSize];

  int len_packet_recv_;
  unsigned char packet_recv_[kBufferSize];

  std::atomic<bool> flag_recv_packet_ready_;

  std::mutex mutex_for_raw_data_queue_;
  std::queue<unsigned char> raw_data_queue_;

  // Related to TX (PC -> Nucleo)
 private:
  int seq_send_{0};
  unsigned char buffer_send_[kBufferSize];

  int len_packet_send_{0};
  unsigned char packet_send_[kBufferSize];

  std::atomic<bool> ready_to_send_;

 private:
  RxStatistics rx_stats_;
  TxStatistics tx_stats_;

  // Variables to elegantly terminate TX & RX threads
 private:
  std::shared_future<void> terminate_future_;
  std::promise<void> terminate_promise_;

  // TX & RX threads and their mutex
 private:
  std::shared_ptr<std::mutex> mutex_tx_;
  std::shared_ptr<std::mutex> mutex_rx_;
  std::thread thread_tx_;
  std::thread thread_rx_;
};

#endif  // SERIAL_COMMUNICATOR_H_