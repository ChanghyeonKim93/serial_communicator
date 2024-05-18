#include "serial_communicator.h"

int main() {
  SerialCommunicator serial_comm("/dev/ttyACM0", 921600);
  while (true) {
    sleep(1);
  }
  return 0;
}