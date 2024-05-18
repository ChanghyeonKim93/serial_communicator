#include "serial_communicator.h"

int main() {
  SerialCommunicator serial_comm("/dev/usb", 115200);
  return 0;
}