#include <iostream>
#include <time.h>
#include <string>

// serial
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/poll.h>
#include <sys/time.h>
#include "serial_comm.h"


using namespace std;

int main(int argc, char **argv) {
    string serial_name = "/dev/arduinoMEGA";
    int message_length = 21;

    SerialCommunicator serial_com(serial_name, message_length);
    serial_com.runThread();
    
    return -1;
}
