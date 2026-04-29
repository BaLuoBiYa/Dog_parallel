#include <fcntl.h>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <memory.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

class CanManager {
  private:
    int sock;
    struct sockaddr_can addr;
    struct can_frame frame;
    struct ifreq ifr;
    /* data */
  public:
    CanManager(const std::string &interface_name);
    bool CanRead(struct can_frame &out_frame);
    bool CanSend(canid_t can_id, unsigned char can_dlc, unsigned char *can_data);
};