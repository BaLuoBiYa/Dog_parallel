#include "dog_remote/serialConfig.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>

#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace Dog8Dof {

    static void setErr(std::string *err, const std::string &msg) {
        if (err) {
            *err = msg;
        }
    }

    bool configureSerialFdLinux(int fd,
                                uint32_t baudrate,
                                bool useSbus8E2,
                                int vmin,
                                int vtimeDeciSeconds,
                                std::string *errMsg) {

        if (fd < 0) {
            setErr(errMsg, "invalid serial fd");
            return false;
        }
        if (baudrate == 0) {
            setErr(errMsg, "invalid baudrate (0)");
            return false;
        }

        bool baudOk = false;

#ifdef TCGETS2
        struct termios2 tio2 {};
        if (ioctl(fd, TCGETS2, &tio2) == 0) {
            tio2.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL |
                              IXON | IXOFF | IXANY);
            tio2.c_oflag &= ~OPOST;
            tio2.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

            tio2.c_cflag &= ~CSIZE;
            tio2.c_cflag |= CS8;
            tio2.c_cflag &= ~CRTSCTS;
            tio2.c_cflag |= (CLOCAL | CREAD);

            if (useSbus8E2) {
                tio2.c_cflag |= PARENB;
                tio2.c_cflag &= ~PARODD;
                tio2.c_cflag |= CSTOPB;
            } else {
                tio2.c_cflag &= ~PARENB;
                tio2.c_cflag &= ~CSTOPB;
            }

            tio2.c_cc[VMIN] = static_cast<cc_t>(std::max(0, vmin));
            tio2.c_cc[VTIME] = static_cast<cc_t>(std::max(0, vtimeDeciSeconds));

            tio2.c_cflag &= ~CBAUD;
            tio2.c_cflag |= BOTHER;
            tio2.c_ispeed = baudrate;
            tio2.c_ospeed = baudrate;

            if (ioctl(fd, TCSETS2, &tio2) == 0) {
                baudOk = true;
            }
        }
#endif

        if (!baudOk) {
            struct serial_struct ser {};
            if (ioctl(fd, TIOCGSERIAL, &ser) == 0 && ser.baud_base > 0) {
                int div = static_cast<int>((static_cast<long long>(ser.baud_base) +
                                            static_cast<long long>(baudrate) / 2) /
                                           static_cast<long long>(baudrate));
                if (div <= 0) {
                    div = 1;
                }

                ser.custom_divisor = div;
                ser.flags &= ~ASYNC_SPD_MASK;
                ser.flags |= ASYNC_SPD_CUST;

                if (ioctl(fd, TIOCSSERIAL, &ser) == 0) {
                    baudOk = true;
                }
            }
        }

        if (!baudOk) {
            setErr(errMsg,
                   std::string("failed to set baudrate: ") + std::strerror(errno));
            return false;
        }

        ioctl(fd, TCFLSH, TCIOFLUSH);
        return true;
    }

} // namespace Dog8Dof