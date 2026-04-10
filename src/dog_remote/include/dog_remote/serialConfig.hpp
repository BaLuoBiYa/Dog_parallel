#pragma once

#include <cstdint>
#include <string>

namespace Dog8Dof {

    bool configureSerialFdLinux(int fd,
                                uint32_t baudrate,
                                bool useSbus8E2,
                                int vmin,
                                int vtimeDeciSeconds,
                                std::string *errMsg);

} // namespace Dog8Dof