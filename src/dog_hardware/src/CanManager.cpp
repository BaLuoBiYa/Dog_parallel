#include <dog_hardware/CanManager.hpp>

/**
 * @param interface_name 交互接口,如 "can0"
 *
 */
CanManager::CanManager(const std::string &interface_name) {
    if (interface_name.size() > IFNAMSIZ) {
        throw std::invalid_argument("CAN interface name is too long!");
    }

    // 创建 Socket
    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        perror("Socket");
    }

    //  设置为非阻塞模式（防止 read 卡死系统）
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    // 获取目标网络接口的索引

    strcpy(ifr.ifr_name, interface_name.c_str());
    ioctl(sock, SIOCGIFINDEX, &ifr);

    // 绑定 Socket
    addr.can_family = PF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(sock, (struct sockaddr *)&addr, sizeof(addr));
}
/**
 * @param out_frame 外部can帧
 *
 */
bool CanManager::CanRead(struct can_frame &out_frame) {
    // 接收消息
    int nbytes = read(sock, &frame, sizeof(struct can_frame));
    if (nbytes == sizeof(struct can_frame)) {
        out_frame = frame;
        return true; // 成功读到新数据
    }
    return false;
}

bool CanManager::CanSend(canid_t can_id, unsigned char can_dlc, unsigned char *can_data) {
    // 发送消息
    frame.can_id = can_id;
    frame.can_dlc = can_dlc;
    memcpy(frame.data, can_data, can_dlc);
    write(sock, &frame, sizeof(struct can_frame));

    return true;
}