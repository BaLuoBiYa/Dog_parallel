# Dog_Parallel ——针对8自由度并联狗的控制

### dog_bringup
用于启动控制回路，包括机器人URDF描述和参数文件（未完成） \
输入：无 \
输出：无

### dog_tools
提供帮助函数和统一类型定义 \
输入：无  \
输出：无

`以上为帮助函数，不加入控制回路`
---

### dog_trajectory
用于离线计算足端轨迹 \
输入：目标步态，当前足端位置 \
输出：足端轨迹

### dog_controller
用于跟踪并微调足端轨迹 \
输入：离线计算的足端轨迹，机体状态，（环境感知）（未完成） \
输出：电机指令

### dog_hardware
用于提供硬件接口 \
输入：电机指令 \
输出： 无

`以上为高频控制回路，由ros_control管理，运行于高频`
---

## dog_interface(未完成)
用于和决策层交互 \
输入：抽象控制指令 \
输出：目标步态

## dog_handler(未完成)
用于针对不同的环境设置不同的处理策略,需要被`dog_interface`调用 \
输入：无 \
输出：目标步态

# 已知问题：
## 1.Ubuntu 22.04版本的yaml-cpp库为0.7.0,无法正常安装，需要手动安装0.8.0以上版本
```bash
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
sudo ldconfig
```

## 2.依赖
```bash
sudo apt install ros-<distro>-pinocchio

```

