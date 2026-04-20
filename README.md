# canbus_record

ROS1 (Noetic) 节点：打开 iTekon USB-CAN 设备的 CAN0 通道，持续接收 CAN 帧并发布到话题 `/canbus_record`，消息类型为 `can_msgs/Frame`。

## 功能概述

- 初始化 USB-CAN 设备并启动 CAN0 通道。
- 使用厂商库读取 CAN 帧。
- 将每帧数据映射为 `can_msgs/Frame` 后发布。
- 时间戳使用 `ros::Time::now()`。

实现位置：`src/canbus_record.cpp`。

## 依赖

### 系统依赖

- Ubuntu 20.04
- ROS Noetic
- `can_msgs`

安装示例：

```bash
sudo apt update
sudo apt install -y ros-noetic-can-msgs
```

### 第三方库

- iTekon USB-CAN 驱动头文件：`include/iTekon-usb.h`
- iTekon USB-CAN 动态库/静态库：`lib/libiTekon-usb.*`

当前 CMake 通过 `link_directories(lib)` + `target_link_libraries(... iTekon-usb)` 链接该库。

## 编译

在工作空间根目录执行：

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

## 运行

```bash
rosrun canbus_record canbus_record
```

或使用 launch（推荐）：

```bash
roslaunch canbus_record canbus_record.launch
```

查看话题：

```bash
rostopic list | grep canbus_record
rostopic echo /canbus_record
```

## 发布消息定义与字段映射

消息类型：`can_msgs/Frame`

字段映射（代码见 `src/canbus_record.cpp`）：

- `header.stamp = ros::Time::now()`
- `header.frame_id = "base_link"`
- `id = recv_msg[i].ID`
- `is_rtr = recv_msg[i].RemoteFlag`
- `is_extended = recv_msg[i].ExternFlag`
- `is_error = false`
- `dlc = min(recv_msg[i].DataLen, 8)`
- `data[0:dlc] = recv_msg[i].Data[0:dlc]`

## 参数说明

节点支持以下 ROS 参数（未设置时使用默认值）：

- `device_type`：默认 `4`
- `device_index`：默认 `0`
- `can_channel`：默认 `0`
- `bot_rate`：默认 `500K`（当前版本支持 `500K`，其他值会回退到 `500K`）
- `frame_id`：默认 `base_link`

对应 launch 文件：`launch/canbus_record.launch`。

## 许可

见 `package.xml`：MIT。
