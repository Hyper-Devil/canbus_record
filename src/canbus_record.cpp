// 打开CAN设备0通道，记录所有CAN数据，发布到话题“/canbus_record”
// sudo apt install ros-noetic-can-msgs
// ROS话题消息类型为can_msgs/Frame(https://docs.ros.org/en/noetic/api/can_msgs/html/msg/Frame.html)
// 要求话题消息时间戳为ros::Time::now())，严格遵守can_msgs/Frame格式规范:
// Header header
// uint32 id // CAN ID
// bool is_rtr // 是否为远程帧
// bool is_extended // 是否为扩展帧
// bool is_error // 是否为错误帧
// uint8 dlc // 字节长度
// uint8[8] data // 数据

#include <stdio.h>
#include <algorithm>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <stdexcept>

#include <ros/ros.h>
#include <can_msgs/Frame.h>

#include "iTekon-usb.h"

namespace
{
volatile sig_atomic_t g_stop_requested = 0;

void sigint_handler(int)
{
    g_stop_requested = 1;
}
} // namespace

class CanbusRecord
{
public:
    CanbusRecord();
    ~CanbusRecord();
    void can_recv_pub();
    void cleanup();
    void signal_handle();
    void load_params();
    int resolve_bot_rate(const std::string &bot_rate_name) const;

    ros::NodeHandle nh;
    ros::Publisher pub_can;

    VCI_BOARD_INFO board_info;
    VCI_INIT_CONFIG init_config0;
    int ret = 0;
    static const int buffer_size = 1000; // 接收缓存为1000帧，溢出时可能会出现异常
    VCI_CAN_OBJ recv_msg[buffer_size];

private:
    int device_type_ = 4;
    int device_index_ = 0;
    int can_channel_ = 0;
    std::string frame_id_ = "base_link";
    std::string bot_rate_name_ = "500K";
    bool has_cleaned_ = false;
};

// 构造函数：初始化CAN设备
CanbusRecord::CanbusRecord()
{
    load_params();
    pub_can = nh.advertise<can_msgs::Frame>("canbus_record", 100);

    // 初始化USB
    ret = VCI_UsbInit();
    if (ret == 0)
    {
        cleanup();
        throw std::runtime_error("VCI_UsbInit failed");
    }

    // 打开设备
    ret = VCI_OpenDevice(device_type_, device_index_, 0);
    if (ret == 0)
    {
        cleanup();
        throw std::runtime_error("VCI_OpenDevice failed");
    }

    bzero(&init_config0, sizeof(VCI_INIT_CONFIG));

    // CAN0初始化参数
    init_config0.filter_num = 1;                   // CAN0通道使用过滤器组数量
    init_config0.filter_info[0].FilterEN = 1;      // 默认第一组过滤器使能（至少应使能一组过滤器）
    init_config0.filter_info[0].FilterMode = 0x01; // 32位掩码模式
    init_config0.filter_info[0].FilterId = 0x00000000;
    init_config0.filter_info[0].FilterMask = 0x00000000; // 所有Id全部接收
    init_config0.Mode = MODE_NORMAL;                     // 正常工作模式
    init_config0.BotRate = resolve_bot_rate(bot_rate_name_);

    // 初始化CAN0通道
    ret = VCI_InitCan(device_type_, device_index_, can_channel_, &init_config0);
    if (ret == 0)
    {
        cleanup();
        throw std::runtime_error("VCI_InitCan failed");
    }

    // 读设备信息
    bzero(&board_info, sizeof(board_info));
    ret = VCI_ReadBoardInfo(device_type_, device_index_, &board_info);
    if (ret == 0)
    {
        cleanup();
        throw std::runtime_error("VCI_ReadBoardInfo failed");
    }

    // 启动CAN0通道
    ret = VCI_StartCAN(device_type_, device_index_, can_channel_);
    if (ret == 0)
    {
        cleanup();
        throw std::runtime_error("VCI_StartCAN failed");
    }

    signal_handle();
}

CanbusRecord::~CanbusRecord()
{
    cleanup();
}

void CanbusRecord::load_params()
{
    nh.param("device_type", device_type_, 4);
    nh.param("device_index", device_index_, 0);
    nh.param("can_channel", can_channel_, 0);
    nh.param<std::string>("frame_id", frame_id_, "base_link");
    nh.param<std::string>("bot_rate", bot_rate_name_, "500K");
}

int CanbusRecord::resolve_bot_rate(const std::string &bot_rate_name) const
{
    if (bot_rate_name == "500K")
    {
        return BOT_500K;
    }

    ROS_WARN_STREAM("Unsupported bot_rate='" << bot_rate_name << "', fallback to 500K");
    return BOT_500K;
}

void CanbusRecord::signal_handle()
{
    printf("---%s\n", __func__);
    struct sigaction sig_handle;
    memset(&sig_handle, 0, sizeof(sig_handle));
    sigemptyset(&sig_handle.sa_mask);
    sig_handle.sa_handler = sigint_handler;
    sigaction(SIGINT, &sig_handle, NULL);
}

// 接收CAN数据并发布到话题
void CanbusRecord::can_recv_pub()
{
    memset(recv_msg, 0, sizeof(VCI_CAN_OBJ) * buffer_size); // 接收缓存->recv_msg
    while (ros::ok() && !g_stop_requested)
    {
        int recvlen = VCI_Receive(device_type_, device_index_, can_channel_, recv_msg, buffer_size, 0);
        // printf("---recvnum:%d\n", recvlen);
        if (recvlen <= 0)
        {
            usleep(1000);
            continue;
        }

        for (int i_frame = 0; i_frame < recvlen; i_frame++) // 遍历缓存帧
        {
            can_msgs::Frame can_frame;
            can_frame.header.stamp = ros::Time::now();
            can_frame.header.frame_id = frame_id_;
            can_frame.id = recv_msg[i_frame].ID;
            can_frame.is_rtr = recv_msg[i_frame].RemoteFlag;
            can_frame.is_extended = recv_msg[i_frame].ExternFlag;
            can_frame.is_error = 0;
            can_frame.dlc = static_cast<uint8_t>(std::min<int>(recv_msg[i_frame].DataLen, 8));
            if (recv_msg[i_frame].DataLen > 8)
            {
                ROS_WARN_STREAM_THROTTLE(1.0, "Received DataLen=" << static_cast<int>(recv_msg[i_frame].DataLen)
                                             << " > 8, truncated to 8");
            }
            can_frame.data.fill(0);
            
            // printf("ID: %08x DataLen:%02x ExternFlag:%02x RemoteFlag: %02x data:", recv_msg[i].ID, recv_msg[i].DataLen, recv_msg[i].ExternFlag, recv_msg[i].RemoteFlag);
            
            /*写法1*/
            // for (j_byte = 0; j_byte < recv_msg[i_frame].DataLen; j_byte++) // 遍历帧数据
            // {
            //     can_frame.data[j_byte] = recv_msg[i_frame].Data[j_byte];
            //     //printf("%02x ", recv_msg[i_frame].Data[j_byte]);
            //     ROS_INFO("0x%02x ", recv_msg[i_frame].Data[j_byte]);
            // }
            /*写法2*/
            if (can_frame.dlc > 0)
            {
                memcpy(can_frame.data.data(), recv_msg[i_frame].Data, can_frame.dlc);
            }
            
            // printf("\n");
            pub_can.publish(can_frame);
            
            ROS_INFO_STREAM("PUBLISH ID: " << std::hex << can_frame.id << std::dec);
        }
    }

    cleanup();
}

void CanbusRecord::cleanup()
{
    if (has_cleaned_)
    {
        return;
    }
    has_cleaned_ = true;

    // 复位CAN0通道
    VCI_ResetCAN(device_type_, device_index_, can_channel_);
    // 复位CAN1通道
    // VCI_ResetCAN(4, 0, 1);
    // 关闭设备
    VCI_CloseDevice(device_type_, device_index_);
    // 释放USB连接
    VCI_UsbExit();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "canbus_record");
    try
    {
        CanbusRecord canbus_record;
        canbus_record.can_recv_pub();
        return 0;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("canbus_record init failed: " << e.what());
        return 1;
    }
}