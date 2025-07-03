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
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <can_msgs/Frame.h>

#include "iTekon-usb.h"

class CanbusRecord
{
public:
    CanbusRecord();
    ~CanbusRecord();
    void can_recv_pub();
    void exit_can(int flag);
    void signal_handle();

    ros::NodeHandle nh;
    ros::Publisher pub_can = nh.advertise<can_msgs::Frame>("canbus_record", 100);

    VCI_BOARD_INFO board_info;
    VCI_INIT_CONFIG init_config0, init_config1;
    int ret = 0;
    static const int buffer_size = 1000; // 接收缓存为1000帧，溢出时可能会出现异常
    VCI_CAN_OBJ recv_msg[buffer_size];
};

// 构造函数：初始化CAN设备
CanbusRecord::CanbusRecord()
{
    // 初始化USB
    ret = VCI_UsbInit();
    if (ret == 0)
    {
        exit_can(1);
        // return 0;
    }

    // 打开设备
    ret = VCI_OpenDevice(4, 0, 0);
    if (ret == 0)
    {
        exit_can(1);
        // return 0;
    }

    bzero(&init_config0, sizeof(VCI_INIT_CONFIG));

    // CAN0初始化参数
    init_config0.filter_num = 1;                   // CAN0通道使用过滤器组数量
    init_config0.filter_info[0].FilterEN = 1;      // 默认第一组过滤器使能（至少应使能一组过滤器）
    init_config0.filter_info[0].FilterMode = 0x01; // 32位掩码模式
    init_config0.filter_info[0].FilterId = 0x00000000;
    init_config0.filter_info[0].FilterMask = 0x00000000; // 所有Id全部接收
    init_config0.Mode = MODE_NORMAL;                     // 正常工作模式
    init_config0.BotRate = BOT_500K;

    // 初始化CAN0通道
    ret = VCI_InitCan(4, 0, 0, &init_config0);
    if (ret == 0)
    {
        exit_can(1);
        // return 0;
    }

    // 读设备信息
    bzero(&board_info, sizeof(board_info));
    ret = VCI_ReadBoardInfo(4, 0, &board_info);
    if (ret == 0)
    {
        exit_can(1);
        // return 0;
    }

    // 启动CAN0通道
    ret = VCI_StartCAN(4, 0, 0);
    if (ret == 0)
    {
        exit_can(1);
        // return 0;
    }

    signal_handle();
}

CanbusRecord::~CanbusRecord()
{
    VCI_CloseDevice(4, 0);
}

void CanbusRecord::signal_handle()
{
    printf("---%s\n", __func__);
    struct sigaction sig_handle;
    memset(&sig_handle, 0, sizeof(sig_handle));
    sigemptyset(&sig_handle.sa_mask);
    sig_handle.sa_flags = SA_SIGINFO;
    // sig_handle.sa_sigaction = exit_can;
    sigaction(SIGINT, &sig_handle, NULL);
}

// 接收CAN数据并发布到话题
void CanbusRecord::can_recv_pub()
{
    memset(recv_msg, 0, sizeof(VCI_CAN_OBJ) * buffer_size); // 接收缓存->recv_msg
    while (true)
    {
        int recvlen = VCI_Receive(4, 0, 0, recv_msg, buffer_size, 0); // 0通道
        // printf("---recvnum:%d\n", recvlen);
        int i_frame, j_byte;
        for (i_frame = 0; i_frame < recvlen; i_frame++) // 遍历缓存帧
        {
            can_msgs::Frame can_frame;
            can_frame.header.stamp = ros::Time::now();
            can_frame.header.frame_id = "base_link";
            can_frame.id = recv_msg[i_frame].ID;
            can_frame.is_rtr = recv_msg[i_frame].RemoteFlag;
            can_frame.is_extended = recv_msg[i_frame].ExternFlag;
            can_frame.is_error = 0;
            can_frame.dlc = recv_msg[i_frame].DataLen;
            
            // printf("ID: %08x DataLen:%02x ExternFlag:%02x RemoteFlag: %02x data:", recv_msg[i].ID, recv_msg[i].DataLen, recv_msg[i].ExternFlag, recv_msg[i].RemoteFlag);
            
            /*写法1*/
            // for (j_byte = 0; j_byte < recv_msg[i_frame].DataLen; j_byte++) // 遍历帧数据
            // {
            //     can_frame.data[j_byte] = recv_msg[i_frame].Data[j_byte];
            //     //printf("%02x ", recv_msg[i_frame].Data[j_byte]);
            //     ROS_INFO("0x%02x ", recv_msg[i_frame].Data[j_byte]);
            // }
            /*写法2*/
            memcpy(can_frame.data.data(), recv_msg[i_frame].Data, recv_msg[i_frame].DataLen);
            
            // printf("\n");
            pub_can.publish(can_frame);
            
            ROS_INFO_STREAM("PUBLISH ID: " << std::hex << can_frame.id);
        }
    }
}

// flag=1：关闭设备
void CanbusRecord::exit_can(int flag)
{
    int time = 0;
    if (1 != flag) //多线程相关，暂时不考虑
    {
        // printf("---------------------------------------ctrl+c\n");
        // // 关闭发送线程。
        // breakflag = 1;
        // while (1)
        // {
        //     if ((breakflag == 2) || (breakflag == 0))
        //     {
        //         break;
        //     }
        //     sleep(1);
        //     time++;
        //     if (time > 5)
        //         break;
        // }
        // breakflag = 0;
    }

    // 复位CAN0通道
    VCI_ResetCAN(4, 0, 0);
    // 复位CAN1通道
    // VCI_ResetCAN(4, 0, 1);
    // 关闭设备
    VCI_CloseDevice(4, 0);
    // 释放USB连接
    VCI_UsbExit();
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "canbus_record");
    CanbusRecord canbus_record;
    canbus_record.can_recv_pub();
    return 0;
}