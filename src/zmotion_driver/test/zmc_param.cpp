#include "rclcpp/rclcpp.hpp"
#include "zmotion_driver/zmcaux.h"

class ZmcDriverNode : public rclcpp::Node {
public:
    ZmcDriverNode() : Node("zmc_driver_node") {
        // --- 1. 获取参数 (如果没有配置，则使用默认值) ---
        // 获取 IP 地址
        std::string ip = this->declare_parameter<std::string>("controller_ip", "192.168.0.11");
        
        // 获取轴 0 的运动参数
        float units = this->declare_parameter<float>("axis0.units", 1000.0f);
        float speed = this->declare_parameter<float>("axis0.speed", 5.0f);
        float accel = this->declare_parameter<float>("axis0.accel", 50.0f);
        float decel = this->declare_parameter<float>("axis0.decel", 50.0f);

        // --- 2. 连接控制器 ---
        if (ZAux_OpenEth((char*)ip.c_str(), &handle_) == 0) {
            RCLCPP_INFO(this->get_logger(), "连接成功: %s", ip.c_str());

            // --- 3. 将 ROS 参数下发给控制器 ---
            int axis = 0;
            ZAux_Direct_SetUnits(handle_, axis, units);
            ZAux_Direct_SetSpeed(handle_, axis, speed);
            ZAux_Direct_SetAccel(handle_, axis, accel);
            ZAux_Direct_SetDecel(handle_, axis, decel);

            RCLCPP_INFO(this->get_logger(), "轴 %d 参数初始化完成: Units=%.1f, Speed=%.1f", axis, units, speed);
        } else {
            RCLCPP_ERROR(this->get_logger(), "连接失败: %s", ip.c_str());
        }
    }

    ~ZmcDriverNode() {
        if (handle_) ZAux_Close(handle_);
    }

private:
    ZMC_HANDLE handle_ = nullptr;
};