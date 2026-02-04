#include <iostream>
#include <vector>
#include <unistd.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "zmotion_driver/zmcaux.h" // 引用头文件

class MotionControlNode : public rclcpp::Node {
public:
    MotionControlNode() : Node("motion_control_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing motion control node...");
        
        // 仿真器通常在Windows上，Linux环境下请修改为实际控制卡的IP
        char ip_addr[] = "192.168.0.11"; 

        // 1. 连接控制器
        int ret = ZAux_OpenEth(ip_addr, &handle_);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to controller, error code: %d", ret);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully connected to controller: %s", ip_addr);

        // 2. 获取控制器信息 (ZAux_GetControllerInfo)
        char soft_type[32], soft_version[32], controller_id[32];
        ret = ZAux_GetControllerInfo(handle_, soft_type, soft_version, controller_id);
        if (ret == 0) {
            RCLCPP_INFO(this->get_logger(), "Controller Info:");
            RCLCPP_INFO(this->get_logger(), "  Model: %s", soft_type);
            RCLCPP_INFO(this->get_logger(), "  Version: %s", soft_version);
            RCLCPP_INFO(this->get_logger(), "  ID: %s", controller_id);
        }

        // 3. 获取最大规格数 (ZAux_GetSysSpecification)
        uint16 max_virt_axes;
        uint8 max_motors;
        uint8 max_io[4]; // 分别存 IN, OUT, AD, DA 的最大值
        ret = ZAux_GetSysSpecification(handle_, &max_virt_axes, &max_motors, max_io);
        if (ret == 0) {
            RCLCPP_INFO(this->get_logger(), "System Specification:");
            RCLCPP_INFO(this->get_logger(), "  Max Virtual Axes: %d", max_virt_axes);
            RCLCPP_INFO(this->get_logger(), "  Max Motors: %d", max_motors);
            RCLCPP_INFO(this->get_logger(), "  Digital Inputs (IN): %d", (int)max_io[0]);
            RCLCPP_INFO(this->get_logger(), "  Digital Outputs (OUT): %d", (int)max_io[1]);
        }
        
        // 4. 关闭连接（在实际应用中，可能需要在节点销毁时关闭连接）
        ZAux_Close(handle_);
        handle_ = NULL;
        RCLCPP_INFO(this->get_logger(), "Connection closed safely");
    }
    
    ~MotionControlNode() {
        if (handle_ != NULL) {
            ZAux_Close(handle_);
            handle_ = NULL;
            RCLCPP_INFO(this->get_logger(), "Connection closed safely in destructor");
        }
    }
    
private:
    ZMC_HANDLE handle_ = NULL;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionControlNode>();
    rclcpp::spin_some(node); // 处理所有待处理的事件，然后返回
    rclcpp::shutdown();
    return 0;
}