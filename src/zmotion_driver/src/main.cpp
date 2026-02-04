#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "zmotion_driver/zmc_controller.h"

int main(int argc, char * argv[]) {
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建ZmcController实例（自动初始化ROS2节点）
    auto zmc_controller = std::make_shared<ZmcController>("zmc_node");
    
    // 显式启动控制器（建立连接并开始发布）
    zmc_controller->start();
    
    // 运行节点
    rclcpp::spin(zmc_controller);
    
    // 停止控制器并关闭
    zmc_controller->stop();
    rclcpp::shutdown();
    
    return 0;
}