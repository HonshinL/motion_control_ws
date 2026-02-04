#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp" // 简单起见使用 Float64，实际项目常用 JointState
#include "zmotion_driver/zmcaux.h"

using namespace std::chrono_literals;

class ZmcStatusPublisher : public rclcpp::Node {
public:
    ZmcStatusPublisher() : Node("zmc_status_publisher") {
        // 1. 声明并获取参数
        std::string ip = this->declare_parameter<std::string>("controller_ip", "192.168.0.11");
        axis_ = this->declare_parameter<int>("monitoring_axis", 0);

        // 2. 连接控制器
        // 注意：ZAux_OpenEth 函数期望 char* 参数，但 std::string::c_str() 返回 const char*
        // 这里创建一个临时的可修改字符串副本
        char ip_buffer[16]; // IPv4 地址最多需要 15 个字符
        std::strncpy(ip_buffer, ip.c_str(), sizeof(ip_buffer)-1);
        ip_buffer[sizeof(ip_buffer)-1] = '\0'; // 确保字符串以 null 结尾
        
        if (ZAux_OpenEth(ip_buffer, &handle_) == 0) {
            RCLCPP_INFO(this->get_logger(), "已连接控制器: %s, 正在监控轴 %d", ip.c_str(), axis_);
        } else {
            RCLCPP_ERROR(this->get_logger(), "无法连接控制器!");
        }

        // 3. 创建发布者 (Publisher)
        // 发布指令位置 (Command Position)
        dpos_pub_ = this->create_publisher<std_msgs::msg::Float64>("zmc/dpos", 10);
        // 发布反馈位置 (Actual Position)
        mpos_pub_ = this->create_publisher<std_msgs::msg::Float64>("zmc/mpos", 10);

        // 4. 创建定时器 (WallTimer)
        // 每 20 毫秒执行一次 timer_callback (50Hz)
        timer_ = this->create_wall_timer(20ms, std::bind(&ZmcStatusPublisher::timer_callback, this));
    }

    ~ZmcStatusPublisher() {
        if (handle_) ZAux_Close(handle_);
    }

private:
    void timer_callback() {
        if (!handle_) return;

        float dpos_val = 0.0;
        float mpos_val = 0.0;

        // 从硬件读取数据
        ZAux_Direct_GetDpos(handle_, axis_, &dpos_val);
        ZAux_Direct_GetMpos(handle_, axis_, &mpos_val);

        // 封装并发布 DPOS
        auto dpos_msg = std_msgs::msg::Float64();
        dpos_msg.data = dpos_val;
        dpos_pub_->publish(dpos_msg);

        // 封装并发布 MPOS
        auto mpos_msg = std_msgs::msg::Float64();
        mpos_msg.data = mpos_val;
        mpos_pub_->publish(mpos_msg);

        // (可选) 调试打印，实际运行时建议注释掉以节省 CPU
        // RCLCPP_DEBUG(this->get_logger(), "Published DPOS: %.3f", dpos_val);
    }

    // 成员变量
    ZMC_HANDLE handle_ = nullptr;
    int axis_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dpos_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mpos_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZmcStatusPublisher>());
    rclcpp::shutdown();
    return 0;
}