#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp" // 头文件依然是小写加下划线
#include "std_msgs/msg/float32.hpp"

class ScanConverter : public rclcpp::Node {
public:
    ScanConverter() : Node("scan_converter_node") {
        // 1. 修复类型名称：LaserScan (没有下划线)
        dist_pub_ = this->create_publisher<std_msgs::msg::Float32>("front_distance", 10);

        // 2. 修复类型名称：LaserScan (没有下划线)
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&ScanConverter::scan_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "雷达距离转换节点已启动.");
    }

private:
    // 回调函数参数类型也要同步修改为 LaserScan
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (msg->ranges.empty()) return;

        size_t center_index = msg->ranges.size() / 2;
        float distance = msg->ranges[center_index];

        auto output_msg = std_msgs::msg::Float32();
        output_msg.data = distance;
        dist_pub_->publish(output_msg);
    }

    // --- 关键：必须在这里声明成员变量 ---
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanConverter>());
    rclcpp::shutdown();
    return 0;
}