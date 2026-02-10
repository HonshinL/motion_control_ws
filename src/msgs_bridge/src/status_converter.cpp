#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MsgConverter : public rclcpp::Node {
public:
    MsgConverter() : Node("msg_converter_node") {
        // 1. 创建发布者，发布到 topic2
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic2", 10);

        // 2. 创建订阅者，订阅 topic1
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "topic1", 10,
            std::bind(&MsgConverter::topic_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "消息转换节点已启动: topic1 -> topic2");
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        // --- 转换逻辑开始 ---
        auto new_msg = std_msgs::msg::String();
        
        // 示例：在原始消息前加上 "Converted: "
        new_msg.data = "Converted: " + msg->data;
        
        RCLCPP_INFO(this->get_logger(), "转发消息: '%s'", new_msg.data.c_str());
        // --- 转换逻辑结束 ---

        // 3. 发布转换后的消息
        publisher_->publish(new_msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MsgConverter>());
    rclcpp::shutdown();
    return 0;
}