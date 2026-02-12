#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "slms_interface/msg/object_position.hpp"
#include "motion_msgs/action/move_to_position.hpp"

class ObjectPositionToActionConverter : public rclcpp::Node {
public:
    using MoveToPosition = motion_msgs::action::MoveToPosition;
    using GoalHandleMove = rclcpp_action::ClientGoalHandle<MoveToPosition>;

    explicit ObjectPositionToActionConverter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("object_position_to_action_converter", options) {
        
        // 创建 ObjectPosition 消息订阅者
        object_position_sub_ = this->create_subscription<slms_interface::msg::ObjectPosition>(
            "/app_pos", 10,
            std::bind(&ObjectPositionToActionConverter::object_position_callback, this, std::placeholders::_1));
        
        // 创建 MoveToPosition Action 客户端
        action_client_ = rclcpp_action::create_client<MoveToPosition>(this, "zmc_act/move_to_position");
        
        RCLCPP_INFO(this->get_logger(), "ObjectPosition 到 Action 转换节点已启动");
        RCLCPP_INFO(this->get_logger(), "订阅话题: /app_pos");
        RCLCPP_INFO(this->get_logger(), "发布 Action: zmc_act/move_to_position");
    }

private:
    void object_position_callback(const slms_interface::msg::ObjectPosition::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "收到 ObjectPosition 消息，模式: %d, 轴号: %d, 坐标系: %d", 
                   msg->mode, msg->axis_num, msg->plane_coord);
        
        // 检查消息有效性
        if (msg->pos.empty()) {
            RCLCPP_ERROR(this->get_logger(), "ObjectPosition 消息中位置数据为空");
            return;
        }
        
        // 根据模式处理不同的运动类型
        switch (msg->mode) {
            case 0: // 单轴回零
                RCLCPP_INFO(this->get_logger(), "模式0: 单轴回零，轴号: %d", msg->axis_num);
                // 这里可以实现回零逻辑
                break;
                
            case 1: // 单轴运动
                if (msg->pos.size() >= 1) {
                    RCLCPP_INFO(this->get_logger(), "模式1: 单轴运动，轴号: %d, 目标位置: %.3f", 
                               msg->axis_num, msg->pos[0]);
                    send_move_action(msg->axis_num, msg->pos[0]);
                }
                break;
                
            case 2: // 双轴运动(X, Y)
                if (msg->pos.size() >= 2) {
                    RCLCPP_INFO(this->get_logger(), "模式2: 双轴运动，X: %.3f, Y: %.3f", 
                               msg->pos[0], msg->pos[1]);
                    send_multi_axis_action({0, 1}, {msg->pos[0], msg->pos[1]});
                }
                break;
                
            case 3: // 振镜运动
                RCLCPP_INFO(this->get_logger(), "模式3: 振镜运动");
                // 这里可以实现振镜控制逻辑
                break;
                
            default:
                RCLCPP_WARN(this->get_logger(), "未知的运动模式: %d", msg->mode);
                break;
        }
    }
    
    // 发送单轴运动 Action
    void send_move_action(int axis_num, float target_position) {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action 服务器不可用");
            return;
        }
        
        auto goal_msg = MoveToPosition::Goal();
        goal_msg.target_axes = {axis_num};
        goal_msg.target_positions = {target_position};
        goal_msg.speed = 50.0;
        goal_msg.acceleration = 100.0;
        goal_msg.deceleration = 100.0;
        
        auto send_goal_options = rclcpp_action::Client<MoveToPosition>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&ObjectPositionToActionConverter::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&ObjectPositionToActionConverter::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&ObjectPositionToActionConverter::result_callback, this, std::placeholders::_1);
        
        RCLCPP_INFO(this->get_logger(), "发送单轴运动 Action: 轴%d -> %.3f", axis_num, target_position);
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }
    
    // 发送多轴运动 Action
    void send_multi_axis_action(const std::vector<int32_t>& axes, const std::vector<float>& positions) {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action 服务器不可用");
            return;
        }
        
        auto goal_msg = MoveToPosition::Goal();
        goal_msg.target_axes = axes;
        goal_msg.target_positions = positions;
        goal_msg.speed = 50.0;
        goal_msg.acceleration = 100.0;
        goal_msg.deceleration = 100.0;
        
        auto send_goal_options = rclcpp_action::Client<MoveToPosition>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&ObjectPositionToActionConverter::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&ObjectPositionToActionConverter::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&ObjectPositionToActionConverter::result_callback, this, std::placeholders::_1);
        
        RCLCPP_INFO(this->get_logger(), "发送多轴运动 Action: %zu 个轴", axes.size());
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }
    
    // Action 回调函数
    void goal_response_callback(const GoalHandleMove::SharedPtr & goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "目标被拒绝");
        } else {
            RCLCPP_INFO(this->get_logger(), "目标被接受");
        }
    }
    
    void feedback_callback(
        GoalHandleMove::SharedPtr,
        const std::shared_ptr<const MoveToPosition::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "运动进度: %.1f%%", feedback->progress * 100);
    }
    
    void result_callback(const GoalHandleMove::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "运动完成，结果: %s", result.result->success ? "成功" : "失败");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "运动被中止");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "运动被取消");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "未知结果");
                break;
        }
    }
    
    rclcpp::Subscription<slms_interface::msg::ObjectPosition>::SharedPtr object_position_sub_;
    rclcpp_action::Client<MoveToPosition>::SharedPtr action_client_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectPositionToActionConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}