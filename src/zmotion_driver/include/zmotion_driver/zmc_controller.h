#ifndef ZMC_CONTROLLER_H
#define ZMC_CONTROLLER_H

#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <atomic>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "motion_msgs/msg/motion_status.hpp"
#include "motion_msgs/msg/object_position.hpp"
#include "motion_msgs/srv/convert_dxf_to_xml.hpp"
#include "motion_msgs/action/move_to_position.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "zmotion_driver/zmcaux.h"

using namespace std::chrono_literals;

class ZmcController : public rclcpp::Node {
public:
    ZmcController(const std::string& node_name = "zmc_node");
    ~ZmcController();

    /**
     * @brief 通过以太网连接到ZMC控制器
     * @param ip 控制器IP地址
     * @return true 连接成功，false 连接失败
     */
    bool connect(const std::string& ip);

    /**
     * @brief 断开与控制器的连接
     */
    void disconnect();

    /**
     * @brief 检查控制器是否已连接
     * @return true 已连接，false 未连接
     */
    bool isConnected() const;

    /**
     * @brief 获取控制器句柄
     * @return 控制器句柄
     */
    ZMC_HANDLE getHandle() const;

    /**
     * @brief 初始化ROS2相关功能（参数、发布者、定时器）
     */
    void initROS();

    /**
     * @brief 启动数据发布循环
     */
    void startPublishing();

    /**
     * @brief 停止数据发布循环
     */
    void stopPublishing();

    /**
     * @brief 显式启动控制器（连接并开始发布）
     */
    void start();

    /**
     * @brief 停止控制器（停止发布并断开连接）
     */
    void stop();

    // DXF到XML转换服务
    /**
     * @brief 处理DXF到XML转换的服务请求
     * @param request 服务请求
     * @param response 服务响应
     */
    void handleConvertDxfToXml(const std::shared_ptr<motion_msgs::srv::ConvertDxfToXml::Request> request,
                              std::shared_ptr<motion_msgs::srv::ConvertDxfToXml::Response> response);

    // 移动到目标位置Action
    /**
     * @brief 处理移动到目标位置的Action请求
     * @param goal_handle Action目标句柄
     */
    rclcpp_action::GoalResponse handleMoveToPositionGoal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const motion_msgs::action::MoveToPosition::Goal> goal);
    
    /**
     * @brief 处理Action取消请求
     * @param goal_handle Action目标句柄
     */
    rclcpp_action::CancelResponse handleMoveToPositionCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_msgs::action::MoveToPosition>> goal_handle);
    
    /**
     * @brief 处理Action接受回调
     * @param goal_handle Action目标句柄
     */
    void handleMoveToPositionAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_msgs::action::MoveToPosition>> goal_handle);
    
    /**
     * @brief 执行移动到目标位置的Action（异步执行）
     * @param goal_handle Action目标句柄
     */
    void executeMoveToPosition(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_msgs::action::MoveToPosition>> goal_handle);
    
    /**
     * @brief 执行单轴移动
     * @param axis 轴号
     * @param target_position 目标位置
     * @param speed 移动速度
     * @param acceleration 加速度
     * @param deceleration 减速度
     * @return 是否成功
     */
    bool moveSingleAxis(int axis, float target_position, float speed, float acceleration, float deceleration);
    
    /**
     * @brief 检查轴是否到达目标位置
     * @param axis 轴号
     * @param target_position 目标位置
     * @param tolerance 容差
     * @return 是否到达
     */
    bool isAxisAtPosition(int axis, float target_position, float tolerance = 0.001);
    
    /**
     * @brief 处理ObjectPosition消息，移动结构到目标位置
     * @param msg 接收到的ObjectPosition消息
     */
    void handleObjectPosition(const motion_msgs::msg::ObjectPosition::SharedPtr msg);
    
    /**
     * @brief 执行多轴运动到目标位置
     * @param target_positions 目标位置数组
     * @param speed 移动速度
     * @param acceleration 加速度
     * @param deceleration 减速度
     * @return 是否成功启动运动
     */
    bool moveToPositions(const std::vector<float>& target_positions, float speed = 50.0f, 
                        float acceleration = 100.0f, float deceleration = 100.0f);
    
    /**
     * @brief 监控多轴运动进度（内部方法）
     * @param target_positions 目标位置数组
     */
    void monitorMultiAxisMotion(const std::vector<float>& target_positions);

    // 位置相关方法
    /**
     * @brief 获取指定轴的命令位置
     * @param axis 轴号
     * @param position 返回的命令位置
     * @return true 成功，false 失败
     */
    bool getDpos(int axis, float& position);

    /**
     * @brief 设置指定轴的命令位置
     * @param axis 轴号
     * @param position 命令位置
     * @return true 成功，false 失败
     */
    bool setDpos(int axis, float position);

    /**
     * @brief 获取指定轴的反馈位置
     * @param axis 轴号
     * @param position 返回的反馈位置
     * @return true 成功，false 失败
     */
    bool getMpos(int axis, float& position);

    // 速度相关方法
    /**
     * @brief 获取指定轴的当前速度
     * @param axis 轴号
     * @param speed 返回的当前速度
     * @return true 成功，false 失败
     */
    bool getCurSpeed(int axis, float& speed);

    /**
     * @brief 获取指定轴的加速度
     * @param axis 轴号
     * @param accel 返回的加速度
     * @return true 成功，false 失败
     */
    bool getAccel(int axis, float& accel);

    /**
     * @brief 设置指定轴的加速度
     * @param axis 轴号
     * @param accel 加速度
     * @return true 成功，false 失败
     */
    bool setAccel(int axis, float accel);

    /**
     * @brief 获取指定轴的减速度
     * @param axis 轴号
     * @param decel 返回的减速度
     * @return true 成功，false 失败
     */
    bool getDecel(int axis, float& decel);

    /**
     * @brief 设置指定轴的减速度
     * @param axis 轴号
     * @param decel 减速度
     * @return true 成功，false 失败
     */
    bool setDecel(int axis, float decel);

    // IO相关方法
    /**
     * @brief 读取输入信号
     * @param ionum 输入口编号
     * @param value 返回的输入口状态
     * @return true 成功，false 失败
     */
    bool getInput(int ionum, uint32& value);

    /**
     * @brief 设置输出信号
     * @param ionum 输出口编号
     * @param value 输出口状态
     * @return true 成功，false 失败
     */
    bool setOutput(int ionum, uint32 value);

    /**
     * @brief 读取输出信号
     * @param ionum 输出口编号
     * @param value 返回的输出口状态
     * @return true 成功，false 失败
     */
    bool getOutput(int ionum, uint32& value);

    // 轴状态相关方法
    /**
     * @brief 获取指定轴的状态
     * @param axis 轴号
     * @param status 返回的轴状态
     * @return true 成功，false 失败
     */
    bool getAxisStatus(int axis, int& status);

    /**
     * @brief 获取指定轴的使能状态
     * @param axis 轴号
     * @param enabled 返回的使能状态
     * @return true 成功，false 失败
     */
    bool getAxisEnable(int axis, int& enabled);

    /**
     * @brief 设置指定轴的使能状态
     * @param axis 轴号
     * @param enabled 使能状态
     * @return true 成功，false 失败
     */
    bool setAxisEnable(int axis, int enabled);

private:
    /**
     * @brief 定时器回调函数，定期读取并发布数据
     */
    void timer_callback();

    /**
     * @brief 检查错误码并返回结果
     * @param error_code ZMC API返回的错误码
     * @return true 成功，false 失败
     */
    bool checkError(int32 error_code) const;

    // ZMC控制器相关成员
    ZMC_HANDLE handle_;      ///< 控制器句柄
    bool is_connected_;      ///< 连接状态
    std::atomic<bool> connecting_; ///< 正在连接标志
    std::string controller_ip_;  ///< 控制器IP地址
    int connect_search_timeout_ms_{1000}; ///< 搜索超时 (ms)

    // ROS2相关成员
    int axis_;  ///< 监控的轴号
    std::vector<int> axes_;  ///< 轴列表
    rclcpp::TimerBase::SharedPtr timer_;  ///< 定时器
    rclcpp::Publisher<motion_msgs::msg::MotionStatus>::SharedPtr motion_status_pub_;  ///< 运动状态发布者
    rclcpp::Subscription<motion_msgs::msg::ObjectPosition>::SharedPtr object_position_sub_;  ///< 目标位置订阅者
    rclcpp::Service<motion_msgs::srv::ConvertDxfToXml>::SharedPtr convert_dxf_to_xml_service_;  ///< DXF到XML转换服务
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr convert_status_pub_;  ///< DXF->XML 转换状态发布者
    rclcpp_action::Server<motion_msgs::action::MoveToPosition>::SharedPtr move_to_position_action_server_;  ///< 移动到目标位置Action服务器
    
    // Action执行状态
    std::atomic<bool> action_running_;  ///< Action是否正在执行
    std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_msgs::action::MoveToPosition>> current_goal_handle_;  ///< 当前Action目标句柄
    
    static constexpr int NUM_AXES = 5;  ///< 轴数量
    static constexpr int AXES[NUM_AXES] = {0, 1, 2, 4, 5};  ///< 轴列表定义
};

#endif // ZMC_CONTROLLER_H