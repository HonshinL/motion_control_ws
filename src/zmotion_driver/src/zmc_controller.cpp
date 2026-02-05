#include "zmotion_driver/zmc_controller.h"
#include <cstring>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <thread>
#include <atomic>

ZmcController::ZmcController(const std::string& node_name) : Node(node_name), handle_(nullptr), is_connected_(false), connecting_(false) {
    initROS();
}

ZmcController::~ZmcController() {
    disconnect();
}

bool ZmcController::connect(const std::string& ip) {
    // 确保之前的连接已关闭
    if (is_connected_) {
        disconnect();
    }

    controller_ip_ = ip;
    
    // 准备IP地址缓冲区
    char ip_buffer[16]; // IPv4地址最多需要15个字符
    std::strncpy(ip_buffer, ip.c_str(), sizeof(ip_buffer) - 1);
    ip_buffer[sizeof(ip_buffer) - 1] = '\0'; // 确保字符串以null结尾

    // 连接控制器
    int32 result = ZAux_OpenEth(ip_buffer, &handle_);
    if (checkError(result)) {
        is_connected_ = true;
        RCLCPP_INFO(this->get_logger(), "成功连接到ZMC控制器: %s", ip.c_str());
        return true;
    } else {
        handle_ = nullptr;
        is_connected_ = false;
        RCLCPP_ERROR(this->get_logger(), "无法连接到ZMC控制器: %s", ip.c_str());
        return false;
    }
}

void ZmcController::disconnect() {
    if (is_connected_ && handle_) {
        ZAux_Close(handle_);
        handle_ = nullptr;
        is_connected_ = false;
        RCLCPP_INFO(this->get_logger(), "已断开与ZMC控制器的连接: %s", controller_ip_.c_str());
    }
}

bool ZmcController::isConnected() const {
    return is_connected_;
}

ZMC_HANDLE ZmcController::getHandle() const {
    return handle_;
}

// 位置相关方法
bool ZmcController::getDpos(int axis, float& position) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_GetDpos(handle_, axis, &position);
    return checkError(result);
}

bool ZmcController::setDpos(int axis, float position) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_SetDpos(handle_, axis, position);
    return checkError(result);
}

bool ZmcController::getMpos(int axis, float& position) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_GetMpos(handle_, axis, &position);
    return checkError(result);
}

// 速度相关方法
bool ZmcController::getCurSpeed(int axis, float& speed) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_GetVpSpeed(handle_, axis, &speed);
    return checkError(result);
}

bool ZmcController::getAccel(int axis, float& accel) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_GetAccel(handle_, axis, &accel);
    return checkError(result);
}

bool ZmcController::setAccel(int axis, float accel) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_SetAccel(handle_, axis, accel);
    return checkError(result);
}

bool ZmcController::getDecel(int axis, float& decel) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_GetDecel(handle_, axis, &decel);
    return checkError(result);
}

bool ZmcController::setDecel(int axis, float decel) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_SetDecel(handle_, axis, decel);
    return checkError(result);
}

// IO相关方法
bool ZmcController::getInput(int ionum, uint32& value) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_GetIn(handle_, ionum, &value);
    return checkError(result);
}

bool ZmcController::setOutput(int ionum, uint32 value) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_SetOp(handle_, ionum, value);
    return checkError(result);
}

bool ZmcController::getOutput(int ionum, uint32& value) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_GetOp(handle_, ionum, &value);
    return checkError(result);
}

// 轴状态相关方法
bool ZmcController::getAxisStatus(int axis, int& status) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_GetAxisStatus(handle_, axis, &status);
    return checkError(result);
}

bool ZmcController::getAxisEnable(int axis, int& enabled) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_GetAxisEnable(handle_, axis, &enabled);
    return checkError(result);
}

bool ZmcController::setAxisEnable(int axis, int enabled) {
    if (!is_connected_) return false;
    
    int32 result = ZAux_Direct_SetAxisEnable(handle_, axis, enabled);
    return checkError(result);
}

// 新方法实现
void ZmcController::initROS() {
    // 声明并获取参数
    std::string ip = this->declare_parameter<std::string>("controller_ip", "192.168.0.11");
    axis_ = this->declare_parameter<int>("monitoring_axis", 0);
    connect_search_timeout_ms_ = this->declare_parameter<int>("controller_connect_search_timeout_ms", 10000);
    
    // 初始化轴列表（假设支持4个轴）
    axes_ = {0, 1, 2, 4, 5}; // 示例轴号，根据实际情况调整

    // 创建发布者 (Publisher)
    // 发布运动状态
    motion_status_pub_ = this->create_publisher<motion_msgs::msg::MotionStatus>("zmc_pub/motion_status", 10);

    // 创建DXF到XML转换服务
    convert_dxf_to_xml_service_ = this->create_service<motion_msgs::srv::ConvertDxfToXml>(
        "zmc_srv/convert_dxf_to_xml",
        std::bind(&ZmcController::handleConvertDxfToXml, this, std::placeholders::_1, std::placeholders::_2));

    // 创建DXF转换状态发布者（异步任务状态通知）
    convert_status_pub_ = this->create_publisher<std_msgs::msg::String>("zmc_pub/convert_dxf_to_xml/status", 10);

    // 创建移动到目标位置Action服务器
    move_to_position_action_server_ = rclcpp_action::create_server<motion_msgs::action::MoveToPosition>(
        this,
        "zmc_act/move_to_position",
        std::bind(&ZmcController::handleMoveToPositionGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ZmcController::handleMoveToPositionCancel, this, std::placeholders::_1),
        std::bind(&ZmcController::handleMoveToPositionAccepted, this, std::placeholders::_1));
    
    // 初始化Action状态
    action_running_ = false;

    // 不在构造/初始化阶段进行阻塞性连接，使用显式的 start() 方法进行连接和启动发布
}

void ZmcController::start() {
    if (connecting_.load()) {
        RCLCPP_WARN(this->get_logger(), "连接已在进行中，跳过本次连接尝试");
        return;
    }

    if (is_connected_) {
        RCLCPP_INFO(this->get_logger(), "控制器已连接，无需重新连接");
        return;
    }

    connecting_.store(true);
    
    // 直接使用固定IP地址 192.168.0.11
    std::string ip = "192.168.0.11";
    
    RCLCPP_INFO(this->get_logger(), "开始尝试连接控制器: %s", ip.c_str());

    std::thread([this, ip]() {
        // 直接尝试连接，不进行搜索
        bool connect_success = connect(ip);
        
        if (connect_success) {
            RCLCPP_INFO(this->get_logger(), "✅ 成功连接到控制器: %s", ip.c_str());
            RCLCPP_INFO(this->get_logger(), "📊 开始监控 %d 个轴: [%d, %d, %d, %d, %d]", 
                       NUM_AXES, AXES[0], AXES[1], AXES[2], AXES[3], AXES[4]);
            
            // 启动数据发布
            startPublishing();
            
            RCLCPP_INFO(this->get_logger(), "🚀 控制器已启动并开始发布数据");
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ 连接控制器失败: %s", ip.c_str());
            RCLCPP_ERROR(this->get_logger(), "💡 请检查以下事项:");
            RCLCPP_ERROR(this->get_logger(), "  1. 控制器电源是否打开");
            RCLCPP_ERROR(this->get_logger(), "  2. 网络连接是否正常");
            RCLCPP_ERROR(this->get_logger(), "  3. IP地址 %s 是否正确", ip.c_str());
            RCLCPP_ERROR(this->get_logger(), "  4. 防火墙设置是否允许连接");
        }

        connecting_.store(false);
    }).detach();
}

void ZmcController::stop() {
    stopPublishing();
    disconnect();
}

void ZmcController::startPublishing() {
    // 创建定时器 (WallTimer)
    // 每 20 毫秒执行一次 timer_callback (50Hz)
    if (!timer_) {
        timer_ = this->create_wall_timer(20ms, std::bind(&ZmcController::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "开始发布控制器数据");
    }
}

void ZmcController::stopPublishing() {
    if (timer_) {
        timer_.reset();
        RCLCPP_INFO(this->get_logger(), "停止发布控制器数据");
    }
}

void ZmcController::timer_callback() {
    if (!is_connected_) return;

    // 创建MotionStatus消息
    auto motion_status_msg = motion_msgs::msg::MotionStatus();
    
    // 直接设置JointState的Header，避免重复
    auto& joint_state = motion_status_msg.joint_state;
    joint_state.header.stamp = this->now();
    joint_state.header.frame_id = "zmc_status";
    
    bool all_axes_success = true;
    
    // 读取所有轴的数据
    for (int axis : axes_) {
        float dpos_val = 0.0;
        float mpos_val = 0.0;
        float speed_val = 0.0;
        
        // 从硬件读取数据
        bool axis_success = getDpos(axis, dpos_val) && getMpos(axis, mpos_val) && getCurSpeed(axis, speed_val);
        
        if (axis_success) {
            // 设置关节名称
            joint_state.name.push_back("axis_" + std::to_string(axis));
            
            // 设置关节位置 (使用MPOS作为实际位置)
            joint_state.position.push_back(mpos_val);
            
            // 设置关节速度
            joint_state.velocity.push_back(speed_val);
            
            // 设置关节加速度 (暂不支持，留空)
            // joint_state.effort.push_back(0.0); // 不支持力/力矩
            
            // 调试信息
            RCLCPP_DEBUG(this->get_logger(), "轴 %d: MPOS=%.3f, Speed=%.3f", axis, mpos_val, speed_val);
        } else {
            RCLCPP_WARN(this->get_logger(), "无法读取轴 %d 的数据", axis);
            all_axes_success = false;
        }
    }
    
    // 如果至少有一个轴读取成功，则发布消息
    if (!joint_state.name.empty()) {
        motion_status_pub_->publish(motion_status_msg);
        
        if (all_axes_success) {
            RCLCPP_DEBUG(this->get_logger(), "成功发布 %zu 个轴的状态数据", joint_state.name.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "部分轴数据读取失败，成功发布 %zu 个轴的状态数据", joint_state.name.size());
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "所有轴数据读取失败，无法发布状态消息");
    }
}

// DXF到XML转换服务实现
void ZmcController::handleConvertDxfToXml(const std::shared_ptr<motion_msgs::srv::ConvertDxfToXml::Request> request,
                                         std::shared_ptr<motion_msgs::srv::ConvertDxfToXml::Response> response) {
    
    std::string dxf_file_path = request->dxf_file_path;
    
    RCLCPP_INFO(this->get_logger(), "收到DXF到XML转换请求，文件路径: %s", dxf_file_path.c_str());
    
    // 检查输入文件是否存在
    if (!std::filesystem::exists(dxf_file_path)) {
        response->success = false;
        response->message = "输入DXF文件不存在: " + dxf_file_path;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    
    // 检查文件扩展名是否为.dxf
    std::filesystem::path input_path(dxf_file_path);
    if (input_path.extension() != ".dxf" && input_path.extension() != ".DXF") {
        response->success = false;
        response->message = "输入文件不是DXF格式，扩展名应为.dxf: " + dxf_file_path;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    
    // 生成输出XML文件路径（与输入文件相同目录，文件名相同但扩展名为.xml）
    std::filesystem::path output_path = input_path.parent_path() / input_path.stem().concat(".xml");
    std::string xml_file_path = output_path.string();
    
    // 发布状态：启动
    std_msgs::msg::String start_msg;
    start_msg.data = "started: " + dxf_file_path + " -> " + xml_file_path;
    convert_status_pub_->publish(start_msg);

    // 启动异步转换任务（立即返回响应，实际转换在后台执行）
    std::string input = dxf_file_path;
    std::string output = xml_file_path;
    std::thread([this, input, output]() {
        try {
            auto start_time = std::chrono::high_resolution_clock::now();
            std::ifstream dxf_file(input, std::ios::binary);
            if (!dxf_file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "无法打开DXF文件: %s", input.c_str());
                std_msgs::msg::String fail_msg;
                fail_msg.data = "failed: cannot open input: " + input;
                convert_status_pub_->publish(fail_msg);
                return;
            }
            std::ofstream xml_file(output);
            if (!xml_file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "无法创建XML文件: %s", output.c_str());
                dxf_file.close();
                std_msgs::msg::String fail_msg;
                fail_msg.data = "failed: cannot create output: " + output;
                convert_status_pub_->publish(fail_msg);
                return;
            }
            xml_file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
            xml_file << "<!-- Converted from DXF file: " << std::filesystem::path(input).filename().string() << " -->\n";
            xml_file << "<MotionConfig>\n";
            std::string line;
            int line_count = 0;
            while (std::getline(dxf_file, line)) {
                line_count++;
                if (line.empty()) continue;
                if (line[0] == '#') {
                    xml_file << "<!-- " << line.substr(1) << " -->\n";
                    continue;
                }
                size_t equals_pos = line.find('=');
                if (equals_pos != std::string::npos) {
                    std::string key = line.substr(0, equals_pos);
                    std::string value = line.substr(equals_pos + 1);
                    key.erase(0, key.find_first_not_of(" \t\n\r\f\v"));
                    key.erase(key.find_last_not_of(" \t\n\r\f\v") + 1);
                    value.erase(0, value.find_first_not_of(" \t\n\r\f\v"));
                    value.erase(value.find_last_not_of(" \t\n\r\f\v") + 1);
                    xml_file << "  <" << key << ">" << value << "</" << key << ">\n";
                } else {
                    xml_file << "  <!-- DXF Line " << line_count << ": " << line << " -->\n";
                }
            }
            xml_file << "</MotionConfig>\n";
            dxf_file.close();
            xml_file.close();
            auto end_time = std::chrono::high_resolution_clock::now();
            double duration_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end_time - start_time).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "格式转换耗时: " << std::fixed << std::setprecision(3) << duration_ms << " ms");
            RCLCPP_INFO(this->get_logger(), "异步转换完成，输出文件: %s", output.c_str());

            std_msgs::msg::String done_msg;
            done_msg.data = "completed: " + output + " lines=" + std::to_string(line_count);
            convert_status_pub_->publish(done_msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "异步转换过程中发生错误: %s", e.what());
            std_msgs::msg::String err_msg;
            err_msg.data = std::string("failed: exception: ") + e.what();
            convert_status_pub_->publish(err_msg);
        }
    }).detach();

    response->success = true;
    response->message = "已启动异步转换，输出文件: " + xml_file_path;
    response->xml_file_path = xml_file_path;

    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    RCLCPP_INFO(this->get_logger(), "输出XML文件路径: %s", xml_file_path.c_str());
    return;
}

// Action相关方法实现

// 处理Action目标请求
rclcpp_action::GoalResponse ZmcController::handleMoveToPositionGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const motion_msgs::action::MoveToPosition::Goal> goal) {
    
    RCLCPP_INFO(this->get_logger(), "收到移动到目标位置Action请求");
    
    // 检查控制器是否连接
    if (!is_connected_) {
        RCLCPP_ERROR(this->get_logger(), "控制器未连接，拒绝Action请求");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // 检查是否有正在执行的Action
    if (action_running_) {
        RCLCPP_WARN(this->get_logger(), "已有Action正在执行，拒绝新请求");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // 检查目标参数有效性
    if (goal->target_axes.empty() || goal->target_positions.empty()) {
        RCLCPP_ERROR(this->get_logger(), "目标轴号或位置列表为空，拒绝Action请求");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    if (goal->target_axes.size() != goal->target_positions.size()) {
        RCLCPP_ERROR(this->get_logger(), "目标轴号数量(%zu)与位置数量(%zu)不匹配", 
                     goal->target_axes.size(), goal->target_positions.size());
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    if (goal->speed <= 0 || goal->acceleration <= 0 || goal->deceleration <= 0) {
        RCLCPP_ERROR(this->get_logger(), "速度或加速度参数无效，拒绝Action请求");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    RCLCPP_INFO(this->get_logger(), "接受移动到目标位置Action请求");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 处理Action取消请求
rclcpp_action::CancelResponse ZmcController::handleMoveToPositionCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_msgs::action::MoveToPosition>> goal_handle) {
    
    RCLCPP_INFO(this->get_logger(), "收到Action取消请求");
    
    if (action_running_ && current_goal_handle_ == goal_handle) {
        // 停止所有轴的运动
        for (int axis : axes_) {
            ZAux_Direct_Single_Cancel(handle_, axis, 0);
        }
        
        action_running_ = false;
        RCLCPP_INFO(this->get_logger(), "Action已取消");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    RCLCPP_WARN(this->get_logger(), "没有正在执行的Action可以取消");
    return rclcpp_action::CancelResponse::REJECT;
}

// 接受并执行Action
void ZmcController::handleMoveToPositionAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_msgs::action::MoveToPosition>> goal_handle) {
    
    action_running_ = true;
    current_goal_handle_ = goal_handle;
    
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<motion_msgs::action::MoveToPosition::Result>();
    auto feedback = std::make_shared<motion_msgs::action::MoveToPosition::Feedback>();
    
    RCLCPP_INFO(this->get_logger(), "开始执行移动到目标位置Action");
    
    try {
        // 设置运动参数
        for (size_t i = 0; i < goal->target_axes.size(); ++i) {
            int axis = goal->target_axes[i];
            
            // 设置轴速度
            if (!checkError(ZAux_Direct_SetSpeed(handle_, axis, goal->speed))) {
                throw std::runtime_error("设置轴 " + std::to_string(axis) + " 速度失败");
            }
            
            // 设置加速度
            if (!checkError(ZAux_Direct_SetAccel(handle_, axis, goal->acceleration))) {
                throw std::runtime_error("设置轴 " + std::to_string(axis) + " 加速度失败");
            }
            
            // 设置减速度
            if (!checkError(ZAux_Direct_SetDecel(handle_, axis, goal->deceleration))) {
                throw std::runtime_error("设置轴 " + std::to_string(axis) + " 减速度失败");
            }
        }
        
        // 启动运动
        for (size_t i = 0; i < goal->target_axes.size(); ++i) {
            int axis = goal->target_axes[i];
            float target_position = goal->target_positions[i];
            
            if (!checkError(ZAux_Direct_Single_MoveAbs(handle_, axis, target_position))) {
                throw std::runtime_error("设置轴 " + std::to_string(axis) + " 目标位置失败");
            }
            
            RCLCPP_INFO(this->get_logger(), "轴 %d 开始移动到位置 %.3f", axis, target_position);
        }
        
        // 监控运动过程
        bool all_axes_completed = false;
        auto start_time = this->now();
        
        while (action_running_ && !all_axes_completed) {
            // 检查Action是否被取消
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Action被用户取消";
                goal_handle->canceled(result);
                action_running_ = false;
                RCLCPP_INFO(this->get_logger(), "Action执行被取消");
                return;
            }
            
            // 更新反馈信息
            feedback->current_positions.clear();
            feedback->current_velocities.clear();
            
            int completed_axes = 0;
            all_axes_completed = true;
            
            for (size_t i = 0; i < goal->target_axes.size(); ++i) {
                int axis = goal->target_axes[i];
                float target_position = goal->target_positions[i];
                
                float current_position = 0.0;
                float current_velocity = 0.0;
                
                // 读取当前位置和速度
                if (getMpos(axis, current_position) && getCurSpeed(axis, current_velocity)) {
                    feedback->current_positions.push_back(current_position);
                    feedback->current_velocities.push_back(current_velocity);
                    
                    // 检查是否到达目标位置
                    if (isAxisAtPosition(axis, target_position)) {
                        completed_axes++;
                    } else {
                        all_axes_completed = false;
                    }
                } else {
                    all_axes_completed = false;
                    RCLCPP_WARN(this->get_logger(), "无法读取轴 %d 的当前位置和速度", axis);
                }
            }
            
            // 计算进度
            feedback->progress = static_cast<float>(completed_axes) / goal->target_axes.size();
            feedback->current_status = "移动中，已完成 " + std::to_string(completed_axes) + "/" + 
                                      std::to_string(goal->target_axes.size()) + " 个轴";
            
            // 发布反馈
            goal_handle->publish_feedback(feedback);
            
            // 短暂休眠，避免过度占用CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        if (action_running_) {
            // 所有轴都到达目标位置
            result->end_time = this->now();
            result->success = true;
            result->message = "所有轴成功到达目标位置";
            result->final_positions = feedback->current_positions;
            
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Action执行成功完成");
        }
        
    } catch (const std::exception& e) {
        result->end_time = this->now();
        result->success = false;
        result->message = "Action执行失败: " + std::string(e.what());
        
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Action执行失败: %s", e.what());
    }
    
    action_running_ = false;
}

// 执行单轴移动
bool ZmcController::moveSingleAxis(int axis, float target_position, float speed, float acceleration, float deceleration) {
    if (!is_connected_) return false;
    
    // 设置运动参数
    if (!checkError(ZAux_Direct_SetSpeed(handle_, axis, speed)) ||
        !checkError(ZAux_Direct_SetAccel(handle_, axis, acceleration)) ||
        !checkError(ZAux_Direct_SetDecel(handle_, axis, deceleration)) ||
        !checkError(ZAux_Direct_Single_MoveAbs(handle_, axis, target_position))) {
        return false;
    }
    
    return true;
}

// 检查轴是否到达目标位置
bool ZmcController::isAxisAtPosition(int axis, float target_position, float tolerance) {
    if (!is_connected_) return false;
    
    float current_position = 0.0;
    if (!getMpos(axis, current_position)) {
        return false;
    }
    
    return std::abs(current_position - target_position) <= tolerance;
}

// 私有方法
bool ZmcController::checkError(int32 error_code) const {
    if (error_code == ERR_OK) {
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "ZMC控制器错误: 错误码 = %d", error_code);
        return false;
    }
}