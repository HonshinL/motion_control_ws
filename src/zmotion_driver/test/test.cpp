#include <iostream>
#include <iomanip>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>  // 添加cstring头文件
#include "zmotion_driver/zmcaux.h"

class AxisMotionController {
private:
    ZMC_HANDLE handle_;
    bool is_connected_;
    std::string controller_ip_;
    
public:
    AxisMotionController() : handle_(nullptr), is_connected_(false) {}
    
    ~AxisMotionController() {
        disconnect();
    }
    
    bool connect(const std::string& ip) {
        if (is_connected_) {
            disconnect();
        }
        
        controller_ip_ = ip;
        char ip_buffer[16];
        std::strncpy(ip_buffer, ip.c_str(), sizeof(ip_buffer) - 1);
        ip_buffer[sizeof(ip_buffer) - 1] = '\0';
        
        int32 result = ZAux_OpenEth(ip_buffer, &handle_);
        if (result == ERR_OK) {
            is_connected_ = true;
            std::cout << "✅ 成功连接到控制器: " << ip << std::endl;
            return true;
        } else {
            handle_ = nullptr;
            is_connected_ = false;
            std::cout << "❌ 无法连接到控制器: " << ip << " (错误码: " << result << ")" << std::endl;
            return false;
        }
    }
    
    void disconnect() {
        if (is_connected_ && handle_) {
            ZAux_Close(handle_);
            handle_ = nullptr;
            is_connected_ = false;
            std::cout << "🔌 已断开与控制器的连接" << std::endl;
        }
    }
    
    bool isConnected() const {
        return is_connected_;
    }
    
    // 获取当前位置
    bool getCurrentPosition(int axis, float& position) {
        if (!is_connected_) return false;
        
        int32 result = ZAux_Direct_GetMpos(handle_, axis, &position);
        return result == ERR_OK;
    }
    
    // 获取目标位置 - 使用DPOS作为目标位置
    bool getTargetPosition(int axis, float& position) {
        if (!is_connected_) return false;
        
        int32 result = ZAux_Direct_GetDpos(handle_, axis, &position);
        return result == ERR_OK;
    }
    
    // 相对运动
    bool moveRelative(int axis, float distance, float speed = 50.0f, 
                     float acceleration = 100.0f, float deceleration = 100.0f) {
        if (!is_connected_) return false;
        
        // 设置运动参数
        if (ZAux_Direct_SetSpeed(handle_, axis, speed) != ERR_OK ||
            ZAux_Direct_SetAccel(handle_, axis, acceleration) != ERR_OK ||
            ZAux_Direct_SetDecel(handle_, axis, deceleration) != ERR_OK) {
            return false;
        }
        if (axis == 0)
        {
            ZAux_Direct_SetRevIn(handle_, axis, 5); 
            ZAux_Direct_SetInvertIn(handle_, 5, 1);
        }

        int is_idle = -1;        // 运动空闲状态：1=空闲/运动结束，0=运动中，-1=读取失败
        int alarm_status = 0;    // 告警状态：0=无告警，非0=告警码（参考官方手册）
        int enable_status = 0;   // 使能状态：bit0=1→已使能，0→未使能
        int ret = 0;             // 函数返回值：0=ERR_SUCCESS（成功），非0=失败

        // 1. 读取轴运动空闲状态（ZAux_Direct_GetIfIdle）
        ret = ZAux_Direct_GetIfIdle(handle_, axis, &is_idle);
        if (ret != ERR_SUCCESS) {
            fprintf(stderr, "【错误】轴%d: 读取运动状态失败！错误码：%d\n", axis, ret);
        }

        // 2. 读取轴告警状态（ZAux_Direct_GetAxisStatus）
        ret = ZAux_Direct_GetAxisStatus(handle_, axis, &alarm_status);
        if (ret != ERR_SUCCESS) {
            fprintf(stderr, "【错误】轴%d: 读取告警状态失败！错误码：%d\n", axis, ret);
        }

        // 3. 读取轴使能状态（ZAux_Direct_GetAxisStatus2）
        // ret = ZAux_Direct_GetAxisStatus2(handle_, axis, &enable_status);
        // if (ret != ERR_SUCCESS) {
        //     fprintf(stderr, "【错误】轴%d: 读取使能状态失败！错误码：%d\n", axis, ret);
        // }   

        // 执行相对运动
        int32 result = ERR_OK;
        if (axis == 1 || axis == 2) 
        {
            int master_axis = axis;       // 当前指令轴为主轴
            int slave_axis = 3 - axis;    // 另一个轴为从轴

            // 1. 建立跟随关系：从轴(slave) 叠加 主轴(master) 的运动，比例为 1.0
            ZAux_Direct_Single_Addax(handle_, master_axis, slave_axis);
            
            // 2. 启动主轴运动（使用绝对运动 MoveAbs 或 相对运动 Move）
            // 注意：这里建议统一使用 handle_ 或 g_handle
            int32 result = ZAux_Direct_Single_Move(handle_, master_axis, distance);
        }
        else 
        {
            // 3. 普通单轴运动逻辑
            // 如果不是 1、2 号轴，则直接进行单轴相对运动
            int32 result = ZAux_Direct_Single_Move(handle_, axis, distance);
        }
        return result == ERR_OK;
    }
    
    // 检查运动是否完成
    bool isMotionCompleted(int axis, float tolerance = 0.001f) {
        if (!is_connected_) return false;
        
        float current_pos = 0.0f;
        float target_pos = 0.0f;
        
        if (!getCurrentPosition(axis, current_pos) || !getTargetPosition(axis, target_pos)) {
            return false;
        }
        
        return std::abs(current_pos - target_pos) <= tolerance;
    }
    
    // 实时监控运动进度
    void monitorMotion(int axis, float target_distance, std::atomic<bool>& stop_monitor) {
        float start_position = 0.0f;
        if (!getCurrentPosition(axis, start_position)) {
            std::cout << "❌ 无法获取起始位置" << std::endl;
            return;
        }
        
        float target_position = start_position + target_distance;
        
        std::cout << "\n🚀 开始监控轴 " << axis << " 的运动进度" << std::endl;
        std::cout << "起始位置: " << std::fixed << std::setprecision(3) << start_position << std::endl;
        std::cout << "目标位置: " << std::fixed << std::setprecision(3) << target_position << std::endl;
        std::cout << "运动距离: " << std::fixed << std::setprecision(3) << target_distance << std::endl;
        std::cout << std::endl;
        
        auto start_time = std::chrono::steady_clock::now();
        
        while (!stop_monitor.load() && !isMotionCompleted(axis)) {
            float current_pos = 0.0f;
            if (!getCurrentPosition(axis, current_pos)) {
                std::cout << "❌ 无法读取当前位置" << std::endl;
                break;
            }
            
            float progress = (current_pos - start_position) / target_distance * 100.0f;
            progress = std::max(0.0f, std::min(100.0f, progress)); // 限制在0-100%
            
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
            
            // 清空当前行并显示进度
            std::cout << "\r" << "📊 进度: " << std::setw(6) << std::fixed << std::setprecision(2) << progress 
                      << "% | 当前位置: " << std::setw(10) << current_pos 
                      << " | 耗时: " << elapsed.count() << "s" << std::flush;
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        if (!stop_monitor.load()) {
            std::cout << "\n✅ 运动完成!" << std::endl;
        } else {
            std::cout << "\n⏹️ 运动被中断" << std::endl;
        }
    }
};

void printUsage(const char* program_name) {
    std::cout << "用法: " << program_name << " <轴号> <相对距离> [速度] [加速度] [减速度]" << std::endl;
    std::cout << "示例:" << std::endl;
    std::cout << "  " << program_name << " 0 100.0           # 轴0移动100单位" << std::endl;
    std::cout << "  " << program_name << " 1 -50.0 30.0     # 轴1反向移动50单位，速度30" << std::endl;
    std::cout << "  " << program_name << " 2 200.0 50 100 100 # 完整参数设置" << std::endl;
    std::cout << "\n轴号范围: 0, 1, 2, 4, 5" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }
    
    // 解析参数
    int axis = std::atoi(argv[1]);
    float distance = std::atof(argv[2]);
    
    // 检查轴号有效性
    int valid_axes[] = {0, 1, 2, 4, 5};
    bool valid_axis = false;
    for (int a : valid_axes) {
        if (axis == a) {
            valid_axis = true;
            break;
        }
    }
    
    if (!valid_axis) {
        std::cout << "❌ 无效的轴号: " << axis << "，有效轴号为: 0, 1, 2, 4, 5" << std::endl;
        return 1;
    }
    
    // 设置运动参数（使用默认值或用户输入）
    float speed = 50.0f;
    float acceleration = 100.0f;
    float deceleration = 100.0f;
    
    if (argc >= 4) speed = std::atof(argv[3]);
    if (argc >= 5) acceleration = std::atof(argv[4]);
    if (argc >= 6) deceleration = std::atof(argv[5]);
    
    // 创建控制器实例
    AxisMotionController controller;
    
    // 连接控制器
    if (!controller.connect("192.168.0.11")) {
        return 1;
    }
    
    // 显示初始位置
    float initial_position = 0.0f;
    if (controller.getCurrentPosition(axis, initial_position)) {
        std::cout << "📍 轴 " << axis << " 初始位置: " << std::fixed << std::setprecision(3) << initial_position << std::endl;
    }
    
    // 启动监控线程
    std::atomic<bool> stop_monitor{false};
    std::thread monitor_thread(&AxisMotionController::monitorMotion, &controller, 
                              axis, distance, std::ref(stop_monitor));
    
    // 等待监控线程启动
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 执行运动
    std::cout << "\n🎯 开始执行相对运动..." << std::endl;
    std::cout << "轴号: " << axis << std::endl;
    std::cout << "距离: " << std::fixed << std::setprecision(3) << distance << std::endl;
    std::cout << "速度: " << speed << std::endl;
    std::cout << "加速度: " << acceleration << std::endl;
    std::cout << "减速度: " << deceleration << std::endl;
    
    bool motion_success = controller.moveRelative(axis, distance, speed, acceleration, deceleration);
    
    if (!motion_success) {
        std::cout << "❌ 运动启动失败!" << std::endl;
        stop_monitor.store(true);
    } else {
        std::cout << "✅ 运动已启动，正在执行..." << std::endl;
    }
    
    // 等待监控线程完成
    monitor_thread.join();
    
    // 显示最终结果
    float final_position = 0.0f;
    if (controller.getCurrentPosition(axis, final_position)) {
        std::cout << "\n📋 运动结果报告:" << std::endl;
        std::cout << "轴号: " << axis << std::endl;
        std::cout << "起始位置: " << std::fixed << std::setprecision(3) << initial_position << std::endl;
        std::cout << "最终位置: " << std::fixed << std::setprecision(3) << final_position << std::endl;
        std::cout << "实际移动距离: " << std::fixed << std::setprecision(3) << (final_position - initial_position) << std::endl;
        std::cout << "目标移动距离: " << std::fixed << std::setprecision(3) << distance << std::endl;
        
        if (motion_success && controller.isMotionCompleted(axis)) {
            std::cout << "🎉 运动成功完成!" << std::endl;
        } else {
            std::cout << "⚠️ 运动未完全完成" << std::endl;
        }
    }
    
    return motion_success ? 0 : 1;
}