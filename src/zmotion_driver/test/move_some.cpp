#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include "zmotion_driver/zmcaux.h"

int main() {
    ZMC_HANDLE handle = NULL;
    char ip[] = "192.168.0.11"; 
    
    if (ZAux_OpenEth(ip, &handle) != 0) {
        std::cerr << "错误: 无法连接控制器 " << ip << std::endl;
        return -1;
    }

    int axis = 0; 
    float target_distance = 10.0; // 设定测试距离为 10 mm/Units

    // 1. 运动参数设置
    ZAux_Direct_SetUnits(handle, axis, 1000.0); 
    ZAux_Direct_SetSpeed(handle, axis, 5.0);    // 低速运动 5mm/s
    ZAux_Direct_SetAccel(handle, axis, 50.0);  
    ZAux_Direct_SetDecel(handle, axis, 50.0);
    ZAux_Direct_SetSramp(handle, axis, 20.0);   // 增加 S 曲线平滑，减少机械冲击

    // 2. 轴使能及准备
    ZAux_Direct_SetAxisEnable(handle, axis, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 获取起始位置
    float start_pos = 0;
    ZAux_Direct_GetDpos(handle, axis, &start_pos);

    std::cout << "--- 安全位移测试 (MOVE) ---" << std::endl;
    std::cout << "目标轴: " << axis << " | 起始位置: " << start_pos << std::endl;
    std::cout << "测试位移: " << target_distance << " mm | 预计耗时: ~2s" << std::endl;
    std::cout << "------------------------------------------" << std::endl;

    // 3. 发送相对位移指令
    ZAux_Direct_Single_Move(handle, axis, target_distance);

    // 4. 动态监控直至停止
    int is_idle = 0; // 0-正在运动，-1-停止运行
    float dpos = 0, mpos = 0;

    // 使用 while 循环监控，避免 for 循环时间不够或太长
    while (true) {
        ZAux_Direct_GetDpos(handle, axis, &dpos);
        ZAux_Direct_GetMpos(handle, axis, &mpos);
        ZAux_Direct_GetIfIdle(handle, axis, &is_idle);

        std::cout << "\r状态: " << (is_idle == 0 ? "运行中" : "已停止")
                  << " | DPOS: " << std::setw(8) << std::fixed << std::setprecision(2) << dpos 
                  << " | MPOS: " << std::setw(8) << mpos 
                  << " | 偏差: " << (dpos - mpos) << std::flush;

        if (is_idle == -1) break; // 轴进入 Idle 状态，退出循环

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // 5. 运动后的安全确认
    std::cout << "\n------------------------------------------" << std::endl;
    std::cout << "测试完成。最终位置: " << dpos << std::endl;

    // 6. 安全关闭流程
    // 测试完成后建议关闭使能，防止电机持续发热（如果是调试阶段）
    ZAux_Direct_SetAxisEnable(handle, axis, 0);
    ZAux_Close(handle);
    handle = NULL; 

    std::cout << "通信已关闭，安全退出。" << std::endl;

    return 0;
}