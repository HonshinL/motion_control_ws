#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include "zmotion_driver/zmcaux.h"

int main() {
    ZMC_HANDLE handle = NULL;
    char ip[] = "192.168.0.11"; 
    if (ZAux_OpenEth(ip, &handle) != 0) return -1;

    int axis = 0;

    // 1. 基础运动参数设置
    ZAux_Direct_SetUnits(handle, axis, 1000.0); 
    ZAux_Direct_SetSpeed(handle, axis, 20.0);   
    ZAux_Direct_SetAccel(handle, axis, 200.0);  
    ZAux_Direct_SetDecel(handle, axis, 200.0);
    
    // 2. 【核心安全】设置软限位
    // 注意：软限位必须在回零（Datum）之后才真正有物理意义
    float min_limit = -10.0;
    float max_limit = 100.0;
    ZAux_Direct_SetReverseLimit(handle, axis, min_limit); // 负向限位
    ZAux_Direct_SetForwardLimit(handle, axis, max_limit); // 正向限位

    // 确保使能
    ZAux_Direct_SetAxisEnable(handle, axis, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 3. 启动正向点动 (模拟人按住按钮)
    std::cout << "--- 软限位保护下的 Jog 测试 ---" << std::endl;
    std::cout << "当前设定范围: [" << min_limit << " , " << max_limit << "] mm" << std::endl;
    
    ZAux_Direct_Single_Vmove(handle, axis, 1); 

    // 4. 循环读取 DPOS 和 轴状态
    int axis_status = 0;
    float cur_pos = 0;

    for (int i = 0; i < 60; i++) { // 循环 6 秒，足够跑到 100mm 触发限位
        ZAux_Direct_GetDpos(handle, axis, &cur_pos);
        
        // 读取轴状态：通过位运算判断是否触发限位
        // AXISSTATUS 第 4 位为正向限位，第 5 位为负向限位
        ZAux_Direct_GetAxisStatus(handle, axis, &axis_status);
        
        std::cout << "\r位置: " << std::fixed << std::setprecision(2) << std::setw(6) << cur_pos << " mm"
                  << " | 状态码: " << axis_status;

        if (axis_status & 0x10) { // 0x10 是正向限位触发标志
            std::cout << " [触发正向软限位！]" << std::flush;
            break; 
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 5. 停止并清理
    ZAux_Direct_Single_Cancel(handle, axis, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // 检查最终位置
    ZAux_Direct_GetDpos(handle, axis, &cur_pos);
    std::cout << "\n最终停止位置: " << cur_pos << " mm" << std::endl;

    ZAux_Close(handle);
    handle = NULL;
    return 0;
}