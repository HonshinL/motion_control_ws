#include <iostream>
#include <chrono>
#include <thread>
#include "zmotion_driver/zmcaux.h"

int main() {
    ZMC_HANDLE handle = NULL;
    char ip[] = "192.168.0.11";
    if (ZAux_OpenEth(ip, &handle) != 0) return -1;

    // 1. 检查总线状态 (对于 EtherCAT 轴至关重要)
    // 注意：ZAux_Direct_GetBusState 函数在当前库版本中可能未定义
    // 如果需要检查总线状态，请使用相应的 ZMotion 库函数
    std::cout << "提示: 检查总线状态的函数在当前库版本中未定义，跳过总线检查..." << std::endl;

    // 2. 循环使能 0, 1, 2 号总线轴
    int target_axes[] = {0, 1, 2};
    for (int axis_no : target_axes) {
        // 设置单位（脉冲当量），根据实际机械结构设置
        
        ZAux_Direct_SetUnits(handle, axis_no, 1000.0); 
        // 设置基本速度与加速度
        ZAux_Direct_SetSpeed(handle, axis_no, 100.0);
        ZAux_Direct_SetAccel(handle, axis_no, 1000.0);

        // 执行使能
        int ret = ZAux_Direct_SetAxisEnable(handle, axis_no, 1);
        
        if (ret == 0) {
            std::cout << "轴 " << axis_no << " 使能成功！" << std::endl;
        } else {
            std::cout << "轴 " << axis_no << " 使能失败，错误码: " << ret << std::endl;
        }
    }

    // 3. 简单的安全检测
    // 读取 0 号轴的使能状态位 (IDLE 状态通常表示未运行且准备就绪)
    int status = 0;
    ZAux_Direct_GetAxisStatus(handle, 0, &status);
    std::cout << "0 号轴当前状态字: " << status << std::endl;

    // 4. 安全关闭流程
    std::cout << "\n正在执行安全停机..." << std::endl;
    for (int axis_no : target_axes) {
        // 步骤 A: 停止轴运动 (Cancel)
        // 参数 2 表示减速停止，防止惯性过大损伤机械结构
        ZAux_Direct_Single_Cancel(handle, axis_no, 2); 
        
        // 步骤 B: 关闭使能 (Disable)
        int ret = ZAux_Direct_SetAxisEnable(handle, axis_no, 0);
        
        if (ret == 0) {
            std::cout << "轴 " << axis_no << " 已成功去使能（掉电）。" << std::endl;
        }
    }

    // 给硬件一点响应时间（50ms-100ms）确保指令到达执行器
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 5. 最后才关闭通信句柄
    ZAux_Close(handle);
    handle = NULL; 
    std::cout << "通信已关闭，程序安全退出。" << std::endl;

    return 0;
}