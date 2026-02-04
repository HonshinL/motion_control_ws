#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include "zmotion_driver/zmcaux.h"

int main() {
    ZMC_HANDLE handle = NULL;
    char ip[] = "192.168.0.11"; 
    if (ZAux_OpenEth(ip, &handle) != 0) return -1;

    int axis = 2;
    float distance = 10.0; // 目标移动 10mm

    // 1. 初始化安全参数
    ZAux_Direct_SetUnits(handle, axis, 1000.0); // 1mm = 1000 脉冲
    ZAux_Direct_SetSpeed(handle, axis, 2.0);    // 速度设为很慢的 2mm/s (跑完需5秒)
    ZAux_Direct_SetAccel(handle, axis, 20.0);   // 缓慢加速
    ZAux_Direct_SetDecel(handle, axis, 20.0);   // 缓慢减速
    
    // 设置软限位防止意外
    ZAux_Direct_SetFsLimit(handle, axis, 100.0); // 使用FsLimit代替ForwardLimit
    ZAux_Direct_SetRsLimit(handle, axis, -100.0); // 使用RsLimit代替ReverseLimit

    // 使能轴
    ZAux_Direct_SetAxisEnable(handle, axis, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 2. 执行相对位移运动
    std::cout << ">>> 启动 0 号轴慢速运动: 10mm @ 2mm/s" << std::endl;
    ZAux_Direct_Single_Move(handle, axis, distance);

    // 3. 实时位置读取循环
    float dpos = 0;
    float mpos = 0;
    int is_idle = 0;

    // 表头格式
    std::cout << std::setw(10) << "状态" << std::setw(12) << "指令位置" << std::setw(12) << "反馈位置" << std::endl;

    while (true) {
        // 获取指令位置 (DPOS) 和 编码器反馈位置 (MPOS)
        ZAux_Direct_GetDpos(handle, axis, &dpos);
        ZAux_Direct_GetMpos(handle, axis, &mpos);
        
        // 获取轴状态: -1 表示停止(Idle), 0 表示正在运动
        ZAux_Direct_GetIfIdle(handle, axis, &is_idle);

        // 实时打印刷新
        std::cout << "\r" << std::setw(10) << (is_idle == 0 ? "运行中" : "已完成")
                  << std::setw(10) << std::fixed << std::setprecision(3) << dpos << " mm"
                  << std::setw(10) << mpos << " mm" << std::flush;

        if (is_idle == -1) break; // 运动结束，退出循环

        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20Hz 刷新频率
    }

    // 4. 结束并显示最终偏差
    std::cout << "\n-----------------------------------" << std::endl;
    std::cout << "测试结束。最终静止位置: " << dpos << " mm" << std::endl;
    std::cout << "最终稳态误差: " << (dpos - mpos) * 1000 << " um" << std::endl;

    ZAux_Close(handle);
    handle = NULL;
    return 0;
}