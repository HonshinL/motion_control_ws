#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include "zmotion_driver/zmcaux.h"

int main() {
    ZMC_HANDLE handle = NULL;
    char ip[] = "192.168.0.11"; 
    if (ZAux_OpenEth(ip, &handle) != 0) return -1;

    // 定义要回零的轴列表
    const int axis_count = 3;
    int axes[axis_count] = {0, 1, 2};

    for (int i = 0; i < axis_count; i++) {
        int ax = axes[i];
        
        // 1. 设置回零参数 (这些通常在 ZDevelop 或驱动器软件中已配好，此处为二次确认)
        ZAux_Direct_SetSpeed(handle, ax, 10.0);      // 寻找原点的速度
        ZAux_Direct_SetAccel(handle, ax, 100.0);
        
        // 2. 轴使能
        ZAux_Direct_SetAxisEnable(handle, ax, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // 3. 启动回零 (DATUM 模式)
        // 模式 13: 对应 EtherCAT 驱动器回零。控制器会通知驱动器开始其内部回零动作。
        std::cout << "轴 " << ax << " 启动回零流程..." << std::endl;
        ZAux_Direct_Datum(handle, ax, 13); 
    }

    // 4. 循环监控回零状态
    bool all_done = false;
    while (!all_done) {
        all_done = true;
        std::cout << "\r状态: ";

        for (int i = 0; i < axis_count; i++) {
            int ax = axes[i];
            int res = 0;
            float mpos = 0;

            // 获取回零状态：0-正在回零，1-回零成功
            ZAux_Direct_GetDatumStatus(handle, ax, &res);
            ZAux_Direct_GetMpos(handle, ax, &mpos);

            std::cout << "轴" << ax << ":" << (res == 1 ? "[完成]" : "[搜索]") 
                      << " POS:" << std::fixed << std::setprecision(2) << mpos << " | ";

            if (res != 1) all_done = false; // 只要有一个没完，就继续等
        }
        std::cout << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::cout << "\n所有轴已回到机械零点，坐标已自动清零。" << std::endl;

    // 5. 释放资源
    ZAux_Close(handle);
    handle = NULL;
    return 0;
}