#include <iostream>
#include <vector>
#include <iomanip> // 用于格式化输出
#include "zmotion_driver/zmcaux.h"

int main() {
    ZMC_HANDLE handle = NULL;
    // 仿真器使用 127.0.0.1，实体卡请改为实际 IP (如 192.168.0.11)
    char ip_addr[] = "192.168.0.11"; 

    // 1. 连接控制器
    int ret = ZAux_OpenEth(ip_addr, &handle);
    if (ret != 0) {
        std::cerr << "无法连接控制器，错误码: " << ret << std::endl;
        return -1;
    }

    // 2. 获取系统最大规格
    uint16 max_virt_axes = 0;
    uint8 max_motors = 0;
    uint8 io_info[4];
    ZAux_GetSysSpecification(handle, &max_virt_axes, &max_motors, io_info);

    std::cout << "\n==============================================" << std::endl;
    std::cout << "控制器规格: 最大轴数 " << max_virt_axes << ", 最大物理电机 " << (int)max_motors << std::endl;
    std::cout << "==============================================" << std::endl;
    std::cout << std::left << std::setw(10) << "轴号" 
              << std::setw(12) << "物理地址" 
              << std::setw(15) << "类型(ATYPE)" 
              << "说明" << std::endl;
    std::cout << "----------------------------------------------" << std::endl;

    // 3. 遍历所有可能的轴号进行深度检测
    int active_count = 0;
    for (int i = 0; i < (int)max_virt_axes; i++) {
        int addr = -1;
        int atype = 0;

        // 获取轴地址 (Address)
        ZAux_Direct_GetAxisAddress(handle, i, &addr);
        // 获取轴类型 (Atype)
        ZAux_Direct_GetAtype(handle, i, &atype);

        // 如果地址不是 -1，或者 ATYPE 不是 0，说明这个轴被定义或使用了
        if (addr != -1 || atype != 0) {
            active_count++;
            
            std::cout << std::left << std::setw(10) << i 
                      << std::setw(12) << addr 
                      << std::setw(15) << atype;

            // 识别轴类型含义
            if (addr == -1) {
                std::cout << "虚拟轴 (Virtual Axis)";
            } else if (atype == 1 || atype == 7) {
                std::cout << "脉冲轴 (Pulse/Dir)";
            } else if (atype == 65 || atype == 67) {
                std::cout << "总线轴 (EtherCAT/CANopen)";
            } else {
                std::cout << "其他类型轴";
            }
            std::cout << std::endl;
        }
    }

    if (active_count == 0) {
        std::cout << "未检测到任何已配置的有效轴。" << std::endl;
    }

    std::cout << "----------------------------------------------" << std::endl;
    std::cout << "扫描完成，共发现 " << active_count << " 个有效配置轴。\n" << std::endl;

    // 4. 关闭连接
    ZAux_Close(handle);
    handle = NULL;
    return 0;
}