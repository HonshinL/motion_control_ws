#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include "../include/zmcaux.h"

int main() {
    ZMC_HANDLE handle = NULL;
    // 实体卡 IP 请根据实际修改，仿真器通常用 "127.0.0.1"
    char ip[] = "192.168.0.11"; 
    
    // 1. 建立通信连接
    if (ZAux_OpenEth(ip, &handle) != 0) {
        std::cerr << "连接控制器失败，请检查 IP 或网络连接。" << std::endl;
        return -1;
    }
    std::cout << "连接成功！" << std::endl;

    // 2. 初始化总线
    ZAux_BusCmd_InitBus(handle);
    
    // 增加等待时间或更细致的判断
    int init_status = 0;
    for (int i = 0; i < 50; i++) { // 增加循环次数到 50 次 (约 5秒)
        ZAux_BusCmd_GetInitStatus(handle, &init_status);
        if (init_status == 1) {
            std::cout << "总线初始化成功完成！" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }

    // 如果初始化没成功，最好读取具体的节点状态，看看卡在哪里
    for (int axis = 0; axis < 3; axis++) {
        uint32 node_st = 0;  // 建议使用 int，匹配 SDK 定义
        // 注意：这里的 i 指的是总线上的第几个从站
        ZAux_BusCmd_GetNodeStatus(handle, 0, axis, &node_st);

        // 打印每个节点的具体状态，方便调试
        std::cout << "节点 " << axis << " 状态码: 0x" << std::hex << node_st << std::dec;

        if (node_st == 0x08) {
            std::cout << " [运行正常 (Operational)]" << std::endl;
        } else {
            std::cout << " [异常!]" << std::endl;
            // 常见的状态解读：
            // 0x01: Init (初始化)
            // 0x02: Pre-Op (预操作)
            // 0x04: Safe-Op (安全操作)
            // 如果卡在 0x04，通常是驱动器参数配置有误或同步周期不匹配
        }
    }

    // 3. 获取总线节点数量
    int node_num = 0;
    ZAux_BusCmd_GetNodeNum(handle, 0, &node_num);
    std::cout << "总线上已发现节点数: " << node_num << std::endl;

    // 4. 定义我们要操作的轴 (0, 1, 2 号轴)
    std::vector<int> axes = {0, 1, 2};

    std::cout << "正在清理报警并使能轴..." << std::endl;
    for (int axis : axes) {
        // A. 清除驱动器可能存在的报警
        ZAux_BusCmd_DriveClear(handle, axis, 0);
        
        // B. 设置基本参数（根据你的丝杠/减速比设定 units）
        ZAux_Direct_SetUnits(handle, axis, 1000.0);
        ZAux_Direct_SetSpeed(handle, axis, 100.0);
        ZAux_Direct_SetAccel(handle, axis, 1000.0);

        // C. 开启使能
        int ret = ZAux_Direct_SetAxisEnable(handle, axis, 1);
        if (ret == 0) {
            std::cout << "轴 " << axis << " 使能成功。" << std::endl;
        }
    }

    // 5. 模拟程序运行阶段
    std::cout << "\n系统已进入使能状态，保持 5 秒后自动关闭..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // 6. 安全退出流程 (务必手动关闭使能)
    std::cout << "\n正在执行安全停机流程..." << std::endl;
    for (int axis : axes) {
        // 停止当前轴的所有运动
        ZAux_Direct_Single_Cancel(handle, axis, 2);
        // 关闭使能（电机掉电，手可推）
        ZAux_Direct_SetAxisEnable(handle, axis, 0);
        std::cout << "轴 " << axis << " 已去使能。" << std::endl;
    }

    // 7. 关闭连接
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ZAux_Close(handle);
    handle = NULL; // 手动置空指针，防止误用
    std::cout << "通信关闭，程序退出。" << std::endl;

    return 0;
}