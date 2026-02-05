#include <iostream>
#include <cstdio>
#include <iomanip>  // 用于控制输出格式
#include "zmotion_driver/zmcaux.h" // 使用项目中的Zmotion头文件

int main(int argc, char* argv[])
{
    // 控制器IP：仿真器127.0.0.1，实体控制器替换为实际IP（如192.168.0.11，127.0.0.1）
    char ip_addr[] = "192.168.0.11";
    ZMC_HANDLE handle = NULL; // 连接句柄
    int ret = 0;

    // 1. 连接控制器/仿真器
    ret = ZAux_OpenEth(ip_addr, &handle);
    if (ERR_OK != ret)
    {
        std::cout << "连接失败！错误码：" << ret << std::endl;
        return -1;
    }
    std::cout << "连接控制器/仿真器成功！" << std::endl;

    // 定义要查询的轴号列表
    int axes[] = {0, 1, 2, 4, 5};
    int num_axes = sizeof(axes) / sizeof(axes[0]);
    
    // 定义变量存储查询结果
    float soft_limit_up = 0.0;  // 软限位上限（正向）
    float soft_limit_down = 0.0;// 软限位下限（反向）
    int hard_fwd_status = 0;     // 硬件正向限位状态
    int hard_rev_status = 0;     // 硬件反向限位状态

    // 2. 查询所有轴的限位信息
    for (int i = 0; i < num_axes; ++i) {
        int axis = axes[i];
        
        std::cout << "\n=== 轴 " << axis << " 限位信息 ===" << std::endl;
        
        // 查询软件限位（移动范围）
        ret = ZAux_Direct_GetFsLimit(handle, axis, &soft_limit_up);
        if (ret == ERR_OK)
        {
            std::cout << "软件正向限位（上限）：" << std::fixed << std::setprecision(3) << soft_limit_up << std::endl;
        }
        else
        {
            std::cout << "查询软上限失败！错误码：" << ret << std::endl;
        }

        ret = ZAux_Direct_GetRsLimit(handle, axis, &soft_limit_down);
        if (ret == ERR_OK)
        {
            std::cout << "软件反向限位（下限）：" << std::fixed << std::setprecision(3) << soft_limit_down << std::endl;
        }
        else
        {
            std::cout << "查询软下限失败！错误码：" << ret << std::endl;
        }

        // 查询硬件限位状态
        ret = ZAux_Direct_GetFwdIn(handle, axis, &hard_fwd_status);
        if (ret == ERR_OK)
        {
            std::cout << "硬件正向限位状态：" << hard_fwd_status << "（1=触发，0=正常）" << std::endl;
        }
        else
        {
            std::cout << "查询硬件正向限位失败！错误码：" << ret << std::endl;
        }

        ret = ZAux_Direct_GetRevIn(handle, axis, &hard_rev_status);
        if (ret == ERR_OK)
        {
            std::cout << "硬件反向限位状态：" << hard_rev_status << "（1=触发，0=正常）" << std::endl;
        }
        else
        {
            std::cout << "查询硬件反向限位失败！错误码：" << ret << std::endl;
        }
    }

    // 4. 断开连接
    ZAux_Close(handle);
    handle = NULL;
    std::cout << "已断开控制器连接！" << std::endl;
    return 0;
}