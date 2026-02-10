#!/usr/bin/env python3
import math

def calculate_distance(x, y):
    """一个极简的复用函数：计算到原点的欧几里得距离"""
    return math.sqrt(x**2 + y**2)

def get_status_label(dist):
    """根据距离返回状态描述"""
    if dist < 1.0:
        return "即将到达"
    return "航行中"