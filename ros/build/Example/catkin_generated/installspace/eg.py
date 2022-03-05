#! /bin/python3
"""
上面一行按格式填写Python解释器路径，
这样，当赋予这个脚本文件可执行权限时，
在控制台敲入此文件名并回车，
控制台就能根据此行找到对应的解释器，
自动调用它执行这个脚本，而不需要我们手工指定。
"""

import rospy #rospy在安装ros时已经安装好

if __name__ == "__main__":
    # 第一步：给定节点名，初始化节点
    rospy.init_node("examplePY")
    rospy.loginfo("Hello world.")
