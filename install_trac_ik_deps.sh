#!/bin/bash
# 安装TRAC-IK依赖脚本

echo "正在安装TRAC-IK依赖..."

# 安装libnlopt-cxx-dev
sudo apt-get update
sudo apt-get install -y libnlopt-cxx-dev

echo "依赖安装完成！"
echo "现在可以运行: cd /home/huzy/grasp_perception && colcon build --packages-select trac_ik_lib trac_ik_kinematics_plugin"
