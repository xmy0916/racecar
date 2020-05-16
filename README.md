# 智能车仿真
代码使用参考教程：https://www.guyuehome.com/category/column/%e6%99%ba%e8%83%bd%e8%bd%a6%e7%ab%9e%e8%b5%9b
# bug汇总

## 报错controllers相关
sudo apt-get install ros-kinetic-joint-state-controllers
sudo apt-get install ros-kinetic-joint-effort-controllers

## 报错driver_base相关
sudo apt-get install ros-kinetic-joint-driver-base

## 报错ackermann_msgs相关
sudo apt-get install ros-kinetic-ackermann-msgs

## 报错findline.cpp找不到opencv头文件
执行：locate OpenCVConfig.cmake得到你的opencv的路径

执行gedit ~/racecar_ws/src/racecar_gazebo/CMakeLists.txt

修改第7行的路径成你的路径:set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev/)

## 没有laser相关话题，无法建图
==这个纯属个人失误，写四轮摄像头组的博客的时候觉得激光雷达的蓝色太碍事了就把它关了......我单纯的以为只是关闭了激光显示，原来是把数据都关了！抱歉～==
～/racecar_ws/src/racecar_description/urdf/racecar.gazebo在这个文件中61行改成false即可，代码我已修复，直接下载没有问题的

