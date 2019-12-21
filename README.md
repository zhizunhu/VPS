Visual Positioning System
======
1.Prerequisites
--
    mkdir -p VPS_ws/src
    cd VPS_ws/src
    git clone https://github.com/zhizunhu/VPS.git
    cd ..
2.编译 maplab
--
    cd VPS_ws
    catkin init
    catkin config --merge-devel
    catkin config --extend /opt/ros/kinetic
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
    catkin build maplab
需保证网络良好，会花费较长时间编译maplab<br>

3.编译 建图插件
--
    catkin build --no-deps createmap
4.离线读ROS包建图
--
    source devel/setup.bash
    roslaunch createmap test.launch
 可以在launch文件中指定rosbag路径和话题名
 
 5.在线读ROS话题建图
 --
    roslaunch createmap map.launch
