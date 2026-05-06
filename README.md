本代码在此环境下运行：  
Ubuntu22.04 Ros2 Humble

使用时需安装环境::
1. ACADOtoolkit  
2. nlopt
3. ~~3. cppAD(已经废除)~~
4. glog
colcon build --symlink-install

pip install xmacro
https://github.com/osqp/osqp
git clone https://github.com/robotology/osqp-eigen.git
###ACADO使用注意事项  
需要创建模型对应的cpp->生成mpc代码->引入文件->调用代码;  
一键安装依赖rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y 

所使用的仿真开源为：
https://github.com/LihanChen2004/rmul24_gazebo_simulator

