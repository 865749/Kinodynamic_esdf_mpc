# ROS2 Publish IMU

本项目基于 [chenjunnn](https://github.com/chenjunnn) 的 [rm_serial_driver](https://gitlab.com/rm_vision/rm_serial_driver) 修改，与其他各模块解耦。

在 minipc （上位机）中使用ROS2，实现了与 Robomaster C板（下位机）进行串口通信，获取 IMU 数据发布为 sensor_msgs/Imu 消息类型，同时可由上位机发布自定义数据给下位机


## 使用指南

1. 安装依赖 
    ```
    sudo apt install ros-humble-serial-driver
    ```

2. 编译
    ```
    colcon build --symlink-install
    ```

2. 启动串口:  

    更改 [serial_driver.yaml](config/serial_driver.yaml) 中的参数以匹配与C板通讯的串口
    
    ```
    sudo chmod 777 /dev/ttyACM0
    source install/setup.bash
    ros2 launch rm_serial_driver serial_driver.launch.py
    ```



## 发送和接收

1. Robomaster C板发送
    ```
    //SendData
    typedef struct{
    uint8_t header;

    float quat01;//IMU姿态四元数
    float quat02;
    float quat03;
    float quat04;
    float gyro01;//角加速度
    float gyro02;
    float gyro03;
    float accel01;//线加速度
    float accel02;
    float accel03;

    uint16_t checksum;
    } __attribute__((packed)) SendData_s;
    ```

2. Robomaster C板接收
    ```
    //ReceivedData
    typedef struct{
    uint8_t header;

    float vx; 
    float vy; 
    float vz; 

    uint16_t checksum;
    }__attribute__((packed)) ReceivedData_s;
    ```
Ubuntu上位机接收和发送包结构与C板类似

关于 C 板串口部分的代码，可参考：

- [usb_task.c](https://gitee.com/LihanChen2004/C_Board_SendIMU/blob/master/application/usb_task.c)

- [usb_task.h](https://gitee.com/LihanChen2004/C_Board_SendIMU/blob/master/application/usb_task.h)

## Tips !!!
[rm_serial_driver.cpp](src/rm_serial_driver.cpp) 第47行：

这里设置函数+订阅绑定，而不是在 SendData() 中增加一个外层循环，是为了方便后续拓展，因为上位机发送信息给下位机肯定是通过获取话题数据

每当新的 `sensor_msgs::msg::Imu` 消息发布到 `/imu` 话题时，`sendData()` 函数会被调用，将消息作为参数传递给它，从而触发数据发送操作。

使用时请注意修改这里~ 将 `imu_sub_` 更改为你需要订阅的话题 


```C++
// Create Subscription
imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));
```
  
