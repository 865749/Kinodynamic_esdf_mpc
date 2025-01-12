#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <array>
#include <cstdint>

#if defined(__GNUC__)
#define PACK(__Declaration__) __Declaration__ __attribute__((__packed__))
#elif defined(_MSC_VER)
#define PACK(__Declaration__) __pragma(pack(push, 1)) __Declaration__ __pragma(pack(pop))
#else
#error not support packed struct
#endif

namespace rm_serial_driver {

    PACK(struct SendPacket {
        uint8_t header = 0xFF;

        // 控制相关
        float line_vel_x; // x轴线速度，前方为正方向，单位：m/s
        float line_vel_y; // y轴线速度，左方为正方向，单位：m/s
        float angle_vel_z;// 顺时针旋转速度，弧度制，单位：rad/s

        // 决策相关
        bool is_use_top; // 是否使用小陀螺
        uint8_t priority;// 优先级   0x00-导航优先  0x01-视觉优先

        // 裁判系统相关
        // uint8_t intention;        // 0x00-表示不管该路径数据  0x01-到目标点攻击  0x02-到目标点防守  0x03-移动到目标点
        // uint16_t start_position_x;// 路径起点x轴坐标
        // uint16_t start_position_y;// 路径起点y轴坐标
        // uint8_t delta_x[49];      // 路径点x轴增量数组，单位：dm
        // uint8_t delta_y[49];      // 路径点y轴增量数组，单位：dm

        uint8_t uart_end = 0xFE;
    });

}// namespace rm_serial_driver

#endif// RM_SERIAL_DRIVER__PACKET_HPP_
