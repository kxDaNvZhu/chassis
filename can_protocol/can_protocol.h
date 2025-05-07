#ifndef __CAN_PROTOCOL_H__
#define __CAN_PROTOCOL_H__

#include <cstdio>
#include <linux/can.h>

namespace control_can {

typedef struct Control_10
{
    // float Lon_Control_Val;  // m/s
    // float Lat_Control_Val;  // 
    // int Speed_Mode;
    // int Front_Light;
    // int Brake_Enable;
    // int Relay_Release_1;
    // int Relay_Release_2;

    float Lon_Control_Val;        // 整车速度 m/s  
    float Lat_Control_Val;        // 转向曲率   
    
    int Speed_Mode;               // 高低速切换  高：1，低：0
    int Car_Lights;               // 车灯  灭：0，亮：1
    int Brake_Enable;             // 刹车  无效：0，有效：1
    int Strobe_Light;             // 爆闪灯  无效：0，有效：1
    int Parking;                  // 驻车  无效：0，有效：1
    int Emergency_Stop;           // 急停  无效：0，有效：1
    int Request;                  // 请求（使能）  无效：0，有效：1
    int Steering_Mode_Switching;  // 转向模式切换  关：0，开：1

    int Gear_Position;            // 档位  ：驻车 00、前进 01、空挡 10、后退 11
    int Brake_Pressure;           // 制动压力  
    
    Control_10(){
        // Lon_Control_Val = 0;
        // Lat_Control_Val = 0;
        // Speed_Mode = 0;
        // Front_Light = 0;
        // Brake_Enable = 0;
        // Relay_Release_1 = 0;
        // Relay_Release_2 = 0;

        Lon_Control_Val = 0;          // 整车速度 m/s  
        Lat_Control_Val = 0;          // 转向曲率   
        
        Speed_Mode = 0;               // 高低速切换  高：1，低：0
        Car_Lights = 0;               // 车灯  灭：0，亮：1
        Brake_Enable = 0;             // 刹车  无效：0，有效：1
        Strobe_Light = 0;             // 爆闪灯  无效：0，有效：1
        Parking = 0;                  // 驻车  无效：0，有效：1
        Emergency_Stop = 0;           // 急停  无效：0，有效：1
        Request = 0;                  // 请求（使能）  无效：0，有效：1
        Steering_Mode_Switching = 0;  // 转向模式切换  关：0，开：1

        Gear_Position = 0;        // 档位
        Brake_Pressure = 0;       // 制动压力
    }

    int ctrl_ConvertToCanFrame(struct can_frame &frame) {
        frame.can_id = 0x0010;   // CAN帧的标识符
        int lon_velocity = static_cast<int>((10000 * Lon_Control_Val + 68) / 17);
        int lat_velocity = static_cast<int>((1000 * Lat_Control_Val + 188) / 2);
        frame.data[0] = static_cast<int>(Lon_Control_Val * 100) & 0xFF;
		frame.data[1] = 0x00;
		frame.data[2] = static_cast<int>(Lat_Control_Val * 100) & 0xFF;
		frame.data[3] = 0x00;
        // frame.data[4] = 0x00;
		// frame.data[5] = 0x00;
        // frame.data[6] =  Speed_Mode & 0x01;
        // frame.data[6] |=  (Front_Light & 0x01) << 1;
        // frame.data[6] |=  (Brake_Enable & 0x01) << 2;
        // frame.data[6] |=  (Relay_Release_1 & 0x01) << 3;
        // frame.data[6] |=  (Relay_Release_2 & 0x01) << 4;
        // frame.data[7] = 0x00;

        // frame.data[0] = lon_velocity & 0xFF;
		// frame.data[1] = (lon_velocity >> 8) & 0xFF;
		// frame.data[2] = lat_velocity & 0xFF;
		// frame.data[3] = (lat_velocity >> 8) & 0xFF;
        frame.data[4] = 0x00;
		frame.data[5] = 0x00;
        frame.data[6] =  Speed_Mode & 0x01;
        frame.data[6] |=  (Car_Lights & 0x01) << 1;
        frame.data[6] |=  (Brake_Enable & 0x01) << 2;
        frame.data[6] |=  (Strobe_Light & 0x01) << 3;
        frame.data[6] |=  (Parking & 0x01) << 4;
        frame.data[6] |=  (Emergency_Stop & 0x01) << 5;
        frame.data[6] |=  (Request & 0x01) << 6;
        frame.data[6] |=  (Steering_Mode_Switching & 0x01) << 7;
        frame.data[7] = 0x00;

        return 0;
	}
}Control_10;

typedef struct FeedBack_11
{
    // double velocity; 
    // double angular_velocity;
    // // int soc;
    // int forward_tilt_arm_angle;
    // int rearward_tilt_arm_angle;
    // __uint8_t soc;
    // __uint8_t chassis_fault;

    float Lon_Velocity_Fb;        // 整车速度 m/s
    float Front_Flip_Arm_Angle_Fb;   // 前翻转臂角度 rad
    float Rear_Flip_Arm_Angle_Fb;    // 后翻转臂角度 rad
    float Chassis_Battery_SOC_Fb;    // 底盘电池电量 %
    uint32_t Chassis_Fault_Word_Fb;  // 底盘故障字 bitmask

    FeedBack_11()
    {
        // velocity = 0.0;
        // angular_velocity = 0.0;
        // forward_tilt_arm_angle = 0;
        // rearward_tilt_arm_angle = 0;
        // soc = 0;
        // chassis_fault = 0;

        Lon_Velocity_Fb = 0;          // 整车速度 m/s
        Front_Flip_Arm_Angle_Fb = 0;     // 前翻转臂角度 rad
        Rear_Flip_Arm_Angle_Fb = 0;      // 后翻转臂角度 rad
        Chassis_Battery_SOC_Fb = 0;       // 底盘电池电量 %
        Chassis_Fault_Word_Fb = 0;        // 底盘故障字 bitmask
    }

    int ChassisFb_GetFromCanFrame(struct can_frame &frame) {
        if (frame.can_dlc < 8) return -1;

        // velocity = (short)((frame.data[0] & 0xFF) + ((frame.data[1] & 0xFF) << 8)) * 0.01;
        // angular_velocity = (short)((frame.data[2] & 0xFF) + ((frame.data[3] & 0xFF) << 8)) * 0.01;
        // forward_tilt_arm_angle = (short)((frame.data[2] & 0xFF) + ((frame.data[3] & 0xFF) << 8)) ;
        // rearward_tilt_arm_angle = (short)((frame.data[4] & 0xFF) + ((frame.data[5] & 0xFF) << 8)) ;
        // soc = (short)((frame.data[6] & 0xFF));
        // chassis_fault = (short)((frame.data[7] & 0xFF));

        float DEG_TO_RAD = M_PI / 180.0f;

        Lon_Velocity_Fb = (frame.data[0] + (frame.data[1] << 8)) * 0.01f;
        Front_Flip_Arm_Angle_Fb = (frame.data[2] + (frame.data[3] << 8)) * DEG_TO_RAD;
        Rear_Flip_Arm_Angle_Fb = (frame.data[4] + (frame.data[5] << 8)) * DEG_TO_RAD;
        Chassis_Battery_SOC_Fb = static_cast<float>(frame.data[6]); //  + 44.0f;
        Chassis_Fault_Word_Fb = frame.data[7];
        return 0;
	}
} FeedBack_11;

}

#endif
