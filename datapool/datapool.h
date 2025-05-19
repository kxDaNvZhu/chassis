#ifndef Chassis_DATA_POOL_H_
#define Chassis_DATA_POOL_H_

#include <condition_variable>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <vector>
#include "can_protocol.h"

namespace control_can {
enum MODE_CONTROL_CHASSIS
{
    REMOTE = 0,
    AUTONOMY = 1,
};

enum STATUS_CHASSIS
{
	STATUS_EMERGENCY_STOP = 1,
	STATUS_PARK = 2,
	STATUS_WORK = 3
};

enum IS_EMERGENCY_STOP
{
	NOT_EMERGENCY_STOP = 0,
	YES_EMERGENCY_STOP = 1,
};

enum IS_PARK
{
	NOT_PARK = 0,
	YES_PARK = 1,
};

typedef struct ChassisConfig
{
	float total_mileage = 0;
	float current_mileage = 0;
	float last_service_mileage = 0;
	float time_interval_mileage = 1.0f; // 计算间隔，单位秒
};

typedef struct Localization
{
	double xg;
	double yg;
	double yaw;
}Localization;

typedef struct ctrlCanTrigger{
	int seqBrake = 0;
	int seqDrive = 0;
	int seqSteer = 0;
	int seqVehicle = 0;
};

// typedef struct TimeStruct {
//     double startTimeSeconds = 0;  // 起始时间 s
//     double currentTimeSeconds = 0; // 当前时间 s
// };

typedef struct fromCtrl{
	// float Lon_Control_Val_Ctrl = 0;  // m/s
    // float Lat_Control_Val_Ctrl = 0;  // rad/s
    // int Speed_Mode_Ctrl = 0;
    // int Front_Light_Ctrl = 0;
    // int Brake_Enable_Ctrl = 0;
    // int Relay_Release_1_Ctrl = 0;
    // int Relay_Release_2_Ctrl = 0;

	float Lon_Control_Val_Ctrl;        // 整车速度 m/s  
    float Lat_Control_Val_Ctrl;        // 转向曲率   
    
    int Speed_Mode_Ctrl;               // 高低速切换  高：1，低：0
    int Car_Lights_Ctrl;               // 车灯  灭：0，亮：1
    int Brake_Enable_Ctrl;             // 刹车  无效：0，有效：1
    int Strobe_Light_Ctrl;             // 爆闪灯  无效：0，有效：1
    int Parking_Ctrl;                  // 驻车  无效：0，有效：1
    int Emergency_Stop_Ctrl;           // 急停  无效：0，有效：1
    int Request_Ctrl;                  // 请求（使能）  无效：0，有效：1
    int Steering_Mode_Switching_Ctrl;  // 转向模式切换  关：0，开：1

    int Gear_Position_Ctrl;            // 档位  ：驻车 00、前进 01、空挡 10、后退 11
    int Brake_Pressure_Ctrl;           // 制动压力

	bool is_steering_in_place_Ctrl = false;
	int control_mode_Ctrl = 0;
	bool emergency_stop_Ctrl = false;
};

typedef struct fromRemote{
	int64_t remote_time = 0;

	// float Lon_Control_Val = 0;  // m/s
    // float Lat_Control_Val = 0;  // rad/s
    // int Speed_Mode = 0;
    // int Front_Light = 0;
    // int Brake_Enable = 0;
    // int Relay_Release_1 = 0;
    // int Relay_Release_2 = 0;

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

	int control_mode = 0;
	bool parking = false;
	bool emergency_stop = false;
};

typedef struct causeVehicleHalt{
	int remote_control_mode = 0;
	// 遥控器指令
	int remote_emergency_stop = 0;
	int remote_parking = 0;
	int remote_brake_enable = 0;
	int remote_brake_pressure = 0;

	// 自主控制指令
	int ctrl_emergency_stop = 0;
	int ctrl_parking = 0;
	int ctrl_brake_enable = 0;
	int ctrl_brake_pressure = 0;

	// 车辆底盘故障
	int chassis_fault_fb = 0;
};


typedef struct ControlCanMainData{
	ChassisConfig chassis_config;
	ctrlCanTrigger ctrlCanTrig;
	// MODE_CONTROL_CHASSIS MODE_CTRL_FROM_REMOTE = REMOTE;
	// MODE_CONTROL_CHASSIS MODE_CTRL_FB = REMOTE;
	// STATUS_CHASSIS STATUS_CHASSIS_CMD;
	// STATUS_CHASSIS STATUS_CHASSIS_FB;
	fromCtrl from_ctrl;
	fromRemote from_remote;
	causeVehicleHalt cause_of_vehicle_halt;
	bool isWork = true;

	// TimeStruct time_struct_; 
	// std::mutex time_mutex_;

	bool is_steering_in_place_Ctrl = false; // 新增字段，表示是否在原地转向
	float total_mileage_ = 0.0f;
	float current_mileage_ =  0.0f;

	ControlCanMainData()
	{
	}
}ControlCanMainData;


class DataPool {

public:
    ControlCanMainData* GetMainDataPtr() { return &main_data_; }
    ControlCanMainData& GetMainDataRef() { return main_data_; }

	void SetChassis_StaFb(const FeedBack_11& StaFb) {
		std::unique_lock<std::mutex> locker(StaFb_mutex_);
		StaFb_ = StaFb;
	}

	void GetChassis_StaFb(FeedBack_11& StaFb) {
		std::unique_lock<std::mutex> locker(StaFb_mutex_);
		StaFb = StaFb_;
	}

	void SetCtrl(const Control_10& Control_Data) {
		std::unique_lock<std::mutex> locker(Control_mutex_);
		Control_Data_ = Control_Data;
	}
	void GetCtrl(Control_10& Control_Data) {
		std::unique_lock<std::mutex> locker(Control_mutex_);
		Control_Data = Control_Data_;
	}

	

private:
    ControlCanMainData main_data_;

	

	FeedBack_11 StaFb_;
	std::mutex StaFb_mutex_;
	Control_10 Control_Data_;
	std::mutex Control_mutex_;
    


    DataPool() {}
public:
    static DataPool *Instance() {
        static DataPool* ptr = nullptr;
        if (ptr == nullptr) {
            ptr = new DataPool();
        }
        return ptr;
    }
};
}  // namespace control
#endif  // Chassis_DATA_POOL_H_
