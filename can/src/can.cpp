#include "include/can.h"

#include <iostream>
#include <future>
#include <thread>
#include <memory>
#include <chrono>
#include <ctime>

namespace control_can {

Can::Can() {
    DP = DataPool::Instance();
    ConfigM = ConfigManager::Instance();
    ConfigM->setPath(configPath);
    // DP->GetMainDataPtr()->time_struct_.startTimeSeconds  
    // = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}
Can::~Can() {}

int Can::Init() { 
    int communication_a = InitVars();
    communication_->Init();
    transmitter_->InitDevice();

    const auto calculateMileage_loop = [this] { this->calculateMileage(); };
    calculateMileage_thread_ = std::thread(calculateMileage_loop);

    const auto com_loop = [this] { this->ReadChannel(); };
    rev_thread_ = std::thread(com_loop);

    const auto send_loop = [this] { this->SendChannel(); };
    send_thread_ = std::thread(send_loop);

    // const auto fault_loop = [this] { this->FaultDecisionBasedSystem(); };
    // fault_loop_ = std::thread(fault_loop);
    // usleep(1000 * 50);
    return 0;
}

int Can::InitVars() {
    t_interval = DP->GetMainDataPtr()->chassis_config.time_interval_mileage;
    communication_ = std::make_shared<Rosbridge>();
    transmitter_ = std::make_shared<SocketcanTransmitter>();
    readMileageConfigParameter();
    // RETURN_VAL_IF_NULL(transmitter_, -1);

    return 0;
}


int Can::Start() {
    ros::Rate rate(20);
    while (ros::ok()) {
        communication_->SpinOnce();
        takeModeActual();
        causeHaltFb();
        communication_->Publish();
        rate.sleep();
    }
    return 0;
}

bool Can::readMileageConfigParameter()
{
    std::ifstream infile(configPath);
    // 检查文件是否成功打开
    if (!infile.is_open()) {
        std::cerr << "Error: Failed to open file " << configPath << " for reading!" << std::endl;
        return false;
    }

    DP->GetMainDataPtr()->chassis_config.total_mileage = ConfigM->config()["Mileage"]["total_mileage"].get_value<float>();
    DP->GetMainDataPtr()->chassis_config.current_mileage = ConfigM->config()["Mileage"]["current_mileage"].get_value<float>();
    DP->GetMainDataPtr()->chassis_config.last_service_mileage = ConfigM->config()["Mileage"]["last_service_mileage"].get_value<float>();
    DP->GetMainDataPtr()->chassis_config.time_interval_mileage = ConfigM->config()["Mileage"]["time_interval_mileage"].get_value<float>();

    ConfigM->saveConfig(ConfigM->config());

    // 关闭文件（析构函数会自动关闭，但显式关闭是好习惯）
    infile.close();
    return true;
}

bool Can::writeMileageConfigParameter()
{
    std::ofstream outfile(configPath);
    if (!outfile.is_open()) {
        std::cerr << "Error opening chassis parameter file for writing!" << std::endl;
        return false;
    }

    // 更新YAML文件中的参数
    // ConfigM->config()["Mileage"]["total_mileage"] = DP->GetMainDataPtr()->total_mileage_;
    ConfigM->config()["Mileage"]["total_mileage"] = 8;

    // 保存配置到文件
    ConfigM->saveConfig(ConfigM->config());
    outfile.close();

    return true;
}

void Can::takeModeActual(){
    control_can::Control_10  vehicle_ctrl;

    auto now = std::chrono::system_clock::now();
    // 转换为从 epoch（1970-01-01 00:00:00 UTC）开始的时间间隔
    auto duration = now.time_since_epoch();
    // 转换为毫秒，并用 float 表示
    int64_t milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    int64_t error_time_remote = abs(milliseconds - DP->GetMainDataPtr()->from_remote.remote_time);

    if (DP->GetMainDataPtr()->from_remote.control_mode == 0)
    {
        if ((50.0 < error_time_remote))
        {
            vehicle_ctrl.Lon_Control_Val = 0;
            vehicle_ctrl.Lat_Control_Val = 0;
            vehicle_ctrl.Brake_Enable = true;
            std::cout << "error_time_remote: " << error_time_remote << "\n";
            LOGFM_INFO_C("error_time_remote: %f \n", error_time_remote);
        }
        else
        {
            remoteSpeedCtr();
        }

        // vehicle_ctrl.Lon_Control_Val = DP->GetMainDataPtr()->from_remote.Lon_Control_Val;
        // vehicle_ctrl.Lat_Control_Val = - DP->GetMainDataPtr()->from_remote.Lat_Control_Val;
        // vehicle_ctrl.Speed_Mode = DP->GetMainDataPtr()->from_remote.Speed_Mode;
        // vehicle_ctrl.Front_Light = DP->GetMainDataPtr()->from_remote.Front_Light;
        // vehicle_ctrl.Brake_Enable = DP->GetMainDataPtr()->from_remote.Brake_Enable;
        // vehicle_ctrl.Relay_Release_1 = DP->GetMainDataPtr()->from_remote.Relay_Release_1;
        // vehicle_ctrl.Relay_Release_2 = DP->GetMainDataPtr()->from_remote.Relay_Release_2;

        vehicle_ctrl.Lon_Control_Val = DP->GetMainDataPtr()->from_remote.Lon_Control_Val;
        vehicle_ctrl.Lat_Control_Val = - DP->GetMainDataPtr()->from_remote.Lat_Control_Val;
        
        vehicle_ctrl.Speed_Mode = DP->GetMainDataPtr()->from_remote.Speed_Mode;
        vehicle_ctrl.Car_Lights = DP->GetMainDataPtr()->from_remote.Car_Lights;
        vehicle_ctrl.Brake_Enable = DP->GetMainDataPtr()->from_remote.Brake_Enable;
        vehicle_ctrl.Strobe_Light = DP->GetMainDataPtr()->from_remote.Strobe_Light;
        vehicle_ctrl.Parking = DP->GetMainDataPtr()->from_remote.Parking;
        vehicle_ctrl.Emergency_Stop = DP->GetMainDataPtr()->from_remote.Emergency_Stop;
        vehicle_ctrl.Request = DP->GetMainDataPtr()->from_remote.Request;
        vehicle_ctrl.Steering_Mode_Switching = DP->GetMainDataPtr()->from_remote.Steering_Mode_Switching;
        
        vehicle_ctrl.Gear_Position = DP->GetMainDataPtr()->from_remote.Gear_Position;
        vehicle_ctrl.Brake_Pressure = DP->GetMainDataPtr()->from_remote.Brake_Pressure;
    }

    if (DP->GetMainDataPtr()->from_remote.control_mode == 1)
    {
        // vehicle_ctrl.Lon_Control_Val = DP->GetMainDataPtr()->from_ctrl.Lon_Control_Val_Ctrl;
        // vehicle_ctrl.Lat_Control_Val = DP->GetMainDataPtr()->from_ctrl.Lat_Control_Val_Ctrl;
        // vehicle_ctrl.Speed_Mode = DP->GetMainDataPtr()->from_ctrl.Speed_Mode_Ctrl;
        // vehicle_ctrl.Front_Light = DP->GetMainDataPtr()->from_ctrl.Front_Light_Ctrl;
        // vehicle_ctrl.Brake_Enable = DP->GetMainDataPtr()->from_ctrl.Brake_Enable_Ctrl;
        // vehicle_ctrl.Relay_Release_1 = DP->GetMainDataPtr()->from_ctrl.Relay_Release_1_Ctrl;
        // vehicle_ctrl.Relay_Release_2 = DP->GetMainDataPtr()->from_ctrl.Relay_Release_2_Ctrl;

        vehicle_ctrl.Lon_Control_Val = DP->GetMainDataPtr()->from_ctrl.Lon_Control_Val_Ctrl;
        vehicle_ctrl.Lat_Control_Val = DP->GetMainDataPtr()->from_ctrl.Lat_Control_Val_Ctrl;
        
        vehicle_ctrl.Speed_Mode = DP->GetMainDataPtr()->from_ctrl.Speed_Mode_Ctrl;
        vehicle_ctrl.Car_Lights = DP->GetMainDataPtr()->from_ctrl.Car_Lights_Ctrl;
        vehicle_ctrl.Brake_Enable = DP->GetMainDataPtr()->from_ctrl.Brake_Enable_Ctrl;
        vehicle_ctrl.Strobe_Light = DP->GetMainDataPtr()->from_ctrl.Strobe_Light_Ctrl;
        vehicle_ctrl.Parking = DP->GetMainDataPtr()->from_ctrl.Parking_Ctrl;
        vehicle_ctrl.Emergency_Stop = DP->GetMainDataPtr()->from_ctrl.Emergency_Stop_Ctrl;
        vehicle_ctrl.Request = DP->GetMainDataPtr()->from_ctrl.Request_Ctrl;
        vehicle_ctrl.Steering_Mode_Switching = DP->GetMainDataPtr()->from_ctrl.Steering_Mode_Switching_Ctrl;
        
        vehicle_ctrl.Gear_Position = DP->GetMainDataPtr()->from_ctrl.Gear_Position_Ctrl;
        vehicle_ctrl.Brake_Pressure = DP->GetMainDataPtr()->from_ctrl.Brake_Pressure_Ctrl;
    }

    // if ((true == DP->GetMainDataPtr()->from_remote.parking) 
    // || (true == DP->GetMainDataPtr()->from_remote.emergency_stop))
    // {
    //     vehicle_ctrl.Lon_Control_Val = 0;
    //     vehicle_ctrl.Lat_Control_Val = 0;
    //     vehicle_ctrl.Brake_Enable = true;
    // }

    if (1 == DP->GetMainDataPtr()->from_remote.Parking)
    {
        vehicle_ctrl.Lon_Control_Val = 0;
        vehicle_ctrl.Lat_Control_Val = 0;
        vehicle_ctrl.Brake_Enable = 1;
        vehicle_ctrl.Parking = 1;
    }

    if (1 == DP->GetMainDataPtr()->from_remote.Emergency_Stop)
    {
        vehicle_ctrl.Lon_Control_Val = 0;
        vehicle_ctrl.Lat_Control_Val = 0;
        vehicle_ctrl.Brake_Enable = 1;
        vehicle_ctrl.Emergency_Stop = 1;
    }

    LOGFM_INFO_C("chassis actual command longitudinal velocity: %f, angular velocity: %f, from remote control mode: %d \n",
    vehicle_ctrl.Lon_Control_Val, vehicle_ctrl.Lat_Control_Val, DP->GetMainDataPtr()->from_remote.control_mode);

    DP->SetCtrl(vehicle_ctrl);
}

void Can::remoteSpeedCtr(){
    double targetRemoteSpeed = 0;   // 目标值
    const double maxStepRemote = 0.2; // 最大变化增量
    const double thresholdRemote = 0.5; // 变化量阈值

    double newRemoteSpeedTarget = DP->GetMainDataPtr()->from_ctrl.Lon_Control_Val_Ctrl;

    double delta = fabs(newRemoteSpeedTarget - currentRemoteSpeed);

    if (delta > thresholdRemote)
    {
        // 如果变化量大于阈值，则逐步接近目标值
        if (newRemoteSpeedTarget > currentRemoteSpeed)
        {
            targetRemoteSpeed = currentRemoteSpeed + maxStepRemote;
        }
        else
        {
            targetRemoteSpeed = currentRemoteSpeed - maxStepRemote;
        }
    }
    else
    {
        // 否则直接设置为目标值
        targetRemoteSpeed = newRemoteSpeedTarget;
    }

    currentRemoteSpeed = targetRemoteSpeed;
    DP->GetMainDataPtr()->from_ctrl.Lon_Control_Val_Ctrl = targetRemoteSpeed;
}

void Can::causeHaltFb(){
    DP->GetMainDataPtr()->cause_of_vehicle_halt.remote_control_mode = DP->GetMainDataPtr()->from_remote.control_mode;

    DP->GetMainDataPtr()->cause_of_vehicle_halt.remote_emergency_stop = DP->GetMainDataPtr()->from_remote.Emergency_Stop;
    DP->GetMainDataPtr()->cause_of_vehicle_halt.remote_parking = DP->GetMainDataPtr()->from_remote.Parking;
    DP->GetMainDataPtr()->cause_of_vehicle_halt.remote_brake_enable = DP->GetMainDataPtr()->from_remote.Brake_Enable;
    DP->GetMainDataPtr()->cause_of_vehicle_halt.remote_brake_pressure = DP->GetMainDataPtr()->from_remote.Brake_Pressure;

    DP->GetMainDataPtr()->cause_of_vehicle_halt.ctrl_emergency_stop = DP->GetMainDataPtr()->from_ctrl.Emergency_Stop_Ctrl;
    DP->GetMainDataPtr()->cause_of_vehicle_halt.ctrl_parking = DP->GetMainDataPtr()->from_ctrl.Parking_Ctrl;
    DP->GetMainDataPtr()->cause_of_vehicle_halt.ctrl_brake_enable = DP->GetMainDataPtr()->from_ctrl.Brake_Enable_Ctrl;
    DP->GetMainDataPtr()->cause_of_vehicle_halt.ctrl_brake_pressure = DP->GetMainDataPtr()->from_ctrl.Brake_Pressure_Ctrl;
}

// 获取当前累计里程（单位：米）
float Can::getCurrentMileage()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_mileage_;
}

// 计算里程（应该在独立线程中定期调用）
void Can::calculateMileage()
{
    while (ros::ok())
    {
        auto now = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration<float>(now - last_calc_time_).count();

        control_can::FeedBack_11 feedback_11;
        DP->GetChassis_StaFb(feedback_11);
        float speed = abs(feedback_11.Lon_Velocity_Fb);
        updateSpeed(speed);

        if (elapsed >= t_interval)
        {
            std::lock_guard<std::mutex> lock(data_mutex_);

            // 获取最近1秒的速度数据
            auto start_time = now - std::chrono::milliseconds(1000);
            float sum_speed = 0.0f;
            int count = 0;

            for (const auto &point : speed_history_)
            {
                if (point.timestamp >= start_time)
                {
                    sum_speed += point.speed;
                    count++;
                }
            }

            if (count > 0)
            {
                float avg_speed = sum_speed / count;
                float distance = avg_speed * t_interval; // 里程 = 平均速度 × 时间间隔
                current_mileage_ += distance;

                DP->GetMainDataPtr()->total_mileage_ = (DP->GetMainDataPtr()->chassis_config.total_mileage
                                                        + current_mileage_) / 1000;
                writeMileageConfigParameter();
            }

            last_calc_time_ = now;
            DP->GetMainDataPtr()->current_mileage_ = current_mileage_  / 1000;
        }
        usleep(20 * 1000);
    }
}

// 更新当前速度（单位：m/s）
void Can::updateSpeed(float current_speed)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    SpeedPoint point;
    point.timestamp = std::chrono::system_clock::now();
    point.speed = current_speed;
    speed_history_.push_back(point);

    int num_times = static_cast<int>(t_interval * 1000 + 100);

    // 清理过期数据（保留最近1.1秒的数据）
    auto expire_time = point.timestamp - std::chrono::milliseconds(num_times);
    while (!speed_history_.empty() && speed_history_.front().timestamp < expire_time)
    {
        speed_history_.pop_front();
    }
}

void Can::SendChannel() {
    while (ros::ok()) {
        bool remotectrl_check_result;
        remotectrl_check_result = false;
        int heart_beat = DP->GetMainDataPtr()->ctrlCanTrig.seqDrive % 20;

        heartbeatDriveQueue_.push(heart_beat);
        if (heartbeatDriveQueue_.size() > MAX_QUEUE_SIZE)
        {
            heartbeatDriveQueue_.pop();
        }
        remotectrl_check_result = CheckHeartBeat(heartbeatDriveQueue_);
        if (!remotectrl_check_result)
        {
            printf("超时，横纵控制指令清零 \n");
            control_can::Control_10  vehicle_ctrl;
            DP->GetCtrl(vehicle_ctrl);
            vehicle_ctrl.Lon_Control_Val = 0;
            vehicle_ctrl.Lat_Control_Val = 0;
            DP->SetCtrl(vehicle_ctrl);
        }

        control_can::Control_10 vehicle_ctrl;
        DP->GetCtrl(vehicle_ctrl);
        DP->SetCtrl(vehicle_ctrl);

        SendCtrData();
        usleep(20 * 1000); 
    }

}

void Can::ReadChannel() {
    struct can_frame  can_frame_rev;
    FeedBack_11 feedback_11; 
    
    while (ros::ok()) { 
        if (transmitter_->ReadCanFrame(can_frame_rev) == -1) {
            printf("ReadCanFrame error");
            continue;
        }

        switch(can_frame_rev.can_id){
            case(CHASSIS_StaFb):{
                if (feedback_11.ChassisFb_GetFromCanFrame(can_frame_rev) == -1) {
                    printf("StaFb_GetFromCanFrame error");
                }
                DP->SetChassis_StaFb(feedback_11);
            }break;
        }
    }
}

void Can::SendCtrData(){
    Control_10 ctrl_can;
    DP->GetCtrl(ctrl_can);

    struct can_frame control_frame;
    control_frame.can_id = Ctrl;
    control_frame.can_dlc = 0x08;

    ctrl_can.ctrl_ConvertToCanFrame(control_frame);
    transmitter_->WriteCanFrame(control_frame);
}


void Can::FaultDecisionBasedSystem() {
  return;
}

bool Can::CheckHeartBeat(const std::queue<int>& heartbeatQueue)
{
    if (heartbeatQueue.size() < MAX_QUEUE_SIZE)
    {
        return true; 
    }
    int consecutiveMissing = 0;
    std::queue<int> tempQueue = heartbeatQueue;

    int  previousHeartbeat = tempQueue.front();
    tempQueue.pop();

    while (!tempQueue.empty())
    {
        int currentHeartbeat = tempQueue.front();
        tempQueue.pop();
        if (currentHeartbeat == previousHeartbeat)
        {
            consecutiveMissing++;
            if (consecutiveMissing > MAX_QUEUE_SIZE - 2)
            {
                printf("consecutiveMissing = 超时 = %d", consecutiveMissing);
                return false;
            }
        }
        else
        {
            consecutiveMissing = 0;
        }
        previousHeartbeat = currentHeartbeat;
        
    }
    return true;
}
} // namespace driver
