

#ifndef __CAN_H__
#define __CAN_H__


//c++ header
#include <functional>
#include <thread>
#include <memory>
#include <string>
#include <ctime>
#include <vector>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <thread>
#include <queue>
#include "ros/ros.h"

#include "datapool.h"
#include "can_protocol.h"
// #include "include/transmitter_base.h"
#include "include/socket_can_transmitter.h"
#include "include/rosbridge.h"
#include "common/config_manager/include/config_manager.h"

#define MAX_QUEUE_SIZE 10

namespace control_can {


class Can {
public:
    Can();
    virtual ~Can();
    int Init();
    int Start();

    void updateSpeed(float current_speed);
    void calculateMileage();
    float getCurrentMileage();

private:
    std::shared_ptr<CommunicationMethd> communication_;
    std::shared_ptr<TransmitterBase> transmitter_;
    std::thread rev_thread_, send_thread_,calculateMileage_thread_,fault_loop_;
    std::vector<std::string> current_node_;
    std::queue<int> heartbeatDriveQueue_;
    std::queue<int> heartbeatBrakeQueue_;
    std::string configPath = PROJECT_SOURCE_DIR + std::string("/conf/chassisParameter.yaml");

    DataPool *DP;
    ConfigManager *ConfigM;

    struct SpeedPoint {
        std::chrono::time_point<std::chrono::system_clock> timestamp;
        float speed; // 单位：m/s
    };
    std::chrono::time_point<std::chrono::system_clock> last_calc_time_;
    std::deque<SpeedPoint> speed_history_;
    float current_mileage_ = 0.0f;
    std::mutex data_mutex_;
    float t_interval;

    double currentRemoteSpeed = 0;  // 遥控器当前速度值

    int InitVars();

    bool readMileageConfigParameter();
    bool writeMileageConfigParameter();

    void takeModeActual();
    void remoteSpeedCtr();

    void ReadChannel();
    void SendChannel();

    void FaultDecisionBasedSystem();

    bool CheckHeartBeat(const std::queue<int>& heartbeatQueue);
    void SendCtrData();
  
};

}  //namespace control_can

#endif
