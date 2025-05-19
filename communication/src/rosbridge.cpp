#include "include/rosbridge.h"

Rosbridge::Rosbridge() {
    DP = DataPool::Instance();
    
	// sub_VehicleCtrl_ = nh_.subscribe("/VehicleCtrl", 10, &Rosbridge::VehicleCtrlCallBack,this);
    sub_vehicletocan_ = nh_.subscribe("/vehicletocan", 10, &Rosbridge::VehicleToCanCallBack,this);
	sub_remotecontrol_ = nh_.subscribe("/remotecontrol", 10, &Rosbridge::RemoteControlCallBack,this);
	
    pub_chassis_fb_ = nh_.advertise<exterior_common_msgs::Chassis_Fb>("/chassis_fb", 10);
    pub_cause_vehicle_halt_fb_ = nh_.advertise<exterior_common_msgs::cause_vehicle_halt_fb>("/cause_vehicle_halt_fb", 10);
    pub_carstatus_ = nh_.advertise<exterior_common_msgs::CarStatus>("/tpvehiclestate", 10);
}
      
int Rosbridge::Start() {
    ros::Rate rate(20);
    while (ros::ok()) {
        communication_->SpinOnce();
        communication_->Publish();
        rate.sleep();
    }
    return 0;
}

int Rosbridge::Init() {
    communication_ = std::make_shared<Rosbridge>();
    // RETURN_VAL_IF_NULL(communication_, -1);
    InitLog();
	return 0;
}

int Rosbridge::SpinOnce() {
	ros::spinOnce();
	return 0;
}

int Rosbridge::InitLog(){
  fmt::print("hello world, logger version = {}\n", LOGFM.getVersion());
  std::string timeCur = getFormattedTime().substr(0, 14);

//   std::cout << "getFormattedTime: " << getFormattedTime() << "\n"; // 输出当前时间戳，用于调试

  // 若需要，则设置相关参数
  std::string projectSourceDir = PROJECT_SOURCE_DIR;
  std::string log_file =  projectSourceDir + std::string("/../../log/chassis/"+timeCur+"chassis.log");
  size_t max_files = 3;                   // 备份数量限制
  size_t max_file_size = 1024*1024*200;    // 文件大小限制(字节)
  int level = 2;                          // 打印/记录的最低日志等级: 1-debug, 2-info
  LOGFM.getConfig()
      .setFile(log_file, max_files, max_file_size)
      .setLevel(level);

  // 创建 logger, 创建之后上面的config设置才生效
  LOGFM.create();
  FMLoggerS::LogHelper logHelper(__func__);  

  LOGFM_INFO("InitLog, logger version = "+LOGFM.getVersion());
  return 0;
}

int Rosbridge::Publish() {
	

    control_can::FeedBack_11  feedback_11;
    DP->GetChassis_StaFb(feedback_11);
    exterior_common_msgs::Chassis_Fb chassis_fb_msg;
    // chassis_fb_msg.velocity = feedback_11.velocity;
    // chassis_fb_msg.angular_velocity = feedback_11.angular_velocity;
    // chassis_fb_msg.forward_tilt_arm_angle = feedback_11.forward_tilt_arm_angle;
    // chassis_fb_msg.rearward_tilt_arm_angle = feedback_11.rearward_tilt_arm_angle;
    // chassis_fb_msg.soc = feedback_11.soc;
    // chassis_fb_msg.chassis_fault = feedback_11.chassis_fault;

    chassis_fb_msg.lon_velocity_fb = feedback_11.Lon_Velocity_Fb;
    chassis_fb_msg.front_flip_arm_angle_fb = feedback_11.Front_Flip_Arm_Angle_Fb;
    chassis_fb_msg.rear_flip_arm_angle_fb = feedback_11.Rear_Flip_Arm_Angle_Fb;
    chassis_fb_msg.chassis_battery_soc_fb = feedback_11.Chassis_Battery_SOC_Fb;
    chassis_fb_msg.chassis_fault_word_fb = feedback_11.Chassis_Fault_Word_Fb;

    chassis_fb_msg.total_mileage = DP->GetMainDataPtr()->total_mileage_;
    chassis_fb_msg.current_mileage = DP->GetMainDataPtr()->current_mileage_;
    chassis_fb_msg.control_mode_fb = DP->GetMainDataPtr()->from_remote.control_mode;  // 临时使用实际接收的，因为故障判断机制还没写

    pub_chassis_fb_.publish(chassis_fb_msg);

    // LOGFM_INFO_C(" chassis feedback longitudinal velocity: %f, angular velocity: %f \n",
    // feedback_11.velocity, feedback_11.angular_velocity);

    LOGFM_INFO_C(" chassis feedback longitudinal velocity: %f \n",
        feedback_11.Lon_Velocity_Fb);

    exterior_common_msgs::CarStatus car_status_msg;
    if(chassis_fb_msg.lon_velocity_fb < 0){
        car_status_msg.shiftlvlposition = 1;
    }
    else{
        car_status_msg.shiftlvlposition = 3;
    }
    car_status_msg.velocity = std::fabs(chassis_fb_msg.lon_velocity_fb);
    
    car_status_msg.system_status = 0;

    if (DP->GetMainDataPtr()->is_steering_in_place_Ctrl)
    {
        car_status_msg.shiftlvlposition = 2;
    }
    if (!DP->GetMainDataPtr()->is_steering_in_place_Ctrl && (chassis_fb_msg.lon_velocity_fb == 0))
    {
        car_status_msg.shiftlvlposition = 0;
    }

    pub_carstatus_.publish(car_status_msg);

    exterior_common_msgs::cause_vehicle_halt_fb cause_halt_fb_msg;
    cause_halt_fb_msg.remote_control_mode = DP->GetMainDataPtr()->cause_of_vehicle_halt.remote_control_mode; // 控制模式

    cause_halt_fb_msg.remote_emergency_stop = DP->GetMainDataPtr()->cause_of_vehicle_halt.remote_emergency_stop; // 遥控急停指令
    cause_halt_fb_msg.remote_parking = DP->GetMainDataPtr()->cause_of_vehicle_halt.remote_parking;               // 遥控驻车指令
    cause_halt_fb_msg.remote_brake_enable = DP->GetMainDataPtr()->cause_of_vehicle_halt.remote_brake_enable;     // 遥控制动使能
    cause_halt_fb_msg.remote_brake_pressure = DP->GetMainDataPtr()->cause_of_vehicle_halt.remote_brake_pressure; // 遥控制动压力

    cause_halt_fb_msg.ctrl_emergency_stop = DP->GetMainDataPtr()->cause_of_vehicle_halt.ctrl_emergency_stop; // 自主控制急停指令
    cause_halt_fb_msg.ctrl_parking = DP->GetMainDataPtr()->cause_of_vehicle_halt.ctrl_parking;               // 自主控制驻车指令
    cause_halt_fb_msg.ctrl_brake_enable = DP->GetMainDataPtr()->cause_of_vehicle_halt.ctrl_brake_enable;     // 自主控制制动使能
    cause_halt_fb_msg.ctrl_brake_pressure = DP->GetMainDataPtr()->cause_of_vehicle_halt.ctrl_brake_pressure; // 自主控制制动压力

    cause_halt_fb_msg.chassis_fault_fb = feedback_11.Chassis_Fault_Word_Fb; // 车辆底盘故障反馈

    pub_cause_vehicle_halt_fb_.publish(cause_halt_fb_msg);
    
    return 0;
}

void Rosbridge::VehicleToCanCallBack(const exterior_common_msgs::VehicleToCan::ConstPtr& msg){
    
    // control_can::Control_10  vehicle_ctrl;
    DP->GetMainDataPtr()->ctrlCanTrig.seqDrive = msg->header.seq;
    // DP->GetCtrl(vehicle_ctrl);
    // if(DP->GetMainDataPtr()->from_remote.control_mode == 1){//0是遥控，1是自主
    //     return;
    // }
    // DP->GetMainDataPtr()->ctrlCanTrig.seqDrive = msg->header.seq;

    // vehicle_ctrl.Lon_Control_Val = msg->loncontrol_ctrl;
    // vehicle_ctrl.Lat_Control_Val = msg->latcontrol_ctrl;
    // vehicle_ctrl.Speed_Mode = (int)msg->speed_mode_ctrl;
    // vehicle_ctrl.Front_Light = (int)msg->front_light_ctrl;
    // vehicle_ctrl.Brake_Enable = (int)msg->brake_enable_ctrl;
    // vehicle_ctrl.Relay_Release_1 = (int)msg->relay_release_1_ctrl;
    // vehicle_ctrl.Relay_Release_2 = (int)msg->relay_release_2_ctrl;
    // vehicle_ctrl.Control_Mode = (int)msg->control_mode_ctrl;

    // DP->GetMainDataPtr()->from_ctrl.Lon_Control_Val_Ctrl = msg->loncontrol_ctrl;
    // DP->GetMainDataPtr()->from_ctrl.Lat_Control_Val_Ctrl = msg->latcontrol_ctrl;
    // DP->GetMainDataPtr()->from_ctrl.Speed_Mode_Ctrl = (int)msg->speed_mode_ctrl;
    // DP->GetMainDataPtr()->from_ctrl.Front_Light_Ctrl = (int)msg->front_light_ctrl;
    // DP->GetMainDataPtr()->from_ctrl.Brake_Enable_Ctrl = (int)msg->brake_enable_ctrl;
    // DP->GetMainDataPtr()->from_ctrl.Relay_Release_1_Ctrl = (int)msg->relay_release_1_ctrl;
    // DP->GetMainDataPtr()->from_ctrl.Relay_Release_2_Ctrl = (int)msg->relay_release_2_ctrl;
    // DP->GetMainDataPtr()->from_ctrl.Control_Mode_Ctrl = (int)msg->control_mode_ctrl;

    DP->GetMainDataPtr()->from_ctrl.Lon_Control_Val_Ctrl = msg->loncontrol_ctrl;
    DP->GetMainDataPtr()->from_ctrl.Lat_Control_Val_Ctrl = msg->latcontrol_ctrl;
    
    DP->GetMainDataPtr()->from_ctrl.Speed_Mode_Ctrl = msg->speed_mode;
    DP->GetMainDataPtr()->from_ctrl.Car_Lights_Ctrl = msg->car_lights;
    DP->GetMainDataPtr()->from_ctrl.Brake_Enable_Ctrl = msg->brake_enable;
    DP->GetMainDataPtr()->from_ctrl.Strobe_Light_Ctrl = msg->strobe_light;
    DP->GetMainDataPtr()->from_ctrl.Parking_Ctrl = msg->parking;
    DP->GetMainDataPtr()->from_ctrl.Emergency_Stop_Ctrl = msg->emergency_stop;
    DP->GetMainDataPtr()->from_ctrl.Request_Ctrl = msg->request;
    DP->GetMainDataPtr()->from_ctrl.Steering_Mode_Switching_Ctrl = msg->steering_mode_switching;
    DP->GetMainDataPtr()->from_ctrl.Gear_Position_Ctrl = msg->gear_position;
    
    DP->GetMainDataPtr()->from_ctrl.Brake_Pressure_Ctrl = msg->brake_pressure;

    // DP->SetCtrl(vehicle_ctrl);
    // LOGFM_INFO_C("chassis command longitudinal velocity: %f, angular velocity: %f \n",
    // vehicle_ctrl.Lon_Control_Val, vehicle_ctrl.Lat_Control_Val);

    DP->GetMainDataPtr()->is_steering_in_place_Ctrl = msg->is_steering_in_place_ctrl;

    return;
}

void Rosbridge::RemoteControlCallBack(const exterior_common_msgs::RemoteControl::ConstPtr& msg){
    // std::cout << "control mode:" << msg->control_mode << "\n";
    DP->GetMainDataPtr()->ctrlCanTrig.seqDrive = msg->header.seq;
    // control_can::Control_10  vehicle_ctrl;
    // DP->GetCtrl(vehicle_ctrl);
    // if((int)msg->control_mode == 0){
    //     vehicle_ctrl.Lon_Control_Val = msg->loncontrol;
    //     vehicle_ctrl.Lat_Control_Val = msg->latcontrol;
    // }
    // vehicle_ctrl.Speed_Mode = (int)msg->speed_mode;
    // vehicle_ctrl.Front_Light = (int)msg->front_light;
    // vehicle_ctrl.Brake_Enable = (int)msg->brake_enable;
    // vehicle_ctrl.Relay_Release_1 = (int)msg->relay_release_1;
    // vehicle_ctrl.Relay_Release_2 = (int)msg->relay_release_2;

    DP->GetMainDataPtr()->from_remote.remote_time = msg->header.stamp.toNSec() / 1000000;  // 转为毫秒（1e-3 秒）

    // DP->GetMainDataPtr()->from_remote.Lon_Control_Val = msg->loncontrol;
    // DP->GetMainDataPtr()->from_remote.Lat_Control_Val = msg->latcontrol;
    // DP->GetMainDataPtr()->from_remote.Speed_Mode = (int)msg->speed_mode;
    // DP->GetMainDataPtr()->from_remote.Front_Light = (int)msg->front_light;
    // DP->GetMainDataPtr()->from_remote.Brake_Enable = (int)msg->brake_enable;
    // DP->GetMainDataPtr()->from_remote.Relay_Release_1 = (int)msg->relay_release_1;
    // DP->GetMainDataPtr()->from_remote.Relay_Release_2 = (int)msg->relay_release_2;

    DP->GetMainDataPtr()->from_remote.control_mode = msg->control_mode;
    // DP->GetMainDataPtr()->from_remote.parking = msg->parking;
    // DP->GetMainDataPtr()->from_remote.emergency_stop = msg->emergency_stop;

    DP->GetMainDataPtr()->from_remote.Lon_Control_Val = msg->lon_control_val;
    DP->GetMainDataPtr()->from_remote.Lat_Control_Val = msg->lat_control_val;
    
    DP->GetMainDataPtr()->from_remote.Speed_Mode = msg->speed_mode;
    DP->GetMainDataPtr()->from_remote.Car_Lights = msg->car_lights;
    DP->GetMainDataPtr()->from_remote.Brake_Enable = msg->brake_enable;
    DP->GetMainDataPtr()->from_remote.Strobe_Light = msg->strobe_light;
    DP->GetMainDataPtr()->from_remote.Parking = msg->parking;
    DP->GetMainDataPtr()->from_remote.Emergency_Stop = msg->emergency_stop;
    DP->GetMainDataPtr()->from_remote.Request = msg->request;
    DP->GetMainDataPtr()->from_remote.Steering_Mode_Switching = msg->steering_mode_switching;
    DP->GetMainDataPtr()->from_remote.Gear_Position = msg->gear_position;
    
    DP->GetMainDataPtr()->from_remote.Brake_Pressure = msg->brake_pressure;

    // if (msg->emergency_stop || msg->parking)
    // {
    //     vehicle_ctrl.Lon_Control_Val = 0;
    //     vehicle_ctrl.Lat_Control_Val = 0;
    //     vehicle_ctrl.Brake_Enable = 1;
    // }    

    // DP->GetMainDataPtr()->MODE_CTRL_FROM_REMOTE = static_cast<control_can::MODE_CONTROL_CHASSIS>(msg->control_mode);

	// DP->SetCtrl(vehicle_ctrl);
    // LOGFM_INFO_C("RemoteControl longitudinal velocity: %f, angular velocity: %f, from remote control mode: %d \n",
    // vehicle_ctrl.Lon_Control_Val, vehicle_ctrl.Lat_Control_Val, DP->GetMainDataPtr()->MODE_CTRL_FROM_REMOTE);
    return;
}

std::string Rosbridge::getFormattedTime() {
    // 获取当前时间点（含毫秒）
    auto now = std::chrono::system_clock::now();
    
    // 转换为 time_t 和 tm 结构体（线程安全）
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time;
    localtime_r(&now_time, &local_time);
    
    // 提取毫秒部分
    auto since_epoch = now.time_since_epoch();
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(since_epoch) % 1000;
    
    // 格式化时间字符串
    std::ostringstream oss;
    oss << std::put_time(&local_time, "%Y_%m%d_%H%M:%S")  // 基础格式
        << "." << std::setfill('0') << std::setw(3) << milliseconds.count();  // 毫秒
    
    return oss.str();
}
