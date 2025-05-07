/*
 * Copyright (C) 200X-200X 重庆建设工业（集团）有限责任公司/智控技术研究所
 * All Rights Reserved. 
 * 
 * @FilePath     : /config_manager/src/config_manager.hpp
 * @Description  : config manager: 1) 提供读写配置文件的接口; 2) 异步写入防止阻塞UI; 3) 线程安全
 * @Author       : 闫 深义 ysyjs3@163.com
 * @Date         : 2024-09-20 08:45:14
 * @LastEditors  : 闫 深义 ysyjs3@163.com
 * @LastEditTime : 2024-09-23 10:46:25
 */

#pragma once

#include <iostream>
#include <string>
#include <queue>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include "fkYAML/node.hpp"


class ConfigManager {


public:                                                                  
static ConfigManager *Instance() 
{                                          
static ConfigManager *instance = nullptr;                                 
if (!instance) {                                                      
    static std::once_flag flag;                                         
    std::call_once(flag,[&] { instance = new (std::nothrow) ConfigManager(); }); 
}                                                                     
return instance;                                                      
}                                                                       
                                                                        
private:                                                                 
ConfigManager(const ConfigManager &) = delete;                                  
ConfigManager &operator=(const ConfigManager &) = delete;

public:
    ConfigManager(){
        workerThread_ = std::thread(&ConfigManager::processBuffer, this);
    }
    ~ConfigManager();
    void setPath(const std::string& path);
    fkyaml::node& config();
    std::mutex& mutex();

    void saveConfig(const fkyaml::node& config);

private:
    void addToBuffer(const fkyaml::node& config);
    void processBuffer();
    void dump2File(const fkyaml::node& config);

private:
    std::string path_;
    std::mutex mutexFile_;  // for file

    fkyaml::node config_;
    std::mutex mutexConfig_;  // for config
    
    std::queue<fkyaml::node> buffer_;
    std::mutex mutexBuffer_;      // for config buffer
    std::condition_variable cv_;
    // std::atomic<bool> stopFlag_ = false;
    std::atomic<bool> stopFlag_{ATOMIC_VAR_INIT(false)};  // 使用 ATOMIC_VAR_INIT 宏初始化
    std::thread workerThread_;
};
