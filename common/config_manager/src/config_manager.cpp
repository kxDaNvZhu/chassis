/*
 * Copyright (C) 200X-200X 重庆建设工业（集团）有限责任公司/智控技术研究所
 * All Rights Reserved. 
 * 
 * @FilePath     : /config_manager/src/config_manager.cpp
 * @Description  : config manager
 * @Author       : 闫 深义 ysyjs3@163.com
 * @Date         : 2024-09-20 08:45:14
 * @LastEditors  : 闫 深义 ysyjs3@163.com
 * @LastEditTime : 2024-09-23 10:47:19
 */

#include "common/config_manager/include/config_manager.h"
#include <fstream>

void ConfigManager::setPath(const std::string& path) {
    path_ = path;
    {
        std::lock_guard<std::mutex> fileLock(mutexFile_);
        std::ifstream ifs(path_);
        config_ = fkyaml::node::deserialize(ifs);
        ifs.close();
    }
}

ConfigManager::~ConfigManager() {
    {
        std::lock_guard<std::mutex> lock(mutexBuffer_);
        stopFlag_ = true;
    }
    cv_.notify_all();
    std::cout << "waiting for config dump..." << std::endl;
    workerThread_.join();
}

// 多线程修改需加锁 mutex()
fkyaml::node& ConfigManager::config()
{
    std::lock_guard<std::mutex> lock(mutexConfig_);
    return config_;
}

std::mutex& ConfigManager::mutex()
{
    return mutexConfig_;
}


void ConfigManager::saveConfig(const fkyaml::node& config)
{
    addToBuffer(config);
}

void ConfigManager::addToBuffer(const fkyaml::node& config) {
    {
        std::lock_guard<std::mutex> lock(mutexBuffer_);
        buffer_.push(config);
    }
    cv_.notify_one(); // Notify worker thread
}

void ConfigManager::processBuffer() {
    while (true) {
        fkyaml::node config;
        {
            std::unique_lock<std::mutex> lock(mutexBuffer_);
            cv_.wait(lock, [this] { return !buffer_.empty() || stopFlag_; });

            if (stopFlag_ && buffer_.empty())
                break;

            if (!buffer_.empty()) {
                config = buffer_.front();
                buffer_.pop();

                // Remove duplicates
                while (!buffer_.empty()) {
                    if (buffer_.front() == config) {
                        buffer_.pop();
                    } else {
                        break;
                    }
                }
            }
        }
    
        dump2File(config);
    }
}

void ConfigManager::dump2File(const fkyaml::node& config) {
    std::lock_guard<std::mutex> fileLock(mutexFile_);
    std::ofstream outfile(path_);
    if (!outfile.is_open()) {
        std::cerr << "Error opening file for writing!" << std::endl;
    }
    outfile << config << std::endl;
    outfile.close();
}
