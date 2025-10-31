#pragma once

#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>
#include <string>
#include <fstream>
#include <functional>

class DataLogger{
public:
    static DataLogger& getInstance(){
        static DataLogger instance ;
        return instance;
    }

    DataLogger(const DataLogger&) = delete;
    DataLogger& operator=(const DataLogger&) = delete;

    //configure Logger
    void configure(const std::string& filename, int intervalMs = 1);

    //set callback: what to do each log tick
    void setCallback(std::function<void(std::ostream&)> callback);

    void start();
    void stop();

private:
    DataLogger() = default;
    ~DataLogger();

    void loggingLoop();

    std::string filename_;
    std::ofstream file_;
    std::atomic<bool> running_{false};
    std::thread thread_;
    int intervalMs_{1};

    std::mutex mtx_;
    std::function<void(std::ostream&)> callback_;
};