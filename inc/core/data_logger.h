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
    // optional header row written once when the file is empty
    void setHeader(std::string header);

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
    std::string header_;
    bool headerWritten_{false};
    std::chrono::high_resolution_clock::time_point startTime_;
};
