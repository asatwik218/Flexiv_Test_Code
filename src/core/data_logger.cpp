#include "core/data_logger.h"
#include <iostream>

#ifdef _WIN32
#include <Windows.h> 
#endif

void DataLogger::configure(const std::string& filename , int intervalMs){
    if(running_) throw std::runtime_error("Cannot Configure while logger is running.");

    intervalMs_ = intervalMs;
    filename_ = filename;
}

void DataLogger::setCallback(std::function<void(std::ostream&)> callback){
    std::lock_guard<std::mutex> lk(mtx_);
    callback_ = std::move(callback);
}

void DataLogger::setHeader(std::string header){
    std::lock_guard<std::mutex> lk(mtx_);
    header_ = std::move(header);
    headerWritten_ = false;
}

void DataLogger::start(){
    if(running_)return ;

#ifdef _WIN32
    timeBeginPeriod(1); // improve Windows sleep resolution
#endif

    if(!file_.is_open()){
         file_.open(filename_);
        if(!file_.is_open()) throw std::runtime_error("Failed to open file");
    }
    if (file_.tellp() == 0 && !header_.empty() && !headerWritten_) {
        file_ << header_ << "\n";
        headerWritten_ = true;
    }

    // Reset the start time to now - this makes timestamps start at 0
    startTime_ = std::chrono::high_resolution_clock::now();

    running_ = true;
    thread_ = std::thread(&DataLogger::loggingLoop, this);
}

void DataLogger::stop(){
    running_ = false;
    if(thread_.joinable()) thread_.join();

#ifdef _WIN32
    timeEndPeriod(1);
#endif

    {   
        std::lock_guard<std::mutex> lk(mtx_);
        if(file_.is_open()) file_.close();
    }
}

DataLogger::~DataLogger(){
    stop();
}

void DataLogger::loggingLoop(){
    auto nextTick = std::chrono::high_resolution_clock::now();

    while(running_){
        nextTick += std::chrono::milliseconds(intervalMs_);

        // Calculate time elapsed since start() was called in milliseconds
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime_).count();

        {
            std::lock_guard<std::mutex> lk(mtx_);
            file_ << elapsed << ",";
            if(callback_) callback_(file_);
            file_<<"\n";
        }

        std::this_thread::sleep_until(nextTick);

    }
}
