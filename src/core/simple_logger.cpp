#include "core/simple_logger.h"
#include <iostream>
#include <filesystem>
#include <stdexcept>
#include <iomanip>

SimpleLogger::SimpleLogger(const std::string& filename)
    : filename_(filename)
{
    // Ensure parent directory exists
    std::filesystem::path filePath(filename);
    if (filePath.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(filePath.parent_path(), ec);
        if (ec) {
            throw std::runtime_error("SimpleLogger: Failed to create directory: " + ec.message());
        }
    }

    // Open file for writing (truncate if exists)
    file_.open(filename, std::ios::out | std::ios::trunc);
    if (!file_.is_open()) {
        throw std::runtime_error("SimpleLogger: Failed to open file: " + filename);
    }

    // Set precision for floating point output
    file_ << std::setprecision(6) << std::fixed;

    // Start background writer thread
    running_ = true;
    writerThread_ = std::thread(&SimpleLogger::writerLoop, this);
}

SimpleLogger::~SimpleLogger() {
    close();
}

void SimpleLogger::header(const std::string& headerRow) {
    if (headerWritten_) {
        std::cerr << "[SimpleLogger] Warning: header() called multiple times, ignoring.\n";
        return;
    }

    // Store header and count expected columns
    expectedColumns_ = 1; // Start at 1 (count commas + 1)
    for (char c : headerRow) {
        if (c == ',') ++expectedColumns_;
    }

    // Write header with timestamp as first column (directly to file, before thread processes rows)
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        file_ << "timestamp_ms," << headerRow << "\n";
        file_.flush();
    }
    headerWritten_ = true;

    // Reset start time when header is set (logging begins)
    resetStartTime();
}

void SimpleLogger::resetStartTime() {
    startTime_ = std::chrono::high_resolution_clock::now();
    startTimeSet_ = true;
}

void SimpleLogger::appendValue(double value) {
    if (currentColumnCount_ > 0) {
        rowBuffer_ << ",";
    }
    rowBuffer_ << value;
    ++currentColumnCount_;
}

void SimpleLogger::enqueueRow() {
    // Don't enqueue if logger is closing
    if (!running_.load()) {
        rowBuffer_.str("");
        rowBuffer_.clear();
        currentColumnCount_ = 0;
        return;
    }

    // Ensure start time is set (in case header() was not called)
    if (!startTimeSet_) {
        resetStartTime();
    }

    // Capture timestamp NOW (in control thread) for accurate timing
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime_).count();

    // Check column count mismatch
    if (headerWritten_ && currentColumnCount_ != expectedColumns_) {
        if (!mismatchWarned_) {
            std::cerr << "[SimpleLogger] Warning: Row has " << currentColumnCount_
                      << " values, expected " << expectedColumns_ << " (based on header). "
                      << "This warning will not repeat.\n";
            mismatchWarned_ = true;
        }

        // Pad with empty values if too few columns
        while (currentColumnCount_ < expectedColumns_) {
            rowBuffer_ << ",";
            ++currentColumnCount_;
        }
    }

    // Create row and move buffer content
    LogRow row;
    row.timestamp_ms = elapsed;
    row.data = rowBuffer_.str();

    // Reset buffer for next row
    rowBuffer_.str("");
    rowBuffer_.clear();
    currentColumnCount_ = 0;

    // Push to queue (minimal lock time)
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        rowQueue_.push(std::move(row));
    }
    queueCV_.notify_one(); // Wake writer thread
}

void SimpleLogger::writerLoop() {
    while (true) {
        LogRow row;
        {
            std::unique_lock<std::mutex> lock(queueMutex_);

            // Wait for data or shutdown signal
            queueCV_.wait(lock, [this] {
                return !rowQueue_.empty() || !running_;
            });

            // If shutting down and queue is empty, exit
            if (!running_ && rowQueue_.empty()) {
                break;
            }

            // If queue is empty but still running, continue waiting
            if (rowQueue_.empty()) {
                continue;
            }

            // Pop row from queue
            row = std::move(rowQueue_.front());
            rowQueue_.pop();
        }

        // Write to file (no lock needed - only this thread writes after header)
        file_ << row.timestamp_ms << "," << row.data << "\n";
        ++rowCount_;
    }

    // Final flush before thread exits
    file_.flush();
}

void SimpleLogger::flush() {
    // Wait for queue to drain
    while (true) {
        {
            std::lock_guard<std::mutex> lock(queueMutex_);
            if (rowQueue_.empty()) {
                file_.flush();
                return;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void SimpleLogger::close() {
    // Check if already closed (atomic read is safe)
    if (!running_.load()) {
        return;
    }

    // Warn if incomplete row pending
    if (currentColumnCount_ > 0) {
        std::cerr << "[SimpleLogger] Warning: Closing with incomplete row ("
                  << currentColumnCount_ << " values pending). Discarding.\n";
        rowBuffer_.str("");
        rowBuffer_.clear();
        currentColumnCount_ = 0;
    }

    // Signal thread to stop (must notify under lock to avoid race)
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        running_ = false;
    }
    queueCV_.notify_one();

    // Wait for thread to finish (it will drain the queue first)
    if (writerThread_.joinable()) {
        writerThread_.join();
    }

    // Close file (safe now - writer thread has exited)
    if (file_.is_open()) {
        file_.close();
    }
}

bool SimpleLogger::isOpen() const {
    return file_.is_open() && running_.load();
}

std::size_t SimpleLogger::rowCount() const {
    return rowCount_.load();
}
