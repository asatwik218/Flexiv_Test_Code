#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <chrono>
#include <mutex>
#include <array>
#include <vector>
#include <queue>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <type_traits>
#include <Eigen/Core>

class SimpleLogger;

// Custom endl manipulator - triggers row write with timestamp
SimpleLogger& endl(SimpleLogger& logger);

class SimpleLogger {
public:
    // Constructor: opens file, creates directories, starts background thread
    // Throws std::runtime_error if file cannot be opened
    explicit SimpleLogger(const std::string& filename);

    // Destructor: stops thread, drains queue, closes file
    ~SimpleLogger();

    // Non-copyable, non-movable
    SimpleLogger(const SimpleLogger&) = delete;
    SimpleLogger& operator=(const SimpleLogger&) = delete;
    SimpleLogger(SimpleLogger&&) = delete;
    SimpleLogger& operator=(SimpleLogger&&) = delete;

    // Set CSV header (called once before logging begins)
    // Automatically prepends "timestamp_ms" as first column
    // Example: logger.header("fx,fy,fz") -> writes "timestamp_ms,fx,fy,fz"
    void header(const std::string& headerRow);

    // Stream single values (int, double, float, etc.)
    template<typename T>
    std::enable_if_t<std::is_arithmetic_v<T>, SimpleLogger&>
    operator<<(const T& value);

    // Stream std::array<T, N> - auto-expands to N columns
    template<typename T, std::size_t N>
    SimpleLogger& operator<<(const std::array<T, N>& arr);

    // Stream std::vector<T> - auto-expands to N columns
    template<typename T>
    SimpleLogger& operator<<(const std::vector<T>& vec);

    // Stream C-style array - auto-expands
    template<typename T, std::size_t N>
    SimpleLogger& operator<<(const T (&arr)[N]);

    // Stream Eigen vectors (Vector3d, VectorXd, Matrix<double,6,1>, etc.)
    template<typename Derived>
    SimpleLogger& operator<<(const Eigen::MatrixBase<Derived>& vec);

    // Manipulator support (for endl)
    SimpleLogger& operator<<(SimpleLogger& (*manip)(SimpleLogger&));

    // Force flush pending rows to disk
    void flush();

    // Stop thread, drain queue, close file
    void close();

    // Check if logger is in valid state
    bool isOpen() const;

    // Get number of rows written (excluding header) - thread-safe
    std::size_t rowCount() const;

private:
    friend SimpleLogger& endl(SimpleLogger& logger);

    // Internal: append single value to row buffer
    void appendValue(double value);

    // Push row to queue (non-blocking, called from control thread)
    void enqueueRow();

    // Background thread function - pops from queue and writes to file
    void writerLoop();

    // Reset start time (called when header is set)
    void resetStartTime();

    // Row data structure for queue
    struct LogRow {
        int64_t timestamp_ms;
        std::string data;
    };

    // File (accessed only by background thread)
    std::ofstream file_;
    std::string filename_;

    // Row buffer (owned by caller thread until enqueue)
    std::ostringstream rowBuffer_;
    std::size_t currentColumnCount_{0};

    // Thread-safe queue for pending rows
    std::queue<LogRow> rowQueue_;
    std::mutex queueMutex_;
    std::condition_variable queueCV_;

    // Background writer thread
    std::thread writerThread_;
    std::atomic<bool> running_{false};

    // Timing and header
    std::chrono::high_resolution_clock::time_point startTime_;
    bool startTimeSet_{false};
    std::size_t expectedColumns_{0};
    std::atomic<std::size_t> rowCount_{0};
    bool headerWritten_{false};
    bool mismatchWarned_{false};
};

// ============================================================================
// Template implementations (must be in header)
// ============================================================================

template<typename T>
std::enable_if_t<std::is_arithmetic_v<T>, SimpleLogger&>
SimpleLogger::operator<<(const T& value) {
    appendValue(static_cast<double>(value));
    return *this;
}

template<typename T, std::size_t N>
SimpleLogger& SimpleLogger::operator<<(const std::array<T, N>& arr) {
    for (std::size_t i = 0; i < N; ++i) {
        appendValue(static_cast<double>(arr[i]));
    }
    return *this;
}

template<typename T>
SimpleLogger& SimpleLogger::operator<<(const std::vector<T>& vec) {
    for (const auto& val : vec) {
        appendValue(static_cast<double>(val));
    }
    return *this;
}

template<typename T, std::size_t N>
SimpleLogger& SimpleLogger::operator<<(const T (&arr)[N]) {
    for (std::size_t i = 0; i < N; ++i) {
        appendValue(static_cast<double>(arr[i]));
    }
    return *this;
}

template<typename Derived>
SimpleLogger& SimpleLogger::operator<<(const Eigen::MatrixBase<Derived>& vec) {
    for (Eigen::Index i = 0; i < vec.size(); ++i) {
        appendValue(static_cast<double>(vec(i)));
    }
    return *this;
}

// Inline manipulator implementation
inline SimpleLogger& SimpleLogger::operator<<(SimpleLogger& (*manip)(SimpleLogger&)) {
    return manip(*this);
}

// Inline endl implementation
inline SimpleLogger& endl(SimpleLogger& logger) {
    logger.enqueueRow();
    return logger;
}
