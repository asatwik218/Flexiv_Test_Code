#pragma once

#include <string>
#include <ostream>

/**
 * @brief Generic test result structure.
 */
struct TestResult
{
    bool success;
    std::string message;
};

/**
 * @brief Interface for all tests (robotic or otherwise).
 *
 * Defines a unified lifecycle for tests:
 *  - initialise():   prepare any resources or connections
 *  - runTest():      execute the main test logic
 *  - stop():         stop execution safely
 *  - cleanup():      free resources
 *
 * Optionally, tests can implement a log() callback which is periodically called
 * by a DataLogger if the test uses logging.
 */
class ITest
{
public:
    virtual ~ITest() = default;

    /**
     * @brief Prepare the test (connect to robot, allocate memory, etc.)
     * @return true if successful, false otherwise
     */
    virtual bool initialise() = 0;

    /**
     * @brief Execute the test (the main logic).
     * This function should start and stop logging internally if required.
     * @return A TestResult indicating success or failure and a message.
     */
    virtual TestResult runTest() = 0;

    /**
     * @brief Request the test to stop safely (can be called from another thread).
     */
    virtual void stop() = 0;

    /**
     * @brief Clean up after the test (close connections, release memory, etc.)
     */
    virtual void cleanup() = 0;

    /**
     * @brief Optional logging callback — called by DataLogger at a fixed frequency.
     * Default does nothing.
     * @param out Stream to write the log data (usually a CSV row)
     */
    virtual void log(std::ostream& out) {}
};
