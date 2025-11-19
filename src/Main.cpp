#include "flexiv_tests/step_sensing_test.h"
#include "flexiv_tests/velocity_tracking_test.h"
#include <iostream>
#include <conio.h>
#include <thread>
#include <chrono>
#include <array>

int main()
{
    std::cout << "===== Flexiv Velocity Tracking Test =====\n";

    // Example robot serial number
    std::string robotSn = "Rizon10s-062287";  

/* STEP SENSING TEST (example, currently not used) */
    
    // std::array<double,6> startPos = {300, 0, 500, 0, 0, 0};  // mm, deg
    // std::array<double,6> endPos   = {300, 0, 480, 0, 0, 0};  // mm, deg
    // std::vector<double> targetForces = {5.0, 10.0};          // N
    // std::vector<double> targetVels   = {0.01, 0.02};         // m/s
    // StepSensingTest test(robotSn, startPos, endPos, targetForces, targetVels);

/* STEP SENSING TEST */

/* VELOCITY TRACKING TEST */
    VelocityTrackingTest test(robotSn);

/* VELOCITY TRACKING TEST */


    // Initialize robot
    if (!test.initialise()) {
        std::cerr << "[Main] Failed to initialize robot.\n";
        return -1;
    }

    // Start test (asynchronous)
    TestResult result = test.runTest();
    std::cout << "[Main] " << result.message << "\n";

    std::cout << "\nPress ESC to stop the test safely...\n";

    // Separate thread monitors ESC keypress and calls stop() to interrupt the test
    std::thread inputThread([&test]() {
        while (!test.isFinished()) {
            if (_kbhit()) {
                int key = _getch();
                if (key == 27) {
                    std::cout << "\n[Main] ESC pressed. Stopping test...\n";
                    test.stop();
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });

    test.waitForCompletion();
    if (inputThread.joinable()) {
        inputThread.join();
    }

    // Cleanup resources
    test.cleanup();

    std::cout << "[Main] Test finished. Resources cleaned up.\n";
    return 0;
}
