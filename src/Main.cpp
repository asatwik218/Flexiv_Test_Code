#include "flexiv_tests/step_sensing_test.h"
#include "flexiv_tests/velocity_tracking_test.h"
#include <iostream>
#include <conio.h>
#include <thread>
#include <chrono>
#include <array>
#include <vector>

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
    // Choose which scenario(s) to run. Pick one or build a vector to run back-to-back.
    bool runAll = true; // toggle to run all presets or just one

    std::vector<VelocityTrackingTest::Scenario> scenarios;
    if (runAll) {
        scenarios.push_back(VelocityTrackingTest::MakeStaticOnce());
        scenarios.push_back(VelocityTrackingTest::MakeStaticLoop(3.0));
        scenarios.push_back(VelocityTrackingTest::MakeConstantSweep(0.1, 3.0));
        scenarios.push_back(VelocityTrackingTest::MakeSineWaveZ(0.1, 0.5, 6.0, 1.0));
    } else {
        // Example: run only sine-wave with custom amplitude/duration
        scenarios.push_back(VelocityTrackingTest::MakeSineWaveZ(0.05, 0.3, 8.0, 1.0));
    }

    // Choose log file name here per run
    std::string logFile = "velocity_tracking.csv";

    VelocityTrackingTest test(robotSn, std::move(scenarios), /*logIntervalMs=*/1, /*logFilename=*/logFile);
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

    // Wait for ESC key
    while (true) {
        if (test.isFinished()) {
            std::cout << "\n[Main] Test completed. Exiting wait loop...\n";
            test.waitForCompletion();
            break;
        }

        if (_kbhit()) {
            int key = _getch();
            if (key == 27) { // ESC key ASCII code
                std::cout << "\n[Main] ESC pressed. Stopping test...\n";
                test.stop();
                break;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Cleanup resources
    test.cleanup();

    std::cout << "[Main] Test finished. Resources cleaned up.\n";
    return 0;
}
