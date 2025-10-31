#include "flexiv_tests/step_sensing_test.h"
#include <iostream>
#include <conio.h>     
#include <thread>
#include <chrono>

int main()
{
    std::cout << "===== Flexiv Step Sensing Test =====\n";

    // Example robot serial number
    std::string robotSn = "Rizon4s-062820";  

    // Example test parameters
    std::array<double,6> startPos = {300, 0, 500, 0, 0, 0};  // mm, deg
    std::array<double,6> endPos   = {300, 0, 480, 0, 0, 0};  // mm, deg
    std::vector<double> targetForces = {5.0, 10.0};          // N
    std::vector<double> targetVels   = {0.01, 0.02};         // m/s

    // Create the test
    StepSensingTest test(robotSn, startPos, endPos, targetForces, targetVels);

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
