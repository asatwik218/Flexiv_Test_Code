#include "flexiv_tests/step_sensing_test.h"
#include "flexiv_tests/velocity_tracking_test.h"
#include "flexiv_tests/position_streaming_test.h"
#include "flexiv_tests/path_repeatability_test.h"
#include "flexiv_tests/response_time_test.h"
#include "flexiv_tests/payload_test.h"
#include "flexiv_tests/simple_logging_test.h"
#include "flexiv_tests/admittance_test.h"
#include <iostream>
#include <conio.h>
#include <thread>
#include <chrono>
#include <array>

int main()
{
    std::cout << "===== Flexiv Payload Test =====\n";

    // Example robot serial number
    std::string robotSn = "Rizon4s-063157";

/* STEP SENSING TEST (example, currently not used) */

    // std::array<double,6> startPos = {141,-685,0,0,180,0};  // mm, deg
    // std::array<double,6> endPos   = {-132, -685, 70, 0, 180, 0};  // mm, deg
    // std::vector<double> targetForces = {1.0};          // N
    // std::vector<double> targetVels   = {0.05};         // m/s
    // StepSensingTest test(robotSn, startPos, endPos, targetForces, targetVels);

/* STEP SENSING TEST */

/* VELOCITY TRACKING TEST */
    // VelocityTrackingTest test(robotSn);

/* VELOCITY TRACKING TEST */

/* POSITION STREAMING TEST */
    // PositionStreamingTest test(robotSn, {-690, -434, 80 , -0.58 , -179 , -0.30} );

/* POSITION STREAMING TEST */

/* PATH REPEATABILITY TEST */
    // PathRepeatabilityTest test(robotSn);

/* PATH REPEATABILITY TEST */

/* RESPONSE TIME TEST */
    // ResponseTimeTest test(robotSn);

/* RESPONSE TIME TEST */

/* CONTINUOUS LOGGING TEST */
    // ContinuousLoggingTest test(robotSn);

/* CONTINUOUS LOGGING TEST */

/* PAYLOAD TEST */
    // PayloadTest test(robotSn);

/* PAYLOAD TEST */

/* SIMPLE LOGGING TEST */
    // SimpleLoggingTest test(robotSn);

/* SIMPLE LOGGING TEST */

/* ADMITTANCE TEST */

    //Example with custom parameters:
    double pos_m = 1.0;                                      // Virtual mass [kg]
    double pos_k = 50.0;                                     // Stiffness [N/m]   
    const std::array<double,3> high_end ={1, 1, 1};        // High force threshold [N]
    bool isAdmittance = true;                                // Enable force feedback
    uint16_t testDuration = 60;                             // Test duration [s] per interval
    AdmittanceTest test(robotSn,
                        isAdmittance,
                        testDuration,
                        std::array<double,6>{-221,-576,90, 0.0, 180.0, 0.0},
                        std::array<double,6>{-221, -576, 250, 0.0, 180.0, 0.0},
                        pos_m,
                        pos_k,
                        high_end
                    );

/* ADMITTANCE TEST */


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
