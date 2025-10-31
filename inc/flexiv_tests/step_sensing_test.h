#pragma once

#include "flexiv_tests/flexiv_robot_test.h"

class StepSensingTest : public FlexivRobotTest{

public:
    
    StepSensingTest(const std::string& robotSn,   
        std::array<double,6> startPos_mm_deg,
        std::array<double,6> endPos_mm_deg,
        std::vector<double> targetForces_N,
        std::vector<double> targetVelocities_mPsec
    );
        
protected:
    void performTest() override;

private:
    std::array<double,7> startPose_;
    std::array<double,7> endPose_;
    std::vector<double> targetForces_;
    std::vector<double> targetVelocities_;

};