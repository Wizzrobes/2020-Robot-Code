 #pragma once

#include <string>

#include <frc/TimedRobot.h>

class Robot : public frc::TimedRobot {

    public:
        void RobotInit() override;
        void RobotPeriodic() override;
        void AutonomousInit() override;
        void AutonomousPeriodic() override;
        void TeleopInit() override;
        void TeleopPeriodic() override;

        void TestPeriodic() override;
};
