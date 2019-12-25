#include <iostream>

#include <frc/XboxController.h>

#include "Robot.h"
#include "RobotMap.h"

frc::XboxController* playerOne;

void Robot::RobotInit() {

    playerOne = new frc::XboxController(R_playerOneControllerPort);
}
void Robot::RobotPeriodic() {}
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
