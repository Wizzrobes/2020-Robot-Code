#include <frc/Joystick.h>
#include <cameraserver/CameraServer.h>

#include "Robot.h"
#include "SwerveModule.h"
#include "SwerveTrain.h"
#include "RobotMap.h"
#include "NavX.h"

SwerveModule frontRightModule(R_frontRightDriveMotorCANID, R_frontRightSwerveMotorCANID);
SwerveModule frontLeftModule(R_frontLeftDriveMotorCANID, R_frontLeftSwerveMotorCANID);
SwerveModule rearLeftModule(R_rearLeftDriveMotorCANID, R_rearLeftSwerveMotorCANID);
SwerveModule rearRightModule(R_rearRightDriveMotorCANID, R_rearRightSwerveMotorCANID);

NavX navX(NavX::ConnectionType::kMXP);
SwerveTrain zion(frontRightModule, frontLeftModule, rearLeftModule, rearRightModule, navX);

frc::Joystick *playerOne;

void Robot::RobotInit() {

    playerOne = new frc::Joystick(R_playerOneControllerPort);

    frc::CameraServer::GetInstance()->StartAutomaticCapture();
}
void Robot::RobotPeriodic() {}
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {

    if (playerOne->GetRawButtonPressed(3)) {

        zion.setZeroPosition();
    }
    else if (playerOne->GetRawButton(11)) {

        zion.driveController(playerOne);
    }
    else {

        zion.setDriveSpeed(0);
        zion.setSwerveSpeed(0);
    }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
