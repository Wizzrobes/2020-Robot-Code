#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <cameraserver/CameraServer.h>

#include "Robot.h"
#include "SwerveModule.h"
#include "SwerveTrain.h"
#include "Launcher.h"
#include "RobotMap.h"

SwerveModule frontRightModule(R_frontRightDriveMotorCANID, R_frontRightSwerveMotorCANID);
SwerveModule frontLeftModule(R_frontLeftDriveMotorCANID, R_frontLeftSwerveMotorCANID);
SwerveModule rearLeftModule(R_rearLeftDriveMotorCANID, R_rearLeftSwerveMotorCANID);
SwerveModule rearRightModule(R_rearRightDriveMotorCANID, R_rearRightSwerveMotorCANID);
SwerveTrain zion(frontRightModule, frontLeftModule, rearLeftModule, rearRightModule);

Launcher launcher(R_launcherIndexMotorCANID, R_launcherLaunchMotorCANID);

frc::Joystick *playerOne;
frc::XboxController *playerTwo;

void Robot::RobotInit() {

    playerOne = new frc::Joystick(R_playerOneControllerPort);
    playerTwo = new frc::XboxController(R_playerTwoControllerPort);

    frc::CameraServer::GetInstance()->StartAutomaticCapture();
}
void Robot::RobotPeriodic() {}
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {

    if (playerOne->GetRawButtonPressed(3)) {

        zion.setSwerveZeroPosition();
    }
    else if (playerOne->GetRawButton(11)) {

        zion.driveController(playerOne);
    }
    else {

        zion.setDriveSpeed(0);
        zion.setSwerveSpeed(0);
    }


    if (playerTwo->GetXButton) {

        launcher.setIndexSpeed(playerTwo->GetY(frc::GenericHID::kLeftHand));
    }
    if (playerTwo->GetYButton) {

        launcher.setLaunchSpeed(playerTwo->GetY(frc::GenericHID::kLeftHand));
    }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
