#include <cameraserver/CameraServer.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>

#include "Launcher.h"
#include "NavX.h"
#include "Robot.h"
#include "RobotMap.h"
#include "SwerveModule.h"
#include "SwerveTrain.h"

Launcher launcher(R_launcherIndexMotorCANID, R_launcherLaunchMotorCANID);
NavX navX(NavX::ConnectionType::kMXP);

SwerveModule frontRightModule(R_frontRightDriveMotorCANID, R_frontRightSwerveMotorCANID);
SwerveModule frontLeftModule(R_frontLeftDriveMotorCANID, R_frontLeftSwerveMotorCANID);
SwerveModule rearLeftModule(R_rearLeftDriveMotorCANID, R_rearLeftSwerveMotorCANID);
SwerveModule rearRightModule(R_rearRightDriveMotorCANID, R_rearRightSwerveMotorCANID);
SwerveTrain zion(frontRightModule, frontLeftModule, rearLeftModule, rearRightModule, navX);

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

        zion.setZeroPosition();
    }
    else if (playerOne->GetRawButton(11)) {

        zion.driveController(playerOne);
    }
    else {

        zion.setDriveSpeed(0);
        zion.setSwerveSpeed(0);
    }


    if (playerTwo->GetXButton()) {

        launcher.setLaunchSpeed(playerTwo->GetY(frc::GenericHID::kLeftHand));
    }
    if (playerTwo->GetYButton()) {

        m_launcherIndexSpeed = playerTwo->GetY(frc::GenericHID::kLeftHand);
    }
    if (playerTwo->GetAButton()) {

        launcher.setIndexSpeed(m_launcherIndexSpeed);
    }
    else {

        launcher.setIndexSpeed(0);
    }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
