#include <frc/Joystick.h>
#include <cameraserver/CameraServer.h>

#include "Robot.h"
#include "SwerveModule.h"
#include "SwerveTrain.h"
#include "RobotMap.h"

SwerveModule frontRightModule(R_frontRightDriveMotorCANID, R_frontRightSwerveMotorCANID);
SwerveModule frontLeftModule(R_frontLeftDriveMotorCANID, R_frontLeftSwerveMotorCANID);
SwerveModule rearLeftModule(R_rearLeftDriveMotorCANID, R_rearLeftSwerveMotorCANID);
SwerveModule rearRightModule(R_rearRightDriveMotorCANID, R_rearRightSwerveMotorCANID);
SwerveTrain zion(frontRightModule, frontLeftModule, rearLeftModule, rearRightModule);

//frc::XboxController *playerOne;
frc::Joystick *joystickPlayer; 

void Robot::RobotInit() {

    //playerOne = new frc::XboxController(R_playerOneControllerPort);
    joystickPlayer = new frc::Joystick(R_playerOneControllerPort); 

    frc::CameraServer::GetInstance()->StartAutomaticCapture();
}
void Robot::RobotPeriodic() {}
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {

  /*  if (playerOne->GetAButton()) {

        zion.setSwerveZeroPosition();
    }
    if (playerOne->GetXButton()) {

        zion.driveController(playerOne);
    }
    */ 
   if(joystickPlayer->GetRawButton(3)) {
       zion.setSwerveZeroPosition(); 
   } 
   if(joystickPlayer->GetRawButton(11)) {
       zion.driveController(joystickPlayer);
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
