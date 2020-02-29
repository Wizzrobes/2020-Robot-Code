#include <cameraserver/CameraServer.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>

#include "Arduino.h"
#include "Climber.h"
#include "Hal.h"
#include "Intake.h"
#include "Launcher.h"
#include "Limelight.h"
#include "NavX.h"
#include "Robot.h"
#include "RobotMap.h"
#include "SwerveModule.h"
#include "SwerveTrain.h"

Arduino arduino;
Climber climber(R_PWMPortClimberMotorClimb, R_PWMPortClimberMotorTranslate, R_PWMPortClimberMotorWheel, R_PWMPortClimberServoLock);
Intake intake(R_CANIDmotorIntake);
Launcher launcher(R_CANIDmotorLauncherIndex, R_CANIDmotorLauncherLaunch);
Limelight limelight;
NavX navX(NavX::ConnectionType::kMXP);
SwerveModule frontRightModule(R_CANIDzionFrontRightDrive, R_CANIDzionFrontRightSwerve);
SwerveModule frontLeftModule(R_CANIDzionFrontLeftDrive, R_CANIDzionFrontLeftSwerve);
SwerveModule rearLeftModule(R_CANIDzionRearLeftDrive, R_CANIDzionRearLeftSwerve);
SwerveModule rearRightModule(R_CANIDzionRearRightDrive, R_CANIDzionRearRightSwerve);
SwerveTrain zion(frontRightModule, frontLeftModule, rearLeftModule, rearRightModule, navX);

Hal Hal9000(arduino, intake, launcher, limelight, navX, zion);

frc::Joystick *playerOne;
frc::XboxController *playerTwo;

void Robot::RobotInit() {

    playerOne = new frc::Joystick(R_controllerPortPlayerOne);
    playerTwo = new frc::XboxController(R_controllerPortPlayerTwo);

    frc::CameraServer::GetInstance()->StartAutomaticCapture();

    frc::SmartDashboard::PutNumber("Launcher::Speed-Index:", R_launcherDefaultSpeedIndex);
    frc::SmartDashboard::PutNumber("Launcher::Speed-Launch:", R_launcherDefaultSpeedLaunch);

    zion.setSwerveBrake(false);
}
void Robot::RobotPeriodic() {}
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {

    zion.setSwerveBrake(true);
}
void Robot::TeleopPeriodic() {

    if (playerOne->GetRawButtonPressed(3)) {

        zion.setZeroPosition();
    }
    if (playerOne->GetRawButton(1)) {

        navX.resetYaw();
    }
    
    if(playerOne->GetRawButton(12)) {

        zion.driveControllerPrecision(playerOne); 
    } else {

        zion.driveController(playerOne);
    }


    if (playerTwo->GetBackButton()) {

        double climbSpeed = -playerTwo->GetTriggerAxis(frc::GenericHID::kLeftHand) + playerTwo->GetTriggerAxis(frc::GenericHID::kRightHand);
        double translateSpeed = playerTwo->GetX(frc::GenericHID::kLeftHand);
        double wheelSpeed = playerTwo->GetX(frc::GenericHID::kRightHand);

        climber.setSpeed(Climber::Motor::kClimb, climbSpeed);
        climber.setSpeed(Climber::Motor::kTranslate, translateSpeed);
        climber.setSpeed(Climber::Motor::kWheel, wheelSpeed);
        climber.lock(false);
    }
    else {

        climber.setSpeed(Climber::Motor::kClimb);
        climber.setSpeed(Climber::Motor::kTranslate);
        climber.setSpeed(Climber::Motor::kWheel);
        climber.lock();
    }

    if (!playerTwo->GetBackButton()) {

        intake.setSpeed(-playerTwo->GetTriggerAxis(frc::GenericHID::kLeftHand) * R_executionCapIntake + playerTwo->GetTriggerAxis(frc::GenericHID::kRightHand) * R_executionCapIntake);
    }
    else {

        intake.setSpeed();
    }

    if (playerTwo->GetXButton()) {

        frc::SmartDashboard::PutNumber("Launcher::Speed-Index:", playerTwo->GetY(frc::GenericHID::kLeftHand));
    }
    if (playerTwo->GetYButton()) {

        frc::SmartDashboard::PutNumber("Launcher::Speed-Launch:", playerTwo->GetY(frc::GenericHID::kLeftHand));
    }
    if (playerTwo->GetAButton()) {

        launcher.setIndexSpeed(frc::SmartDashboard::GetNumber("Launcher::Speed-Index:", 0));
    }
    else {

        launcher.setIndexSpeed(0);
    }
    if (playerTwo->GetBButton()) {

        launcher.setLaunchSpeed(frc::SmartDashboard::GetNumber("Launcher::Speed-Launch:", 0));
    }
    else {

        launcher.setLaunchSpeed(0);
    }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
