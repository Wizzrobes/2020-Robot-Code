#include <cameraserver/CameraServer.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
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
Climber climber(R_PWMPortClimberMotorClimb, R_PWMPortClimberMotorTranslate, R_PWMPortClimberMotorWheel, R_PWMPortClimberServoLock, R_DIOPortSwitchClimberBottom);
frc::DigitalInput switchSwerveUnlock(R_DIOPortSwitchSwerveUnlock);
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

    m_booleanClimberLock = true;
    m_speedClimberClimb     = 0;
    m_speedClimberTranslate = 0;
    m_speedClimberWheel     = 0;
    m_speedIntake           = 0;
    m_speedLauncherIndex    = 0;
    m_speedLauncherLaunch   = 0;
}
void Robot::RobotPeriodic() {

    //Whenever Zion is on, if the unlock swerve button is pressed and held,
    //unlock the swerves for zeroing. Once released, lock them again. The
    //switch is inverted by default.
    zion.setSwerveBrake(switchSwerveUnlock.Get());
}
void Robot::AutonomousInit() {

    //Set the zero position before beginning auto, as it should have been
    //calibrated before the match. This persists for the match duration unless
    //overriden.
    zion.setZeroPosition();
}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {

    if (playerOne->GetRawButtonPressed(3)) {

        zion.setZeroPosition();
    }
    if (playerOne->GetRawButton(1)) {

        navX.resetYaw();
    }
    if (playerOne->GetRawButton(12)) {

        zion.driveControllerPrecision(playerOne); 
    }
    else {

        zion.driveController(playerOne);
    }

    //The second controller works in control layers on top of the basic
    //driving mode engaged with function buttons. If one of the functions
    //running under a button loses its button press, it will be overriden
    //by the regular mode. Useful for cancellation. Layers override each other.

    //The back button is "manual override" control layer. No auto, simply
    //writes unupdated values directly to motors, unlocking the climber,
    //with no execution caps or impediments. Overrides all other layers.
    if (playerTwo->GetBackButton()) {

        m_booleanClimberLock = false;
        m_speedClimberClimb = -playerTwo->GetTriggerAxis(frc::GenericHID::kLeftHand) + playerTwo->GetTriggerAxis(frc::GenericHID::kRightHand);
        m_speedClimberTranslate = playerTwo->GetX(frc::GenericHID::kLeftHand);
        m_speedClimberWheel = playerTwo->GetX(frc::GenericHID::kRightHand);
        m_speedIntake = -playerTwo->GetTriggerAxis(frc::GenericHID::kLeftHand) + playerTwo->GetTriggerAxis(frc::GenericHID::kRightHand);
        m_speedLauncherIndex = playerTwo->GetY(frc::GenericHID::kLeftHand);
        m_speedLauncherLaunch = playerTwo->GetY(frc::GenericHID::kRightHand);
    }
    else {

        m_booleanClimberLock = true;
        m_speedClimberClimb     = 0;
        m_speedClimberTranslate = 0;
        m_speedClimberWheel     = 0;
        m_speedIntake           = 0;
        m_speedLauncherIndex    = 0;
        m_speedLauncherLaunch   = 0;
    }

    //The start button is "climber" control layer. Controls nothing but the
    //climber. Overrides the auto layer.
    if (!playerTwo->GetBackButton() && playerTwo->GetStartButton()) {

        m_speedClimberClimb = -playerTwo->GetTriggerAxis(frc::GenericHID::kLeftHand) + playerTwo->GetTriggerAxis(frc::GenericHID::kRightHand);
        m_speedClimberTranslate = playerTwo->GetX(frc::GenericHID::kLeftHand);
        m_speedClimberWheel = playerTwo->GetX(frc::GenericHID::kRightHand);
        m_booleanClimberLock = !playerTwo->GetBumper(frc::GenericHID::kRightHand);
    }
    else {

        m_speedClimberClimb = 0;
        m_speedClimberTranslate = 0;
        m_speedClimberWheel = 0;
        m_booleanClimberLock = true;
    }

    //The center button is the "auto" control layer. Enables auto functions.
    //Overrides regular driving, but is overriden by all other layers.
    if (!playerTwo->GetBackButton() && !playerTwo->GetStartButton() && playerTwo->GetRawButton(9)) {}
    else {}

    //If no layers were engaged, regular driving can begin.
    if (!playerTwo->GetBackButton() && !playerTwo->GetStartButton() && !playerTwo->GetRawButton(9)) {

        m_speedIntake = (-playerTwo->GetTriggerAxis(frc::GenericHID::kLeftHand) + playerTwo->GetTriggerAxis(frc::GenericHID::kRightHand)) * R_executionCapIntake;

        if (playerTwo->GetXButton()) {

            frc::SmartDashboard::PutNumber("Launcher::Speed-Index:", playerTwo->GetY(frc::GenericHID::kLeftHand));
        }
        if (playerTwo->GetYButton()) {

            frc::SmartDashboard::PutNumber("Launcher::Speed-Launch:", playerTwo->GetY(frc::GenericHID::kLeftHand));
        }
        if (playerTwo->GetAButton()) {

            m_speedLauncherIndex = frc::SmartDashboard::GetNumber("Launcher::Speed-Index:", R_launcherDefaultSpeedIndex);
        }
        else {

            m_speedLauncherIndex = 0;
        }
        if (playerTwo->GetBButton()) {

            m_speedLauncherLaunch = frc::SmartDashboard::GetNumber("Launcher::Speed-Launch:", R_launcherDefaultSpeedLaunch);
        }
        else {

            m_speedLauncherLaunch = 0;
        }
    }

    //Once all layers have been evaluated, write out all of their values.
    climber.lock(m_booleanClimberLock);
    climber.setSpeed(Climber::Motor::kClimb, m_speedClimberClimb);
    climber.setSpeed(Climber::Motor::kTranslate, m_speedClimberTranslate);
    climber.setSpeed(Climber::Motor::kWheel, m_speedClimberWheel);
    intake.setSpeed(m_speedIntake);
    launcher.setIndexSpeed(m_speedLauncherIndex);
    launcher.setLaunchSpeed(m_speedLauncherLaunch);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
