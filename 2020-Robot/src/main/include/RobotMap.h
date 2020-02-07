//RobotMap: A collection of constant variables for declarations and settings.
//Significantly reduces the amount of time spent looking for and configuring.

#pragma once

#include <math.h>

/*_____RoboRIO PWM Pin Declarations_____*/
/*___End RoboRIO PWM Pin Declarations___*/

/*_____RoboRIO DIO Pin Declarations_____*/
/*___End RoboRIO DIO Pin Declarations___*/

/*_____RoboRIO CAN Bus ID Declarations_____*/
const int R_frontRightSwerveMotorCANID = 1;
const int R_frontRightDriveMotorCANID  = 2;
const int R_frontLeftSwerveMotorCANID  = 3;
const int R_frontLeftDriveMotorCANID   = 4;
const int R_rearLeftSwerveMotorCANID   = 5;
const int R_rearLeftDriveMotorCANID    = 6;
const int R_rearRightSwerveMotorCANID  = 7;
const int R_rearRightDriveMotorCANID   = 8;

//TODO: Find actual CANID
const int R_intakeMotorCANID = 0;

const int R_launcherIndexMotorCANID = 9;
const int R_launcherLaunchMotorCANID = 10;
/*___End RoboRIO CAN Bus ID Declarations___*/

/*_____PCM Pin Declarations_____*/
/*___End PCM Pin Declarations___*/

/*_____Controller Settings_____*/
const int R_playerOneControllerPort = 0;
const int R_playerTwoControllerPort = 1;

//This deadzone is used to determine when the controller is completely motionless
const double R_controllerDeadzone = .075;
//And this one is to determine when rotation is being induced, as simply operation
//of the controller often results in errant rotation. Due to how easy it is to
//drift, it is significantly higher.
const double R_controllerZDeadzone = .3;
// This deadzone is for the horizontal offset of the limelight because Zion does not
// have to be perfectly on center with the target. In degrees.
const double R_limelightXDeadzone = 0.75;

//These are the playerTwo raw controller buttons that are used for manually
//zeroing Zion one wheel at a time by holding them down.
const int R_zeroButtonFR = 0;
const int R_zeroButtonFL = 0;
const int R_zeroButtonRL = 0;
const int R_zeroButtonRR = 0;
/*___End Controller Settings___*/

/*_____Global Robot Variable Settigns_____*/
//This is the highest decimal percentage of full speed that Zion can actually go.
const double R_zionExecutionCap = .25;
//And this is the execution cap for how fast manual zeroing can occur.
const double R_controllerZeroExecutionCap = .1;
//This is the speed for automatic lateral movement in autonomous.
const double R_zionAutoLateralMovementSpeed = .4;

//The amount of REV rotations it takes for a swerve assembly to make a full rotation.
//Often, a REV Rotation is referred to as a Nic, although they mean different things.
//Truly, a Nic is ~17.976 REV Rotation values.
const double R_nicsConstant = 17.9761447906494;
//If an xy coordinate plane is centered at the middle of the drivetrain, this
//is the radian measure between the y-axis and the front right wheel. This is
//the basic unit of a non-moving center turn, and it is modified as the basis
//for moving and turning at the same time.
const double R_angleFromCenterToFrontRightWheel = (M_PI / 4.0);

//These contants are used for the functions which provide assuming a position.
//See those functions for further detail.
const double R_swerveTrainAssumePositionTolerance = .1;
const double R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorAt = 3.5;
const double R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorSpeed = .2;
const double R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorAt = 1;
const double R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorSpeed = .02;
/*___End Global Robot Variable Settings___*/
