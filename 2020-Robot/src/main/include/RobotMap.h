//RobotMap: A collection of constant variables for declarations and settings.
//Significantly reduces the amount of time spent looking for and configuring.

#pragma once

#include <math.h>

/*_____RoboRIO PWM Pin Declarations_____*/
const int R_PWMPortClimberMotorClimb = 0;
const int R_PWMPortClimberMotorTranslate = 3;
const int R_PWMPortClimberMotorWheel = 2;
const int R_PWMPortClimberServoLock = 1;
/*___End RoboRIO PWM Pin Declarations___*/

/*_____RoboRIO DIO Pin Declarations_____*/
/*___End RoboRIO DIO Pin Declarations___*/

/*_____RoboRIO CAN Bus ID Declarations_____*/
const int R_CANIDzionFrontRightSwerve = 1;
const int R_CANIDzionFrontRightDrive  = 2;
const int R_CANIDzionFrontLeftSwerve  = 3;
const int R_CANIDzionFrontLeftDrive   = 4;
const int R_CANIDzionRearLeftDrive    = 5;
const int R_CANIDzionRearLeftSwerve   = 6;
const int R_CANIDzionRearRightDrive   = 7;
const int R_CANIDzionRearRightSwerve  = 8;

const int R_CANIDmotorIntake = 9;

const int R_CANIDmotorLauncherIndex  = 10;
const int R_CANIDmotorLauncherLaunch = 11;
/*___End RoboRIO CAN Bus ID Declarations___*/

/*_____Controller Settings_____*/
const int R_controllerPortPlayerOne = 0;
const int R_controllerPortPlayerTwo = 1;

//This deadzone is used to determine when the controller is completely motionless
const double R_deadzoneController = .1;
//And this one is to determine when rotation is being induced, as simply operation
//of the controller often results in errant rotation. Due to how easy it is to
//drift, it is significantly higher.
const double R_deadzoneControllerZ = .3;
// This deadzone is for the maximum allowable Limelight offset.
const double R_deadzoneLimelightX = 0.75;

//And this is the execution cap for how fast manual zeroing can occur.
const double R_executionCapControllerZero = .1;

//These are the playerTwo raw controller buttons that are used for manually
//zeroing Zion one wheel at a time by holding them down.
const int R_zeroButtonFR = 0;
const int R_zeroButtonFL = 0;
const int R_zeroButtonRL = 0;
const int R_zeroButtonRR = 0;
/*___End Controller Settings___*/

/*_____Global Robot Variable Settigns_____*/
//This is the highest decimal percentage of full speed that Zion can actually go.
const double R_executionCapZion = .75;
//This is at what rate the regular execution cap is scaled for precision driving.
const double R_executionCapZionPrecision = .25 * R_executionCapZion;
//This one is for running the intake motors.
const double R_executionCapIntake = .75;

//This is the default launcher index speed.
const double R_launcherDefaultSpeedIndex = 1;
//And this the default launcher launch speed.
const double R_launcherDefaultSpeedLaunch = .7425;

//This is the speed for automatic lateral movement in autonomous.
const double R_zionAutoMovementSpeedLateral = .4;
//And for rotational movement.
const double R_zionAutoMovementSpeedRotational = .4;
//This is how close to zero the Limelight's horizontal target offset can be
//in order to be considered centered.
const double R_zionAutoToleranceHorizontalOffset = .5;
//This is the tolerance for autonomous angle assumption in degrees.
//THIS IS NOT SKEW "NORMAL" ANGLE TOLERANCE - that and distance tolerance
//for auto are defined with the Arduino. This is used ONLY with the NavX.
const double R_zionAutoToleranceAngle = 10;

//The amount of REV rotations it takes for a swerve assembly to make a full rotation.
//Often, a REV Rotation is referred to as a Nic, although they mean different things.
//Truly, a Nic is ~17.976 REV Rotation values.
const double R_nicsConstant = 17.9761447906494;
//The change in encoder output per full wheel rotation around the axle.
//This value can be used to move a certain distance using solely encoder values.
const double R_kuhnsConstant = 8.3121115031;
//If an xy coordinate plane is centered at the middle of the drivetrain, this
//is the radian measure between the y-axis and the front right wheel. This is
//the basic unit of a non-moving center turn, and it is modified as the basis
//for moving and turning at the same time.
const double R_angleFromCenterToFrontLeftWheel = 45.;
const double R_angleFromCenterToFrontRightWheel = 315.;
const double R_angleFromCenterToRearLeftWheel = 135.;
const double R_angleFromCenterToRearRightWheel = 225.;

//These contants are used for the functions which provide assuming a position.
//See those functions for further detail.
const double R_swerveTrainAssumePositionTolerance = .1;
const double R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorAt = 3.5;
const double R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorSpeed = .2;
const double R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorAt = 1;
const double R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorSpeed = .02;
/*___End Global Robot Variable Settings___*/
