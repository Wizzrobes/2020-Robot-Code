//RobotMap: A collection of constant variables for declarations and settings.
//Significantly reduces the amount of time spent looking for and configuring.

#pragma once

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
/*___End RoboRIO CAN Bus ID Declarations___*/

/*_____PCM Pin Declarations_____*/
/*___End PCM Pin Declarations___*/

/*_____Global Robot Variable Settigns_____*/
const int R_playerOneControllerPort = 0;
const double R_controllerXYDeadzone = .075;
const double R_controllerXYTurningDeadzone = .6; 

const double R_swerveTrainAssumePositionTolerance = .1;
const double R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorAt = 3.5;
const double R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorSpeed = .2;
const double R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorAt = 1;
const double R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorSpeed = .02;

//The amount of REV rotations it takes for a swerve assembly to make a full rotation.
const double R_nicsConstant = 17.9761447906494;
/*___End Global Robot Variable Settings___*/
