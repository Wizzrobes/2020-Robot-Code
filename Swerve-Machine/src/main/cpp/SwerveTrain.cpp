#include <frc/XboxController.h>

#include "SwerveTrain.h"

void SwerveTrain::driveController(frc::XboxController &controller) {

    //Get the current REV rotation position of the swerves
    const double currentFrPos = m_frontRight->getSwervePosition();
    //Get the current amount of rotations we need to set from center
    const double controllerREVRotationsFromCenter = getControllerREVRotationsFromCenter(controller.GetX(frc::GenericHID::kLeftHand), controller.GetY(frc::GenericHID::kLeftHand));
    //Get the current magnitude of the controller's joystick
    const double controllerMagnitude = getAbsoluteControllerMagnitude(controller.GetX(frc::GenericHID::kLeftHand), controller.GetY(frc::GenericHID::kLeftHand));

    //If the control stick is within deadzone, return to the zero position
    if (abs(controller.GetX(frc::GenericHID::kLeftHand)) < R_playerOneControllerDeadzone && abs(controller.GetY(frc::GenericHID::kLeftHand)) < R_playerOneControllerDeadzone) {

        assumeSwerveZeroPosition();
    }
}
