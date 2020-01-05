#include <frc/XboxController.h>

#include "SwerveTrain.h"

void SwerveTrain::driveController(frc::XboxController &controller) {

    //Get the current REV rotation position of the swerves
    const double frPos = m_frontRight->getSwervePosition();
    const double flPos = m_frontLeft->getSwervePosition();
    const double rlPos = m_rearLeft->getSwervePosition();
    const double rrPos = m_rearRight->getSwervePosition();
    //Get the current amount of rotations we need to set from center
    const double rotateToPosition = getControllerRotationsFromCenter(controller.GetX(frc::GenericHID::kLeftHand), controller.GetY(frc::GenericHID::kLeftHand));
    //Get the current magnitude of the controller's joystick
    const double controllerMagnitude = getAbsoluteControllerMagnitude(controller.GetX(frc::GenericHID::kLeftHand), controller.GetY(frc::GenericHID::kLeftHand));

    if (!(m_frontRightSwerveZeroPosition + rotateToPosition < m_frontRightSwerveZeroPosition + R_nicsConstant)) {

        m_frontRightSwerveZeroPosition += R_nicsConstant;
        m_frontLeftSwerveZeroPosition += R_nicsConstant;
        m_rearLeftSwerveZeroPosition += R_nicsConstant;
        m_rearRightSwerveZeroPosition += R_nicsConstant;
        assumeSwerveZeroPosition();
    }
    else if (!(m_frontRightSwerveZeroPosition - rotateToPosition > m_frontRightSwerveZeroPosition - R_nicsConstant)) {

        m_frontRightSwerveZeroPosition -= R_nicsConstant;
        m_frontLeftSwerveZeroPosition -= R_nicsConstant;
        m_rearLeftSwerveZeroPosition -= R_nicsConstant;
        m_rearRightSwerveZeroPosition -= R_nicsConstant;
        assumeSwerveZeroPosition();
    }
    else {

        m_frontRight->assumeSwervePosition(frPos + rotateToPosition);
        m_frontLeft->assumeSwervePosition(flPos + rotateToPosition);
        m_rearLeft->assumeSwervePosition(rlPos + rotateToPosition);
        m_rearRight->assumeSwervePosition(rrPos + rotateToPosition);
    }

    setDriveSpeed(controllerMagnitude);
}
