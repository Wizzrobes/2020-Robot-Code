#include <frc/XboxController.h>

#include "SwerveTrain.h"

void SwerveTrain::driveController(frc::XboxController *controller) {

    const double controllerREVRotationsFromCenter = getControllerREVRotationsFromCenter(controller);
    const double controllerMagnitude = getControllerAbsoluteMagnitude(controller);

    //TODO: Why is there a negative here?
    const double controllerTurningMagnitude = -controller->GetX(frc::GenericHID::kRightHand);

    if (getControllerAllInDeadzone(controller)) {
        
        assumeNearestZeroPosition();
        setDriveSpeed(0);
    }
    else if (abs(controllerTurningMagnitude) > R_controllerDeadzone) {

        m_frontRight->assumeSwervePosition((1.0 / 8.0) * R_nicsConstant);
        m_frontLeft->assumeSwervePosition((3.0 / 8.0) * R_nicsConstant);
        m_rearLeft->assumeSwervePosition((5.0 / 8.0) * R_nicsConstant);
        m_rearRight->assumeSwervePosition((7.0 / 8.0) * R_nicsConstant);
        setDriveSpeed(controllerTurningMagnitude / 4.0);
    }

    else {
        
        m_frontRight->assumeSwervePosition(controllerREVRotationsFromCenter);
        m_frontLeft->assumeSwervePosition(controllerREVRotationsFromCenter);
        m_rearLeft->assumeSwervePosition(controllerREVRotationsFromCenter);
        m_rearRight->assumeSwervePosition(controllerREVRotationsFromCenter);
        setDriveSpeed(controllerMagnitude / 4.0);
    }
}
