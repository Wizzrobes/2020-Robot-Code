#include <math.h>

#include "SwerveModule.h"

void SwerveModule::assumeSwervePosition(const double &positionToAssume) {

    double currentPosition = getSwervePositionSingleRotation();

    //If the current position is close enough to where we want to go (within one tolerance value)...
    if (abs(positionToAssume - currentPosition) < R_swerveTrainAssumePositionTolerance) {

        //Stop rotating the swerve motor and skip checking anything else...
        m_swerveMotor->Set(0);
    }
    //If the position to assume is greater than half a revolution in the clockwise direction...
    else if (abs(positionToAssume - currentPosition) > R_nicsConstant / 2) {

        //If such a rotation needs to be clockwise...
        if (positionToAssume < currentPosition) {

            //Set the speed of the motor using the Nic's Constant distance between the two points...
            m_swerveMotor->Set(calculateAssumePositionSpeed(R_nicsConstant - (currentPosition - positionToAssume)));
        }
        //If such a rotation needs to be counterclockwise...
        else if (positionToAssume > currentPosition) {

            //Set the speed similarly, but negatively...
            m_swerveMotor->Set(calculateAssumePositionSpeed(-R_nicsConstant + (positionToAssume - currentPosition)));
        }
    }
    else {

        //Otherwise, perform a normal between two points rotation with a Nic's Constant value.
        m_swerveMotor->Set(calculateAssumePositionSpeed(positionToAssume - currentPosition));
    }
}

double SwerveModule::calculateAssumePositionSpeed(const double& howFarRemainingInTravel) {

    //Begin initally with a double calculated with the simplex function...
    double toReturn = ((1) / (1 + exp((-1 * abs(howFarRemainingInTravel)) + 5)));
    //If we satisfy conditions for the first linear piecewise, take that speed instead...
    if (abs(howFarRemainingInTravel) < R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorAt) {

        toReturn = R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorSpeed;
    }
    //Do the same for the second...
    if (abs(howFarRemainingInTravel) < R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorAt) {

        toReturn = R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorSpeed;
    }
    //And if we needed to travel negatively to get where we need to be, make the final speed negative...
    if (howFarRemainingInTravel < 0) {

        toReturn = -toReturn;
    }
    return toReturn;
}