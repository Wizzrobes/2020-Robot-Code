#include <math.h>
#include <cmath>

#include <frc/Joystick.h>

#include "SwerveTrain.h"
#include "VectorDouble.h"

void SwerveTrain::driveController(frc::Joystick *controller) {

    //All value are inverted as the functions' logic is written for an upside-down Zion
    double x = -controller->GetX();
    double y = -controller->GetY();
    double z = -controller->GetZ(); 

    //To prevent controller drift, if the values of X, Y, and Z are inside of
    //deadzone, set them to 0
    forceControllerXYZToZeroInDeadzone(x, y, z); 

    VectorDouble translationVector = getTranslationVector(x, y, navX->getYaw());

    //The rotation vectors' i-components take the cosine of the R_ angle (see
    //RobotMap) in order to discern the first component of a vector which
    //points in the direction that would be applied to the swerve if the
    //rotation were only around center. This is multipled by the magnitude of Z
    //in order to make it a proportionally smaller component to
    //result in a less drastic turn, as a Z-value of 1 would imply the most
    //drastic turn possible. The j-component does the same, but using sine to
    //find the other half of the hypotenuse's components which make up the
    //direction that will be applied in rotation, as cosine only provides the
    //x-component. Cosine for X, sine for Y. If none of the Z-values were
    //negative, each vector applied to the swerves would point the same
    //direction - so axis inversion is used to ensure that traversing the
    //swerves from frontRight clockwise compounds 90 to the rotation vector.
    VectorDouble frontRightRotationVector(z * cos(R_angleFromCenterToFrontRightWheel), -z * sin(R_angleFromCenterToFrontRightWheel)); 
    VectorDouble frontLeftRotationVector(z * cos(R_angleFromCenterToFrontRightWheel), z * sin(R_angleFromCenterToFrontRightWheel)); 
    VectorDouble rearLeftRotationVector(-z * cos(R_angleFromCenterToFrontRightWheel), z * sin (R_angleFromCenterToFrontRightWheel)); 
    VectorDouble rearRightRotationVector(-z * cos(R_angleFromCenterToFrontRightWheel), -z * sin (R_angleFromCenterToFrontRightWheel));
    
    //And the vector we actually want to apply to the swerves is the sum of
    //the two vectors - the vector that forms "straight" (translationVector)
    //and the vector that forms strictly the rotation (rotationVector).
    VectorDouble frontRightResultVector = translationVector + frontRightRotationVector; 
    VectorDouble frontLeftResultVector = translationVector + frontLeftRotationVector;
    VectorDouble rearLeftResultVector = translationVector + rearLeftRotationVector;
    VectorDouble rearRightResultVector = translationVector + rearRightRotationVector;

    //If the controller is in the total deadzone (entirely still)...
    if (getControllerInDeadzone(controller)) {

        //Go to the nearest zero position, take it as the new zero, and
        //reset the angle coming off of the gyroscope to zero to allow
        //setting of the next translation vector, as the translation
        //vector is always refereced from "center" (the beginning of the
        //turn) and updated based on how much that gyro angle changes.
        //Due to this, it must be reset when not in movement to allow
        //this behavior to occur...
        assumeNearestZeroPosition();
        setSwerveZeroPosition();
        navX->resetYaw();
    }
    //Otherwise, go to the result vectors and use the magnitude to set the
    //speed of driving.
    else {

        m_frontRight->assumeSwervePosition(getVectorClockwiseREVRotationsFromCenter(frontRightResultVector));
        m_frontLeft->assumeSwervePosition(getVectorClockwiseREVRotationsFromCenter(frontLeftResultVector));
        m_rearLeft->assumeSwervePosition(getVectorClockwiseREVRotationsFromCenter(rearLeftResultVector));
        m_rearRight->assumeSwervePosition(getVectorClockwiseREVRotationsFromCenter(rearRightResultVector));
        setDriveSpeed(frontRightResultVector.magnitude() * R_zionExecutionCap);
    }
}

double SwerveTrain::getControllerClockwiseREVRotationsFromCenter(frc::Joystick *controller) {

    //Invert both the x and y once again, as the logic is written for an
    //upside-down Zion...
    const double x = -controller->GetX(frc::GenericHID::kLeftHand);
    const double y = -controller->GetY(frc::GenericHID::kLeftHand);

    //Create vectors for the line x = 0 and the line formed by the joystick coordinates...
    VectorDouble center(0, 1);
    VectorDouble current(x, y);
    //Get the dot produt of the vectors for use in calculation...
    const double dotProduct = center * current;
    //Multiply each vector's magnitude together for use in calculation...
    const double magnitudeProduct = center.magnitude() * current.magnitude();
    //The cosine of the angle we want in rad is the dot product over the magnitude product...
    const double cosineAngle = dotProduct / magnitudeProduct;
    //The angle we want is the arccosine of its cosine...
    double angleRad = acos(cosineAngle);
    //To go from a full 0pi to 2pi and overcome the limitation of arccos, jump to 2pi and subtract the gradually decreasing angle...
    if (x < 0) {

        angleRad = (2 * M_PI) - angleRad;
    }
    //The decimal total of the whole circle is the radians over 2pi...
    double decimalTotalCircle = ((angleRad) / (2 * M_PI));
    //And the amount of REV rotations we want to rotate is the decimal total by Nic's Constant.
    return decimalTotalCircle * R_nicsConstant;
}
double SwerveTrain::getVectorClockwiseREVRotationsFromCenter(const VectorDouble &vector) {

    const double x = -vector.i; 
    const double y = -vector.j;

    VectorDouble center(0, 1);
    VectorDouble current(x, y);
    const double dotProduct = center * current;
    const double magnitudeProduct = center.magnitude() * current.magnitude();
    const double cosineAngle = dotProduct / magnitudeProduct;
    double angleRad = acos(cosineAngle);
    if (x < 0) {

        angleRad = (2 * M_PI) - angleRad;
    }
    double decimalTotalCircle = ((angleRad) / (2 * M_PI));
    return decimalTotalCircle * R_nicsConstant;
}
double SwerveTrain::getControllerAngleFromCenter(frc::Joystick *controller) {

    const double x = -controller->GetX();
    const double y = -controller->GetY();

    VectorDouble center(0, 1);
    VectorDouble current(x, y);
    const double dotProduct = center * current;
    const double magnitudeProduct = center.magnitude() * current.magnitude();
    const double cosineAngle = dotProduct / magnitudeProduct;
    return acos(cosineAngle);
}
