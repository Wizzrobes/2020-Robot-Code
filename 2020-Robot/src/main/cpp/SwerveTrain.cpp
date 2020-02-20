#include <math.h>

#include <frc/Joystick.h>

#include "SwerveTrain.h"
#include "VectorDouble.h"

void SwerveTrain::driveController(frc::Joystick *controller) {

    //TODO: Why does inverting certain things work?
    double x = -controller->GetX();
    double y = -controller->GetY();
    double z = controller->GetZ();

    double angle = 0.;
    //To prevent controller drift, if the values of X, Y, and Z are inside of
    //deadzone, set them to 0
    forceControllerXYZToZeroInDeadzone(x, y, z);

    //To prevent accidental turning, optimize Z to X and Y's magnitude.
    optimizeZ(x, y, z); 

    /*
    The translation vector is the "standard" vector - that is, if no rotation
    were applied, the robot would simply travel in the direction of this
    vector. In order to obtain this, we need the X and Y from the controller
    in addition to input from a gyroscope. This is due to the fact that
    pushing straight on the controller should always make it drive directly
    away from the operator, and simply driving "straight" at a 45* angle
    would make it drive away from the operator at 45*. This is true of any
    angle, so the gyro is needed to offset the vector described by X and Y.
    VectorDouble translationVector(0, 0);
    */
    //TODO: why inverted?
    VectorDouble translationVector (-x, y);

    //This conditional stops the robot from being field oriented
    //TODO: Make it more usable and state why
    if ((abs(x) > 0 || abs(y) > 0) && abs(z) > 0) {

        angle = navX->getYawFull();
    }
    else {

        navX->resetYaw(); 
    }

    /*
    The rotational vectors are found by multiplying the controller's
    rotational axis [-1, 1] by the cosine of the wheel's RELATIVE yaw (the
    position we put the wheels in so that it can turn, with zero at the top)
    minus the number of degrees we are offset from 0.  Then, for the j value
    we do the same, except we use sine, for Y.  All angles passed as paramaters
    to cos() and sin() are converted to radians first.
    */
    VectorDouble frontRightRotationVector (

        z * cos((R_angleFromCenterToFrontRightWheel - angle) * (M_PI / 180)),
        z * sin((R_angleFromCenterToFrontRightWheel - angle) * (M_PI / 180))
    );

    VectorDouble frontLeftRotationVector (

        z * cos((R_angleFromCenterToFrontLeftWheel - angle) * (M_PI / 180)),
        z * sin((R_angleFromCenterToFrontLeftWheel - angle) * (M_PI / 180))
    );

    VectorDouble rearLeftRotationVector (

        z * cos((R_angleFromCenterToRearLeftWheel - angle) * (M_PI / 180)),
        z * sin((R_angleFromCenterToRearLeftWheel - angle) * (M_PI / 180))
    );

    VectorDouble rearRightRotationVector (

        z * cos((R_angleFromCenterToRearRightWheel - angle) * (M_PI / 180)),
        z * sin((R_angleFromCenterToRearRightWheel - angle) * (M_PI / 180))
    );

    /*
    And the vector we actually want to apply to the swerves is the sum of
    the two vectors - the vector that forms "straight" (translationVector)
    and the vector that forms strictly the rotation (rotationVector).
    */
    VectorDouble frontRightResultVector = translationVector + frontRightRotationVector;
    VectorDouble frontLeftResultVector = translationVector + frontLeftRotationVector;
    VectorDouble rearLeftResultVector = translationVector + rearLeftRotationVector;
    VectorDouble rearRightResultVector = translationVector + rearRightRotationVector;

    //If the controller is in the total deadzone (entirely still)...
    if (getControllerInDeadzone(controller)) {

        /*
        Go to the nearest zero position, take it as the new zero, and
        reset the angle coming off of the gyroscope to zero to allow
        setting of the next translation vector, as the translation
        vector is always refereced from "center" (the beginning of the
        turn) and updated based on how much that gyro angle changes.
        Due to this, it must be reset when not in movement to allow
        this behavior to occur...
        */
        assumeNearestZeroPosition();
        setDriveSpeed(0);
    }
    
    //Otherwise, go to the result vectors and use the magnitude to set the
    //speed of driving, and set each wheel's swerve position based on its
    //respective resulting vector.
    else {

        /*
        Here, all of the resulting vectors are
        converted into Nics so that they can be written to the swerve modules
        using assumeSwervePosition().  We get Nics from degrees by calling
        getSwerveRotatingPosition().
        */

        m_frontRight->assumeSwervePosition(m_frontRight->getStandardDegreeSwervePosition(frontRightResultVector, angle));
        m_frontLeft->assumeSwervePosition(m_frontLeft->getStandardDegreeSwervePosition(frontLeftResultVector, angle));
        m_rearLeft->assumeSwervePosition(m_rearLeft->getStandardDegreeSwervePosition(rearLeftResultVector, angle));
        m_rearRight->assumeSwervePosition(m_rearRight->getStandardDegreeSwervePosition(rearRightResultVector, angle));

        m_frontRight->setDriveSpeed(frontRightResultVector.magnitude() * R_zionExecutionCap);
        m_frontLeft->setDriveSpeed(frontLeftResultVector.magnitude() * R_zionExecutionCap);
        m_rearLeft->setDriveSpeed(rearLeftResultVector.magnitude() * R_zionExecutionCap);
        m_rearRight->setDriveSpeed(rearRightResultVector.magnitude() * R_zionExecutionCap);
    }
}

void SwerveTrain::zeroController(frc::Joystick *controller) {

    //This one is also built for being upside down, so invert it.
    const double controllerTurningMagnitude = -controller->GetZ();

    if (controller->GetRawButton(R_zeroButtonFR)) {

        m_frontRight->setSwerveSpeed(controllerTurningMagnitude * R_controllerZeroExecutionCap);
    }
    else if (controller->GetRawButton(R_zeroButtonFL)) {

        m_frontLeft->setSwerveSpeed(controllerTurningMagnitude * R_controllerZeroExecutionCap);
    }
    else if (controller->GetRawButton(R_zeroButtonRL)) {

        m_rearLeft->setSwerveSpeed(controllerTurningMagnitude * R_controllerZeroExecutionCap);
    }
    else if (controller->GetRawButton(R_zeroButtonRR)) {

        m_rearRight->setSwerveSpeed(controllerTurningMagnitude * R_controllerZeroExecutionCap);
    }
    else {

        setZeroPosition();
    }
}

double SwerveTrain::getClockwiseREVRotationsFromCenter(frc::Joystick *controller) {

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
    //If an imaginary situation is presented, set the angle equal to 0...
    if (magnitudeProduct == 0) {

       angleRad = 0;
    }

    //To go from a full 0pi to 2pi and overcome the limitation of arccos, jump
    //to 2pi and subtract the gradually decreasing angle...
    if (x < 0) {

        angleRad = (2 * M_PI) - angleRad;
    }
    //The decimal total of the whole circle is the radians over 2pi...
    double decimalTotalCircle = ((angleRad) / (2 * M_PI));
    //And the amount of REV rotations we want to rotate is the decimal total by Nic's Constant.
    return decimalTotalCircle * R_nicsConstant;
}
double SwerveTrain::getClockwiseREVRotationsFromCenter(const VectorDouble &vector) {

    const double x = vector.i;
    const double y = vector.j;
    VectorDouble center(0, 1);
    VectorDouble current(x, y);

    const double dotProduct = center * current;
    const double magnitudeProduct = center.magnitude() * current.magnitude();
    const double cosineAngle = dotProduct / magnitudeProduct;
    double angleRad = acos(cosineAngle);
    if (magnitudeProduct == 0) {

        angleRad = 0;
    }

    if (x < 0) {

        angleRad = (2 * M_PI) - angleRad;
    }
    double decimalTotalCircle = ((angleRad) / (2 * M_PI));
    return decimalTotalCircle * R_nicsConstant;
}
//TODO: Inline function documentation
double SwerveTrain::getStandardDegreeAngleFromCenter(const double &x, const double &y) {

    VectorDouble center(0, 1);
    VectorDouble current(x, y);

    const double dotProduct = center * current;
    const double magnitudeProduct = center.magnitude() * current.magnitude();
    const double cosineAngle = dotProduct / magnitudeProduct;
    double angleRad = acos(cosineAngle);
    if (magnitudeProduct == 0) {

        angleRad = 0;
    }

    if (x < 0) {

        angleRad = (2 * M_PI) - angleRad;
    }
    angleRad *= (180. / M_PI);
    return angleRad;
}
//TODO: Inline function documentation
//no longer needed in theory
VectorDouble SwerveTrain::getTranslationVector(const double &x, const double &y, double angleGyro) {

    //TODO: Why does inverting this as well solve our problems?
    double joystickAngle = getStandardDegreeAngleFromCenter(-x, y);

    double vectorAngle = 0;
    if (angleGyro < 0.) {

        angleGyro += 360.;
    }
    vectorAngle = 450. - joystickAngle + angleGyro;
    if (vectorAngle > 360.) {

       vectorAngle = fmod (vectorAngle, 360.);
    }
    //Make the conversion to radians to faciliate trigonometric usage
    vectorAngle *= (M_PI / 180.);

    //The absolute value of X and Y is taken because cosine and sine account
    //for signage - allowing X and Y signage causes double negative errors.
    VectorDouble translationVector(abs(x) * cos(vectorAngle), abs(y) * sin(vectorAngle));
    return translationVector;
}

void SwerveTrain::lineupToTarget(const double &leftDistToWall, const double &rightDistToWall, const double &targetOffset, const double &targetDistance) {

    double rotationDirection = 0;
    double averageDistanceFromWall = (leftDistToWall + rightDistToWall) / 2;
    //Dependent on units the sensor outputs.
    double shotDistance = 144;

    //Vectors for lateral movement in certain directions.
    VectorDouble leftMovementVector(-1, 0);
    VectorDouble rightMovementVector(1, 0);
    VectorDouble forwardMovementVector(0, 1);
    VectorDouble backwardsMovementVector(0, -1);
    
    //Empty vectors for the wheels themselves.
    VectorDouble frontRightVector(0, 0);
    VectorDouble frontLeftVector(0, 0);
    VectorDouble rearLeftVector(0, 0);
    VectorDouble rearRightVector(0, 0);
    
    //if and else if statements make sure the program runs in proper order-
    //rotation to be square with the wall, left-to-right lateral motion to be
    //centered with the target, and then front-to-back motion to set the proper
    //distance from the target.
    //If Zion is not plumb with the wall:
    if (abs(leftDistToWall - rightDistToWall) > 0/*TOLERANCE*/) {

        //Sets the rotation direction to 1 if right distance is less than the left,
        //to -1 if left distance is less than the right, and keeps 0 if equal, as this
        //indicates the direction in which rotation must commence to be plumb
        //with the wall.
        if (leftDistToWall > rightDistToWall) {

            rotationDirection = 1;
        }
        else if (rightDistToWall > leftDistToWall) {

            rotationDirection = -1;
        }

        //TODO: Better documentation than I can write :)
        const double squareOfTwo = (sqrt(2) / 2);
        frontRightVector.i = rotationDirection * squareOfTwo;
        frontRightVector.j = rotationDirection * -squareOfTwo;
        frontLeftVector.i = rotationDirection * squareOfTwo;
        frontLeftVector.j = rotationDirection * squareOfTwo;
        rearLeftVector.i = rotationDirection * -squareOfTwo;
        rearLeftVector.j = rotationDirection * squareOfTwo;
        rearRightVector.i = rotationDirection * -squareOfTwo;
        rearRightVector.j = rotationDirection * -squareOfTwo;
    }
    //Next, if we're plumb with the wall, but not centered:
    else if (abs(targetOffset) < 0/*TOLERANCE*/) {

        //Move left or right based on the x-offset.
        if (targetOffset > 0) {

            frontRightVector = rightMovementVector;
            frontLeftVector = rightMovementVector;
            rearLeftVector = rightMovementVector;
            rearRightVector = rightMovementVector;
        }
        if (targetOffset < 0) {

            frontRightVector = leftMovementVector;
            frontLeftVector = leftMovementVector;
            rearLeftVector = leftMovementVector;
            rearRightVector = leftMovementVector;
        }
    }
    //Finally, if we're plumb and centered but off of target distance:
    else if (abs(averageDistanceFromWall - targetDistance) > 0/*TOLERANCE*/) {

        if (averageDistanceFromWall > shotDistance) {

            frontRightVector = forwardMovementVector;
            frontLeftVector = forwardMovementVector;
            rearLeftVector = forwardMovementVector;
            rearRightVector = forwardMovementVector;
        }
        if (averageDistanceFromWall < shotDistance) {

            frontRightVector = backwardsMovementVector;
            frontLeftVector = backwardsMovementVector;
            rearLeftVector = backwardsMovementVector;
            rearRightVector = backwardsMovementVector;
        }
    }
    //Finally, write the calculated values to their respective motors.
    m_frontRight->assumeSwervePosition(getClockwiseREVRotationsFromCenter(frontRightVector));
    m_frontLeft->assumeSwervePosition(getClockwiseREVRotationsFromCenter(frontLeftVector));
    m_rearLeft->assumeSwervePosition(getClockwiseREVRotationsFromCenter(rearLeftVector));
    m_rearRight->assumeSwervePosition(getClockwiseREVRotationsFromCenter(rearRightVector));
    setDriveSpeed(R_zionAutoLateralMovementSpeed);
}


