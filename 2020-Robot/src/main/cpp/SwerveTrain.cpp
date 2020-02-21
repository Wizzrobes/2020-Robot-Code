#include <math.h>

#include <frc/Joystick.h>

#include "SwerveTrain.h"
#include "VectorDouble.h"
#include "Launcher.h"
#include "Limelight.h"

void SwerveTrain::driveController(frc::Joystick *controller) {

    //All value are inverted as the functions' logic is written for an upside-down Zion
    //TODO: Why does uninverting X solve our issues?
    double x = controller->GetX();
    double y = -controller->GetY();
    double z = -controller->GetZ();
    //To prevent controller drift, if the values of X, Y, and Z are inside of
    //deadzone, set them to 0
    forceControllerXYZToZeroInDeadzone(x, y, z);

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
    VectorDouble translationVector = getTranslationVector(x, y, 0.0);

    /*
    The rotation vectors' i-components take the cosine of the R_ angle (see
    RobotMap) in order to discern the first component of a vector which
    points in the direction that would be applied to the swerve if the
    rotation were only around center. This is multipled by the magnitude of Z
    in order to make it a proportionally smaller component to
    result in a less drastic turn, as a Z-value of 1 would imply the most
    drastic turn possible. The j-component does the same, but using sine to
    find the other half of the hypotenuse's components which make up the
    direction that will be applied in rotation, as cosine only provides the
    x-component. Cosine for X, sine for Y. If none of the Z-values were
    negative, each vector applied to the swerves would point the same
    direction - so axis inversion is used to ensure that traversing the
    swerves from frontRight clockwise compounds 90 to the rotation vector.
    */
    VectorDouble frontRightRotationVector(z * cos(R_angleFromCenterToFrontRightWheel), z * sin(R_angleFromCenterToFrontRightWheel));
    VectorDouble frontLeftRotationVector(z * cos(R_angleFromCenterToFrontRightWheel), -z * sin(R_angleFromCenterToFrontRightWheel));
    VectorDouble rearLeftRotationVector(-z * cos(R_angleFromCenterToFrontRightWheel), -z * sin(R_angleFromCenterToFrontRightWheel));
    VectorDouble rearRightRotationVector(-z * cos(R_angleFromCenterToFrontRightWheel), z * sin(R_angleFromCenterToFrontRightWheel));

    /*
    And the vector we actually want to apply to the swerves is the sum of
    the two vectors - the vector that forms "straight" (translationVector)
    and the vector that forms strictly the rotation (rotationVector).
    */
    VectorDouble frontRightResultVector = translationVector + frontRightRotationVector;
    VectorDouble frontLeftResultVector = translationVector + frontLeftRotationVector;
    VectorDouble rearLeftResultVector = translationVector + rearLeftRotationVector;
    VectorDouble rearRightResultVector = translationVector + rearRightRotationVector;

    /*
    Due to the way that the vector math is completed, it is currently
    possible for a vector's magnitude to evaluate to a value of greater
    than 1. As this is in error to send to a speed controller, the following
    calculations ensure that the magnitude of a vector will always evaluate
    to exactly one in the event that it was over one. The rest of the motor
    values then scale proportionally to this value, using the second block
    of code, which prevents non-standard turning from occurring through
    changing motor ratios not by a standard value.
    */
    double largestMagnitude = getLargestMagnitudeValue(frontRightResultVector.magnitude(), frontLeftResultVector.magnitude(), rearLeftResultVector.magnitude(), rearRightResultVector.magnitude());
    if (largestMagnitude > 1) {

        frontRightResultVector.i /= largestMagnitude;
        frontRightResultVector.j /= largestMagnitude;

        frontLeftResultVector.i /= largestMagnitude;
        frontLeftResultVector.j /= largestMagnitude;

        rearLeftResultVector.i /= largestMagnitude;
        rearLeftResultVector.j /= largestMagnitude;

        rearRightResultVector.i /= largestMagnitude;
        rearRightResultVector.j /= largestMagnitude;
    }

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
        navX->resetAll();
    }
    //Otherwise, go to the result vectors and use the magnitude to set the
    //speed of driving.
    else {

        m_frontRight->assumeSwervePosition(getClockwiseREVRotationsFromCenter(frontRightResultVector));
        m_frontLeft->assumeSwervePosition(getClockwiseREVRotationsFromCenter(frontLeftResultVector));
        m_rearLeft->assumeSwervePosition(getClockwiseREVRotationsFromCenter(rearLeftResultVector));
        m_rearRight->assumeSwervePosition(getClockwiseREVRotationsFromCenter(rearRightResultVector));

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


// Autonomous Code

// Backs up and moves the robot to lineup with the target side to side.
// Needs motor encoder values to move a specified distance.
Limelight limelight; 
void SwerveTrain::moveToTarget() {

    // The original value we started with. Uses front right wheel.
    double startingEncodeValue = m_frontRight->getDrivePosition();
    
    frc::SmartDashboard::PutNumber("TX", limelight.getHorizontalOffset());

    // Distances to move. Will change based on match position.
    // How far to back up, in inches.
    double backupDistance = 10;
    // Side movement distance, also in inches. Might be eliminated with.
    //double sideMovementDist = 120;

    double wheelCircumference = 4 * M_PI;

    // The goal motor encoder value for backwards movement.
    double backGoalEncoderValue = startingEncodeValue + ((backupDistance / wheelCircumference) * R_kuhnsConstant);
    
    // Vectors for movement in directions.
    VectorDouble leftVector(1, 0);
    VectorDouble backVector(0, -1);
    VectorDouble rightVector(-1, 0);
    VectorDouble forwardsVector(0, 1);

    //First backs Zion up an above specified distance for optimal shooting.
    while ((backGoalEncoderValue - m_frontRight->getDrivePosition()) > 0) {
        driveAutonomous(leftVector, R_zionAutoExecutionCap);
    }
    stopDriving();

    // Use of encoder values may be eliminated with limelight usage.
    //double secondEncodeValue = m_frontRight->getDrivePosition();
    // Motor encoder value we need to get to for side movement.
    //double secondGoalEncValue = secondEncodeValue + ((sideMovementDist / wheelCircumference) * R_kuhnsConstant);


    // TODO: Fix when can be called properly.
    // Moves Zion left if target is not acquired. Unlikely to be too far right.
    while (!limelight.getTarget()) {
        driveAutonomous(backVector, R_zionAutoExecutionCap);
    }

    // Movement if Zion is left of target...
    while (limelight.getTarget() && limelight.getHorizontalOffset() < -0.5) {
        frc::SmartDashboard::PutNumber("TX", limelight.getHorizontalOffset());
        driveAutonomous(backVector, R_zionAutoExecutionCap);
    }
    stopDriving();

    // ... and movement if Zion is right of target.
    while (limelight.getTarget() && limelight.getHorizontalOffset() > 0.5) {
         frc::SmartDashboard::PutNumber("TX", limelight.getHorizontalOffset());
        driveAutonomous(forwardsVector, R_zionAutoExecutionCap);
    }
    stopDriving();
}


// Turns and moves forwards/backwards for shot.
void SwerveTrain::lineupShot() {
    // Placeholder for sonar values.
    double sonar1Get = 144;
    double sonar2Get = 144;

    // Deadzone for horizontal offset of crosshair and target. In degrees.
    double angleDeadzone = 1;

    // Movement vectors for linear motion.
    // Remember: relative to intake, not shooter.
    VectorDouble forwardsVector(0,-1);
    VectorDouble backwardsVector(0,1);
    VectorDouble rightVector(-1,0);
    VectorDouble leftVector(1,0);

    // Distance we want to be at from wall. Will change based on units sensors report.
    double optimalDistance = 144;

    // Turning Zion to be parallel to the wall.
    while (abs((sonar1Get - sonar2Get) > 2)) { //2 is meant to represent the deadzone in centimeters 

        if (sonar1Get > sonar2Get) {
            turnAutonomous(R_zionAutoExecutionCap);
        }
        else if (sonar2Get > sonar1Get) {
            turnAutonomous(-R_zionAutoExecutionCap);
        }
    }
    stopDriving();


    Limelight limelight;
    // TODO: Fix when can be called properly.
    // Moving side to side to align with target.
    while (abs(limelight.getHorizontalOffset()) >= angleDeadzone) {

        if (limelight.getHorizontalOffset() > 0.5) {
            driveAutonomous(backwardsVector, R_zionAutoExecutionCap);
        }
        else if (limelight.getHorizontalOffset() < -0.5) {
            driveAutonomous(forwardsVector, R_zionAutoExecutionCap);
        }
    }    
    stopDriving(); 
    

    // Moving to get the proper distance away from the target.
    while (abs((sonar1Get + sonar2Get) / 2.0 - optimalDistance) >= 5) { //5 represents a cm distance deadzone btween the robot and the wall
        double averageDistance  = (sonar1Get + sonar2Get) / 2.0; 
            
        if (averageDistance < optimalDistance) {
            driveAutonomous(leftVector, R_zionAutoExecutionCap);
        }
        else if (averageDistance > optimalDistance) {
            driveAutonomous(rightVector, R_zionAutoExecutionCap);
        }
    }
    stopDriving();

    // TODO: Find what speeds the launch motor has to go and proper names.
    //SetIndexSpeed(50000);
    //SetLaunchSpeed(50000);
}


// Moves Zion to get more power cells from the trench while autonomous.
// Needs motor encoder values to move a specified distance.
void SwerveTrain::getTrenchRunPowerCells() {
   
    // Distances to move. Will change based on trench measurements.
    // Movement to the left, in inches. After testing, set to ~72.
    double sideMovementDist = 30;
    // How far to move forwards into trench, also in inches.
    double forwardsDistance = 60;

    double wheelCircumference = 4 * M_PI;
        
    // Vectors for movement in directions.
    VectorDouble leftVector(1, 0);
    VectorDouble forwardsVector(0, 1);


    // Rotate to line up for intake.
    navX->resetAll(); 

    while (90 - abs(navX->getYaw()) > 0) {
        turnAutonomous(R_zionAutoExecutionCap);
    }
    stopDriving();


    // The second value we need after rotation. Uses front right wheel.
    double leftEncodeValue = m_frontRight->getDrivePosition();

    // The goal motor encoder value for left movement.
    double leftGoalEncoderValue = leftEncodeValue + ((sideMovementDist / wheelCircumference) * R_kuhnsConstant);
    
    while ((leftGoalEncoderValue - m_frontRight->getDrivePosition()) > 0) {
        driveAutonomous(leftVector, R_zionAutoExecutionCap);
    }
    stopDriving();


    // Starting encoder value for forwards movement.
    double thirdEncodeValue = m_frontRight->getDrivePosition();

    // Motor encoder value we need to get to for forwards movement.
    double forwardsGoalEncValue = thirdEncodeValue + ((forwardsDistance / wheelCircumference) * R_kuhnsConstant);

    // Moves Zion forward until motor encoder value is at the goal.
    while ((forwardsGoalEncValue - m_frontRight->getDrivePosition()) > 0) {
        driveAutonomous(forwardsVector, R_zionAutoExecutionCap);
    }
    stopDriving();
}

// Moves Zion back to original shooting position.
// Needs motor encoder values to move a specified distance.
void SwerveTrain::returnToShootingPosition() {
   
    // Distances to move. Will change based on trench measurements.
    // How far to move forwards out of trench, in inches.
    double backwardsDistance = 60;
    //Movement to the left, also in inches. Set to ~72 inches; confirm actual value.
    double sideMovementDist = 30;

    double wheelCircumference = 4 * M_PI;

    // Vectors for movement in a linear direction.
    VectorDouble rightVector(-1, 0);
    VectorDouble backwardsVector(0, -1);
    VectorDouble forwardsVector(0, 1);


    // Moves back down the trench run to get back to shooting distance.
    double backwardsEncodeValue = m_frontRight->getDrivePosition();

    // Motor encoder value we need to get to for backwards movement.
    double backGoalEncValue = backwardsEncodeValue + ((backwardsDistance / wheelCircumference) * R_kuhnsConstant);

    // Backs Zion up the distance set above.
    while ((backGoalEncValue - m_frontRight->getDrivePosition()) > 0) {
        driveAutonomous(backwardsVector, R_zionAutoExecutionCap);
    }
    stopDriving();


    // After moving back, move sideways to line up with target.
    // The encoder value we need for side-to-side. Uses front right wheel.
    double sideToSideEncodeValue = m_frontRight->getDrivePosition();

    // The goal motor encoder value for left movement.
    double sToSGoalEncoderValue = sideToSideEncodeValue + ((sideMovementDist / wheelCircumference) * R_kuhnsConstant);

    while ((sToSGoalEncoderValue - m_frontRight->getDrivePosition()) > 0) {
        driveAutonomous(rightVector, R_zionAutoExecutionCap);
    }
    stopDriving();


    navX->resetAll();

    while (90 - abs(navX->getYaw()) > 0) {
        turnAutonomous(-R_zionAutoExecutionCap);
    }
    stopDriving();

    driveAutonomous(forwardsVector, 0);
}

// TODO: Add rest of functions for remaining movements.

// Order of operations for autonomous mode.
void SwerveTrain::runZionAutonomous() {
    moveToTarget();
    lineupShot();
    //shootAuto();
    getTrenchRunPowerCells();
    returnToShootingPosition();
    lineupShot();
    //shootAuto();
}

// Sets direction and speed for linear movement.
void SwerveTrain::driveAutonomous(VectorDouble driveVector, double speed) {
    m_frontRight->assumeSwervePosition(getClockwiseREVRotationsFromCenter(driveVector));
    m_frontLeft->assumeSwervePosition(getClockwiseREVRotationsFromCenter(driveVector));
    m_rearLeft->assumeSwervePosition(getClockwiseREVRotationsFromCenter(driveVector));
    m_rearRight->assumeSwervePosition(getClockwiseREVRotationsFromCenter(driveVector));

    setDriveSpeed(speed);
}

// Tells Zion to stop moving, usually when a task is complete. 
void SwerveTrain::stopDriving() {
    setDriveSpeed(0);
}

// Sets vectors and speed for turning.
void SwerveTrain::turnAutonomous(double speed){
    double squareOfTwo = sqrt(2) / 2;

    // Wheel vectors for rotation.
    VectorDouble frontRightVector(speed*squareOfTwo, speed*squareOfTwo);
    VectorDouble frontLeftVector(speed*squareOfTwo, -speed*squareOfTwo);
    VectorDouble rearLeftVector(-speed*squareOfTwo, -speed*squareOfTwo);
    VectorDouble rearRightVector(-speed*squareOfTwo, speed*squareOfTwo);

    m_frontRight->assumeSwervePosition(getClockwiseREVRotationsFromCenter(frontRightVector));
    m_frontLeft->assumeSwervePosition(getClockwiseREVRotationsFromCenter(frontLeftVector));
    m_rearLeft->assumeSwervePosition(getClockwiseREVRotationsFromCenter(rearLeftVector));
    m_rearRight->assumeSwervePosition(getClockwiseREVRotationsFromCenter(rearRightVector));

    setDriveSpeed(abs(speed));
}
