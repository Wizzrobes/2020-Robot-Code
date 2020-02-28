/*
This class allows interfacing with an Arduino microcontroller as an auxiliary
    processor to the RoboRIO through DIO ports on the RIO attached to a relay
    shield on the Arudino. This shield is active on pins 4, 5, 6, and 7.
    Toggling the state of certain pins on the Arduino, and thus on the RIO,
    indicates the status of certain Arduino sensors, as described in the
    PhoneBook. Currently, the Arduino is only used for dual sonar measurement
    from the launching side of the robot for distance-based triangulation and
    positioning. See the Arduino sketch for more detail on that side of things.

PhoneBook
    ------------------------------
    |RIOPort|Dir|ArduPin|Function|
    ------------------------------
    2       <-  4       RefA: High if left sonar too close or at target
    3       <-  5       RefB: High if left sonar too far or at target
    4       <-  6       RefC: High if sonar skewed left or plumb, low if OOR
    5       <-  7       RefD: High if sonar skewed right or plumb, low if OOR

    RefA
        Once measurement begins, this pin is high if the left sonar sensor
        is in tolerance for the target distance or is too close, otherwise
        it is low.  Targets and tolerances are defined in the Arduino sketch.
    RefB
        This is high if the left sonar sensor is in tolerance for the target
        distance or is too far, otherwise it is low.
        If both A and B are high, target is on.
    RefC
        Once measurement begins, this pin is high if the robot needs to
        rotate clockwise in order to be normal to the wall, within another
        tolerance. Low otherwise.
    RefD
        Same as above, but for counterclockwise. If both C and D are high,
        the amount of skew is within tolerance; if both are low, the sensors
        are both out-of-range.


class Arduino

Constructors

    Arduino()
        Creates an interface with the Arduino on the pins as defined in the
        PhoneBook. These are kept out of RobotMap as they need never change.

Public Methods

    bool getSonarTooCloseTarget()
        Returns true if the left sonar is too close to the target.
    bool getSonarTooFarTarget()
        Returns true if the left sonar is too far from the target.
    bool getSonarSkew(cosnt int&)
        Takes a SonarSkews argument, returns true if the robot is skewed
        in that direciton, false if it was not. Returns false in event of a
        malformed argument.
    bool getSonarNormal()
        Returns true if the sonar sensors are currently within tolerance of
        being normal with a wall, false if they are not.
    bool getSonarInRange()
        Returns true if either of the sonar sensors are within range of
        a target, false if they are not.

    enum SonarSkews
        Used as a paramater for the skew function to specify which
        skew to check.
*/

#pragma once

#include <frc/DigitalInput.h>

class Arduino {

    public:
        Arduino() {

            m_rxDistanceLeftTooClose = new frc::DigitalInput(2);
            m_rxDistanceLeftTooFar = new frc::DigitalInput(3);
            m_rxSkewLeft = new frc::DigitalInput(4);
            m_rxSkewRight = new frc::DigitalInput(5);
        }

        bool getSonarTooCloseTarget() {

            return m_rxDistanceLeftTooClose && !m_rxDistanceLeftTooFar ? true : false;
        }
        bool getSonarTooFarTarget() {

            return m_rxDistanceLeftTooFar && !m_rxDistanceLeftTooClose ? true : false;
        }
        bool getSonarSkew(const int &skewDirection) {

            switch (skewDirection) {

                case SonarSkews::kLeft:
                    return m_rxSkewLeft->Get();
                case SonarSkews::kRight:
                    return m_rxSkewRight->Get();
            }
            return false;
        }
        bool getSonarNormal() {

            return m_rxSkewLeft->Get() && m_rxSkewRight->Get() ? true : false;
        }
        bool getSonarInRange() {

            return !m_rxSkewLeft->Get() && !m_rxSkewRight->Get() ? true : false;
        }

        enum SonarSkews {

            kLeft, kRight
        };

    private:

        frc::DigitalInput *m_rxDistanceLeftTooClose;
        frc::DigitalInput *m_rxDistanceLeftTooFar;
        frc::DigitalInput *m_rxSkewLeft;
        frc::DigitalInput *m_rxSkewRight;
};
