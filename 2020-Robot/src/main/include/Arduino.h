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
    2       <-  4       RefA: Low if no measuring, high if left sonar is target
    3       <-  5       RefB: Low if no measuring, high if rght sonar is target
    4       <-  6       RefC: Low if no measure, high if sonar skewed left
    5       <-  7       RefD: Low if no measure, high if sonar skewed right

    RefA
        Once measurement begins, this pin is high if the left sonar sensor
        is in tolerance for the target distance. Targets and tolerances
        are defined in the Arduino sketch.
    RefB
        Same as above, but for the right sensor.
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

    bool getSonarInTarget(const int&)
        Takes a SonarSides argument, returns true if that side was in target,
        false if it was not. Returns false in event of a malformed argument.
    bool getSonarSkew(cosnt int&)
        Takes a SonarSides argument, returns true if the robot is skewed
        in that direciton, false if it was not. Returns false in event of a
        malformed argument.
    bool getSonarNormal()
        Returns true if the sonar sensors are currently within tolerance of
        being normal with a wall, false if they are not.
    bool getSonarInRange()
        Returns true if either of the sonar sensors are within range of
        a target, false if they are not.

    enum SonarSides
        Used as a paramater for the target and skew functions to specify which
        side of sensor to read, left or right.
*/

#pragma once

#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>

class Arduino {

    public:
        Arduino() {

            m_rxDistanceLeft = new frc::DigitalInput(2);
            m_rxDistanceRight = new frc::DigitalInput(3);
            m_rxSkewLeft = new frc::DigitalInput(4);
            m_rxSkewRight = new frc::DigitalInput(5);
        }

        bool getSonarInTarget(const int &side) {

            switch (side) {

                case SonarSides::kLeft:
                    return m_rxDistanceLeft->Get();
                case SonarSides::kRight:
                    return m_rxDistanceRight->Get();
            }
            return false;
        }
        bool getSonarSkew(const int &skewDirection) {

            switch (skewDirection) {

                case SonarSides::kLeft:
                    return m_rxSkewLeft->Get();
                case SonarSides::kRight:
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

        enum SonarSides {

            kLeft, kRight
        };

    private:

        frc::DigitalInput *m_rxDistanceLeft;
        frc::DigitalInput *m_rxDistanceRight;
        frc::DigitalInput *m_rxSkewLeft;
        frc::DigitalInput *m_rxSkewRight;
};
