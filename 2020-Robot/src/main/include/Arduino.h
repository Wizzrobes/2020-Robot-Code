/*
This class allows interfacing with an Arduino microcontroller as an auxiliary
    processor to the RoboRIO through DIO ports on each controller. Toggling
    the state of certain ports from the RIO illicits a certain response from
    others connected to it, as described in the PhoneBook. Currently, the
    Arduino is only used for dual sonar measurement from the launching side
    of the robot for distance-based triangulation and positioning. See the
    Arduino sketch for more detail on that side of things.

PhoneBook
    ------------------------------
    |RIOPort|Dir|ArduPin|Function|
    ------------------------------
    0       ->  2       RefA: Enables measurement of "far" sonar distances
    1       ->  3       RefB: Enables measurement of "close" sonar distances
    2       <-  4       RefC: Low if no measuring, high if left sonar is target
    3       <-  5       RefD: Low if no measuring, high if rght sonar is target
    4       <-  6       RefE: Low if no measure, high if skewed left
    5       <-  7       RefF: Low if no measure, high if skewed right

    RefA
        Setting this pin high begins measurement and report of sonar
        distances according to the "far" distance as defined in the Arduino
        sketch.
    RefB
        Same as above, but for the "close" sonar distances. This system
        allows measurement of two target distances from the wall. Setting
        both of these pins high is the equivalent of both low. These
        distances are defined in the Arduino sketch,
    RefC
        Once measurement begins, this pin is high if the left sonar sensor
        is in tolerance for the current target distance, "close" or "far".
        Tolerances are also defined in the Arduino sketch.
    RefD
        Same as above, but for the right sensor.
    RefE
        Once measurement begins, this pin is high if the robot needs to
        rotate clockwise in order to be normal to the wall, within another
        tolerance. Low otherwise.
    RefF
        Same as above, but for counterclockwise.


class Arduino

Constructors

    Arduino()
        Creates an interface with the Arduino on the pins as defined in the
        PhoneBook. These are kept out of RobotMap as they need never change.

Public Methods

    void setSonarRxMode(const int& = SonarDistances::kIdle)
        Sets the mode of the Arudino to measure either close or far distances,
        or to go idle, with a SonarDistances argument. Writes this setting to
        the Arduino immediately. Always sets all outbound pins for safety.
        Does nothing in event of malformed argument. Defaults to idling.
    bool getSonarInTarget(const int&)
        Takes a SonarSides argument, returns true if that side was in target,
        false if it was not. Returns false in event of malformed argument.
    bool getSonarSkew(cosnt int&)
        Takes a SonarSides argument, returns true if the robot is skewed
        in that direciton, false if it was not. Returns false in event of a
        malformed argument.

    enum SonarDistances
        Used as a paramater for the setRxMode function to specify either
        close, far, or idle measurement.
    enum SonarSides
        Used as a paramater for the getSonarInTarget function to specify which
        side sensor to read, left or right.
*/

#pragma once

#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>

class Arduino {

    public:
        Arduino() {

            m_txReportFar = new frc::DigitalOutput(0);
            m_txReportClose = new frc::DigitalOutput(1);

            m_rxDistanceLeft = new frc::DigitalInput(2);
            m_rxDistanceRight = new frc::DigitalInput(3);
            m_rxSkewLeft = new frc::DigitalInput(4);
            m_rxSkewRight = new frc::DigitalInput(5);
        }

        void setSonarRxMode(const int &operation = SonarDistances::kIdle) {

            switch (operation) {

                case SonarDistances::kIdle:
                    m_txReportClose->Set(false);
                    m_txReportFar->Set(false);
                    break;
                case SonarDistances::kClose:
                    m_txReportClose->Set(true);
                    m_txReportFar->Set(false);
                    break;
                case SonarDistances::kFar:
                    m_txReportClose->Set(false);
                    m_txReportFar->Set(true);
            }
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

        enum SonarDistances {

            kIdle, kClose, kFar
        };
        enum SonarSides {

            kLeft, kRight
        };

    private:
        frc::DigitalOutput *m_txReportFar;
        frc::DigitalOutput *m_txReportClose;

        frc::DigitalInput *m_rxDistanceLeft;
        frc::DigitalInput *m_rxDistanceRight;
        frc::DigitalInput *m_rxSkewLeft;
        frc::DigitalInput *m_rxSkewRight;
};
