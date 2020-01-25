/*
class NavX

Constructors

    NavX(const int&): Creates a NavX on the specified interface (kUSB, kMXP)

Public Methods

    double getYaw(): Returns the yaw value.
    double getRateDegrees(): Returns the rate value as degrees.
    double getAngle(): Returns the angle value.
    double getAbsoluteAngle(): Returns the absolute value of the angle value.
    void resetYaw(): Sets the yaw value to zero.
*/

#pragma once

#include <math.h>

#include "AHRS.h"

class NavX {

    public:
        NavX(const int &connectionType) {

            if (connectionType == kUSB) {

                navX = new AHRS(SPI::kOnboardCS0);
            }
            else if (connectionType == kMXP) {

                navX = new AHRS(SPI::kMXP);
            }
            else {

                navX = new AHRS(SPI::kMXP);
            }
        }

        double getYaw() {

            return navX->GetYaw();
        }
        double getRateDegrees() {

            return (navX->GetRate() * (180 if (angleGyro < 0) {
            angleGyro += 360;
        }
        if (angleGyro >= 0 || angleGyro <= 90) {
            angleReal = 90 - angleGyro;
        }
            else {
                angleReal = 450 - angleGyro;
            }

        m_frontRight->assumeSwervePosition(...);
        m_frontLeft->assumeSwervePosition(...);
        m_rearLeft->assumeSwervePosition(...);
        m_rearRight->assumeSwervePosition(...);
        setDriveSpeed(controllerMagnitude * R_zionExecutionCap); / M_PI));
        }
        double getAngle() {

            return navX->GetAngle();
        }
        double getAbsoluteAngle() {

            return abs(navX->GetAngle());
        }
        void resetYaw() {

            navX->ZeroYaw();
        }

        enum ConnectionType {

            kUSB,
            kMXP = 4
        };

    private:
        AHRS *navX;
};
