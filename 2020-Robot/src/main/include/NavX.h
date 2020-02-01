/*
class NavX

Constructors

    NavX(const int&): Creates a NavX on the specified interface (kUSB, kMXP)

Public Methods

    double getYaw(): Returns the yaw value.
    double getAngle(): Returns the angle value.
    double getAbsoluteAngle(): Returns the absolute value of the angle value.
    void resetYaw(): Sets the yaw value to zero.
    void resetAll(): Resets all NavX return values.
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
        double getAngle() {

            return navX->GetAngle();
        }
        double getAbsoluteAngle() {

            return abs(navX->GetAngle());
        }

        void resetYaw() {

            navX->ZeroYaw();
        }
        void resetAll() {

            navX->Reset();
        }

        enum ConnectionType {

            kUSB,
            kMXP = 4
        };

    private:
        AHRS *navX;
};
