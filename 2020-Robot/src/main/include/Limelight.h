/*
class Limelight

Constructors

    Limelight
        Creates a limelight on the default NetworkTable interface.

Public Methods

    double getHorizontalOffset()
        Returns the horizontal offset of the target (tx).
    double getVerticalOffset()
        Returns the vertical offset of the target (ty).
    double getTargetArea()
        Returns the area of the target in-sight.
    bool getTarget()
        Returns true if there is a target in-sight, false otherwise.
    All return 0 in event of a null target.
    void setProcessing(const bool& = true)
        Turns on or off the vision processing for using the Limelight
        as a camera. Defaults to on.
    void setLime(const bool& = true)
        Turns the Limelight LEDs on or off. Defaults to on.
*/

#pragma once

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

class Limelight {

    public:
        Limelight() {

            table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-lnchr");
        }

        double getHorizontalOffset() {

            return table->GetNumber("tx",0.0);
        }
        double getVerticalOffset() {

            return table->GetNumber("ty", 0);
        }
        double getTargetArea() {

            return table->GetNumber("ta", 0);
        }
        bool getTarget() {

            return table->GetNumber("tv", 0);
        }

        void setProcessing(const bool &toSet = true) {

            //According to doc, 0 is off, 1 is on
            table->PutNumber("camMode", toSet ? 1 : 0);
        }
        void setLime(const bool &toSet = true) {

            //According to doc, 3 is on, 1 is off, and 2 is blink.
            table->PutNumber("ledMode", toSet ? 3 : 1);
        }
    private:
        std::shared_ptr<NetworkTable> table;
};
