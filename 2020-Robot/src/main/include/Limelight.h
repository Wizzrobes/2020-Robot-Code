/*
class Limelight

Constructors

    Limelight
        Creates a limelight on the default NetworkTable interface.

Public Methods

    double horizontalOffset()
        Returns the horizontal offset of the target (tx).
    double verticalOffset()
        Returns the vertical offset of the target (ty).
    double targetArea()
        Returns the area of the target in-sight.
    bool target()
        Returns true if there is a target in-sight, false otherwise.
    All return 0 in event of a null target.
*/

#pragma once

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

class Limelight {

    public:
        Limelight() {

            table = NetworkTable::GetTable("limelight");
        }

        double horizontalOffset() {

            return table->GetNumber("tx", 0);
        }
        double verticalOffset() {

            return table->GetNumber("ty", 0);
        }
        double targetArea() {

            return table->GetNumber("ta", 0);
        }
        bool target() {

            return table->GetNumber("tv", 0);
        }

    private:
        std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
};
