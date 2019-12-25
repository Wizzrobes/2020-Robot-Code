/*
class Limelight

Constructors

    Limelight: Creates a limelight on the default NetworkTable.

Public Methods

    double getHorizontalOffset(): Returns the horizontal offset (tx).
    double getVerticalOffset(): Returns the vertival ofset (ty).
    bool getTarget(): Returns true if there is a target in-sight, false otherwise.
*/

#pragma once

#include "networktables/NetworkTable.h"

class Limelight {

    public:
        Limelight() {

            table = NetworkTable::GetTable("limelight");
        }
        
        double getHorizontalOffset() {

            return table->GetNumber("tx", 0);
        }
        double getVerticalOffset() {

            return table->GetNumber("ty", 0);
        }
        bool getTarget() {

            return table->GetNumber("tv", 0);
        }
    
    private:
        std::shared_ptr<NetworkTable> table;
};
