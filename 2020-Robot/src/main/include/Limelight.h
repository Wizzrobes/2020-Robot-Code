/*
class Limelight

Constructors

    Limelight: Creates a limelight on the default NetworkTable.

Public Methods

    double getHorizontalOffset(): Returns the horizontal offset of the target
        (tx).
    double getVerticalOffset(): Returns the vertical ofset of the target (ty).
    bool getTarget(): Returns true if there is a target in-sight, false
        otherwise.
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

double  limeGang() {
    double limeLightAngle = 0.392699; //Adjust this later as this is an estimate of the angle the limelight is mounted at (in simplified radians)(currently at 20 degrees)
    double aidsConstant = 73.25; //Adjust This Later as this is an estimate as to how high the sensor is
    double kommieConstant = aidsConstant / tan(limeLightAngle); //The Trig behind how to find the distance from Zion to wall
    //kommieConstant is the target distance, adjust the limelight angle accordingly


    if ( 14.5 < kommieConstant < 15.5 ) {

        //Continue Firing the cannoon
    }
    else{ 

        //Stop Firing the cannoon clearly it aint lined up boi
    }

}
