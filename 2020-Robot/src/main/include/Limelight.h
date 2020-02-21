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

            return table->GetNumber("tx",0.0);

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
    double limeLightAngle = 0.392699; //Adjust This-it is an estimate of the angle the limelight is mounted at in simplified radians(currently at 20 degrees)
    double sensorHeightConstant = 73.25; // Height difference between limelight and target.
    double targetDistConstant = sensorHeightConstant / tan(limeLightAngle); //Trig behind finding distance from Zion to wall
    //targetHeightConstant is the target distance from wall, adjust the limelight angle accordingly

    if (174 < targetDistConstant < 186) {

        //Continue Firing the cannoon-change the zone as needed to adjust for scoring
    }
    else{ 

        //Stop firing the cannoon clearly it aint lined up just like your hairline
    }

}
