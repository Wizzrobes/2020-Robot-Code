/*
struct VectorDouble

Constructors:

    VectorDouble(const double&, const double&)
        Creates a 2D vector with its components i and j, such that A<i,j>.

Public Methods:

    double operator* const(VectorDouble&)
        Returns the dot product of two VectorDoubles.
    VectorDouble operator+ const(VectorDouble&)
        Returns the resultant vector of the addition of two VectorDoubles.
    double magnitude()
        Returns the magnitude of the VectorDouble.
    double unitCircleAngleDeg()
        Returns the angle in degrees of the operated vector when inscribed in
        standard position.
*/

#pragma once

#include <math.h>

struct VectorDouble {

        VectorDouble(const double &iVal, const double &jVal) {

            i = iVal;
            j = jVal;
        }

        double operator* (VectorDouble const &otherVector) {

            return ((i * otherVector.i) + (j * otherVector.j));
        }
        VectorDouble operator+ (VectorDouble const &otherVector) {

            VectorDouble resultVector (i + otherVector.i, j + otherVector.j);
            return resultVector;
        }

        double magnitude() {

            return sqrt(pow(i, 2) + pow(j, 2));
        }
        //TODO: Inline function documentation
        double unitCircleAngleDeg() {

            double calculatedAngle = 0;

            //Quadrant I
            if (i > 0 && j > 0) {

                calculatedAngle = atan(j / i) * (180 / M_PI);
            }
            //Quadrant II
            else if (i < 0 && j > 0) {

                calculatedAngle = 180 - atan(j / -i) * (180 / M_PI);
            }
            //Quadrant III
            else if (i < 0 && j < 0) {

                calculatedAngle = 180 + atan(-j / -i) * (180 / M_PI);
            }
            //Quadrant IV
            else if (i > 0 && j < 0) {

                calculatedAngle = 360 - atan(-j / i) * (180 / M_PI);
            }
            else if (i == 0 && j > 0) {

                calculatedAngle = 90;
            }
            else if (i == 0 && j < 0) {

                calculatedAngle = 270;
            }
            else if (i > 0 && j == 0) {

                calculatedAngle = 0;
            }
            else if (i < 0 && j == 0) {

                calculatedAngle = 180;
            }
            return calculatedAngle;
        }

        double i;
        double j;
};
