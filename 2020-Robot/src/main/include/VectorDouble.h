/*
class VectorDouble

Constructors:

    VectorDouble(const double&, const double&): Creates a 2D vector with its
        components i and j, such that A<i,j>.

Public Methods:

    double operator* const(VectorDouble&): Returns the dot product of two
        VectorDoubles.
    VectorDouble operator+ const(VectorDouble&): Returns the resultant vector
        of the addition of two VectorDoubles.
    double magnitude(): Returns the magnitude of a VectorDouble.
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

        double i;
        double j;
};
