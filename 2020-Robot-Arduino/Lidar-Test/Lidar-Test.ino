#include <Wire.h>
#include <LIDARLite.h>

LIDARLite lidarLite;

void setup() {
    
    Serial.begin(9600);

    lidarLite.begin(0, true);
    lidarLite.configure(0);
}

void loop() {

    //Average 100 readings of the LIDAR with the last omitting distance
    //correction per the documentation, convert the result to inches,
    //and print it to the serial monitor
    double totalDistance = 0;
    for (int x = 0; x != 99; ++x) {

        totalDistance += lidarLite.distance();
    }
    totalDistance += lidarLite.distance(false);

    totalDistance /= 100;
    totalDistance /= 2.54;

    Serial.println(totalDistance);
}
