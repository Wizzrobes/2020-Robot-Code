//Documentation here is purely inline - see Arduino.h in the main robot
//project for the theory of operation of this system as well as the RIO
//side of things.

const int pinSonarLeft = 2;
const int pinSonarRight = 3;

const int pinSonarLeftInTarget = 4;
const int pinSonarRightInTarget = 5;
const int pinSonarLeftSkew = 6;
const int pinSonarRightSkew = 7;

const double sonarInchesTargetDistance = 12;
//If distances are less than this far away from target in either direction,
//they are reported as target.
const double sonarInchesTargetTolerance = 1;
//If distances are this close together on each side, they are reported as
//not skewed.
const double sonarInchesSkewTolerance = 4;

//Manipulates a sonar sensor on the supplied pin to return the amount of inches
//between the sensor and a target as a double. Has no handling for out-of-range
//targets.
double getSonarPinInches(const int &pin) {

    //Pulse the sensor pin to activate a return pulse as stated in datasheet
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(2);
    digitalWrite(pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin, LOW);

    //Read the resulting pulse in as microseconds and convert it to inches
    //using the speed of sound, as it is the duration it took a sound wave
    //to propagate to a target and then return to the sensor. Divide by two
    //as we want the distance to the target, not to the target and back.
    pinMode(pin, INPUT);
    return pulseIn(pinSonarLeft, HIGH) / 74 / 2;
}

void setup() {

    pinMode(pinSonarLeftInTarget, OUTPUT);
    pinMode(pinSonarRightInTarget, OUTPUT);
    pinMode(pinSonarLeftSkew, OUTPUT);
    pinMode(pinSonarRightSkew, OUTPUT);
}

void loop() {

    double sonarInchesLeft = 0, sonarInchesRight = 0;
    //Average 5 samples of each sonar sensor to its variable for accuracy
    //(measurements are so fast that there isn't a reason not to)
    for (int trial = 0; trial != 5; ++trial) {

        sonarInchesLeft += getSonarPinInches(pinSonarLeft);
    }
    sonarInchesLeft /= 5;

    //Take a short delay to prevent cross-sensor interference
    delay(10);

    for (int trial = 0; trial != 5; ++trial) {

        sonarInchesRight += getSonarPinInches(pinSonarLeft);
    }
    sonarInchesRight /= 5;

    //Check the left and right sensors for meeting their distance target, left
    //and then right, and respond on the relay pins accordingly.
    digitalWrite(pinSonarLeftInTarget, abs(sonarInchesLeft - sonarInchesTargetDistance) < sonarInchesTargetTolerance ? HIGH : LOW);
    digitalWrite(pinSonarRightInTarget, abs(sonarInchesRight - sonarInchesTargetDistance) < sonarInchesTargetTolerance ? HIGH : LOW);

    //Assign the skew pins by comparing the left and right distance within
    //that tolerance, bearing in mind the special conditions laid out.
    //Always set all pins for safety.
    //If left is 10 and right is 5, for example, we are skewed left...
    if (sonarInchesLeft - sonarInchesRight > sonarInchesSkewTolerance) {

        digitalWrite(pinSonarLeftSkew, HIGH);
        digitalWrite(pinSonarRightSkew, LOW);
    }
    //And the inverse for being skewed right...
    else if (sonarInchesRight - sonarInchesLeft > sonarInchesSkewTolerance) {

        digitalWrite(pinSonarLeftSkew, LOW);
        digitalWrite(pinSonarRightSkew, HIGH);
    }
    //If we're out of range (beyond 110 inches, as in the datasheet) fulfill
    //a special condition...
    else if (sonarInchesLeft > 110 && sonarInchesRight > 110) {

        digitalWrite(pinSonarLeftSkew, LOW);
        digitalWrite(pinSonarRightSkew, LOW);
    }
    //If we've made it here, we must be normal in range, so fulfill that
    //full success condition and be done.
    else {

        digitalWrite(pinSonarLeftSkew, HIGH);
        digitalWrite(pinSonarRightSkew, HIGH);
    }
}
