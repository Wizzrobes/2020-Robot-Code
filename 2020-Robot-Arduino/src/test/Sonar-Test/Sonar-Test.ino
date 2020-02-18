const int sensorPin = 9;

//Takes a microsecond value measuring the return speed of a
//sound wave and uses the speed of sound to convert it to inches
inline double microsecondsToInches(double microseconds) {

    //The result is divided by two as we only want the return
    //trip of the sound (to the wall, not to and back)
    return microseconds / 74 / 2;
}

void setup() {

    Serial.begin(9600);
}

void loop() {
  
    double inches;

    //Pulse the sensor pin to activate a pulse
    pinMode(sensorPin, OUTPUT);
    digitalWrite(sensorPin, LOW);
    delayMicroseconds(2);
    digitalWrite(sensorPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(sensorPin, LOW);

    //Read the resulting pulse in as microseconds and convert it to inches
    //using the speed of sound, as it is the duration it took a sound wave
    //to propagate to a target and then return
    pinMode(sensorPin, INPUT);
    inches = microsecondsToInches(pulseIn(sensorPin, HIGH));

    //Print the result over serial and wait an arbitrary amount of time
    Serial.println(inches);
    delay(1000);
}
