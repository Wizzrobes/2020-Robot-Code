//See Arduino.h in 2020-Robot for the theory of this system.

//Since Serial operates on character arrays, set up one of those
//along with a string for ease of access.
char registryArray[32];
String registry;

void setup() {

    Serial.begin(115200);
    Serial.setTimeout(50);
    for (int x = 2; x < 13; x += 2) {

        pinMode(x, OUTPUT);
    }
}
void loop() {

    //At the beginning of each loop, set the array equal to 32 (the size
    //of a full register) meaningless characters (which are not used by
    //the registry) and the registry itself equal to 32 whitespaces.
    for (unsigned currentChar = 0; currentChar != 32; ++currentChar) {

        registryArray[currentChar] = '|';
    }
    registry = "                                ";

    //Once data is available from the Rio, try to read 32 chars of it
    //into the array.
    if (Serial.available() > 0) {

        Serial.readBytes(registryArray, 32);
    }
    //Set each of those chars which isn't the meaningnless character
    //to the string. The meaningless character allows the string to
    //represent the true amount of information transmitted by the Rio.
    for (unsigned currentChar = 0; currentChar != 32; ++currentChar) {

        if (registryArray[currentChar] != '|') {

            registry[currentChar] = registryArray[currentChar];
        }
    }
    //To complete representing only the true amount of information,
    //remove all of the whitespace from the registry that we added
    //at the beginning. This turns "potential" into "actual".
    registry.trim();

    //If we actually received something...
    if (registry.length() > 0) {

        //If there's data left over (more than 32 chars sent)...
        if (Serial.available() > 0) {

            //Discard and read the rest of the data to null...
            Serial.println("NACK(TooLen):");
            while (Serial.available()) {

                Serial.read();
                delay(5);
            }
        }
        //If there wasn't enough data (less than 32 chars sent)...
        else if (registry.length() < 32) {

            Serial.println("NACK(BadLen):" + registry);
        }
        //If there was, but the beginning is incorrect...
        else if (registry.charAt(0) != 'A') {

            Serial.println("NACK(BadBeg):" + registry);
        }
        //If there was, but the ending is incorrect...
        else if (registry.charAt(31) != 'Y') {

            Serial.println("NACK(BadEnd):" + registry);
        }
        //If we did everything correctly...
        else {

            //Form a string of the first tx register (six characters after the first)
            String optionRegister = "000000";
            for (unsigned x = 1; x != 7; ++x) {

                optionRegister[x - 1] = registry.charAt(x);
            }
            //Step through it and assign 1s and 0s to LEDs on even pins 12-2 sequentially
            for (unsigned x = 0; x != 6; ++x) {

                if (optionRegister.charAt(x) == '1') {

                    digitalWrite(12 - 2 * x, HIGH);
                }
                else {

                    digitalWrite(12 - 2 * x, LOW);
                }
            }
        }
    }
}
