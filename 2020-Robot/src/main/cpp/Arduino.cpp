#include <string>
#include <iterator>

#include "Arduino.h"

std::string Arduino::getRegister(const int &regToGet) {

    std::string resultString;
    //If we're looking in the sanity register, just return the first digit in the
    //registry and don't bother with the rest...
    if (regToGet == kSanity) {

        resultString += m_registry.at(0);
    }
    //Likewise, if we just want the whole thing, set the result string equal to it...
    else if (regToGet == kRegistry) {

        resultString += m_registry;
    }
    //Otherwise, we need to index the string between two points (the limits of a register)
    //and return the value found therein. As the enum values are set to the beginning
    //of their respective registers...
    else {

        //Set a begin and end iterator of the registry string where begin starts at the
        //location described by the enum and end occurs three places later (four so we can
        //use !=). This then sandwiches the area we want to return...
        std::string::iterator beg = m_registry.begin();
        beg += regToGet;
        std::string::iterator end = beg + 4;

        //So iterate over the values contained, appending each to the result string...
        for (; beg != end; ++beg) {

            resultString += *beg;
        }
    }
    //And return the result.
    return resultString;
}
