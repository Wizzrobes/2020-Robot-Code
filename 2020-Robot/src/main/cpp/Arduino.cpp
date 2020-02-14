#include <string>
#include <iterator>

#include "Arduino.h"

std::string Arduino::getRegister(const int &regToGet) {

    std::string resultString;
    //If we're looking in the begin register, just return the first digit in the
    //registry and don't bother with the rest...
    if (regToGet == Registry::kBegin) {

        resultString += m_registry.at(0);
    }
    //If we're looking in the end register, just return the last digit...
    if (regToGet == Registry::kEnd) {

        resultString += m_registry.at(31);
    }
    //Likewise, if we just want the whole thing, set the result string equal to it...
    else if (regToGet == Registry::kRegistry) {

        resultString += m_registry;
    }
    //Otherwise, we need to index the string between two points (the limits of a register)
    //and return the value found therein. As the enum values are set to the beginning
    //of their respective registers...
    else {

        //Set a begin and end iterator of the registry string, where begin starts at the
        //location described by the enum and end occurs five places later (six so we can
        //use !=). This then sandwiches the area we want to return...
        std::string::iterator beg = m_registry.begin();
        beg += regToGet;
        std::string::iterator end = beg + 6;
        //So iterate over the values contained, appending each to the result string...
        for (; beg != end; ++beg) {

            resultString += *beg;
        }
    }
    //And return the result.
    return resultString;
}
bool Arduino::setRegister(const int &regToSet, const std::string &stringToSet) {

    //If setting the begin register...
    if (regToSet == Registry::kBegin) {

        //Make sure that the supplied string has at least one character available...
        if (stringToSet.length() >= 1) {

            //If so, set the first char in the registry equal to it, returning success...
            m_registry.at(Registry::kBegin) = stringToSet.at(0);
            return true;
        }
        //Otherwise, return failure.
        else {

            return false;
        }
    }
    //If setting the end register, do practically the same...
    if (regToSet == Registry::kEnd) {

        if (stringToSet.length() >= 1) {

            m_registry.at(Registry::kEnd) = stringToSet.at(0);
            return true;
        }
        else {

            return false;
        }
    }
    //If setting the whole registry...
    if (regToSet == Registry::kRegistry) {

        //Ensure that the supplied string has a full 32 chars available...
        if (stringToSet.length() >= 32) {

            //If so, loop through all 32 and set them, indicating success...
            for (unsigned currentChar = 0; currentChar != 32; ++currentChar) {

                m_registry.at(currentChar) = stringToSet.at(currentChar);
            }
            return true;
        }
        //Otherwise, indicate failure.
        else {

            return false;
        }
    }
    //If we've made it this far, it means that we're setting a regular registry,
    //so make sure that there's enough available chars for that (5)...
    if (stringToSet.length() >= 5) {

        //Compose an iterator at the beginning of the registryToSet and the end
        //of it, as before...
        std::string::iterator beg = m_registry.begin();
        beg += regToSet;
        std::string::iterator end = beg + 6;
        //Loop through the iterator sandwhich and thus the first five chars of
        //the string to apply, overwriting char by char in the process...
        for (unsigned currentToSetChar = 0; beg != end; ++beg, ++currentToSetChar) {

            *beg = stringToSet.at(currentToSetChar);
        }
        //And indicate success at the end...
        return true;
    }
    //Otherwise, return failure.
    else {

        return false;
    }
}