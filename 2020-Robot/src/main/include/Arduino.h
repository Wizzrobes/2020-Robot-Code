/*
This class allows interfacing with an Arduino microcontroller as an auxiliary
    processor to the RoboRIO with both data transmission (Tx) and reception
    (Rx). This is implemented for use in exporting sensor data of sensors
    which require more low-level operation and pinging than the RIO can handle
    alone, as well as commanding devices connected to an Arduino, such as RGB
    LED strips, which are also too high-refresh and low-level for the RIO to
    efffectively manage. The way it does this is over USB, which is over
    serial. To minimize and simplify the amount of serial transactions
    necessary, it only completes one transaction with one object each time data
    transmission occurs, and it fulfills this with a string called the
    registry, formed of multiple registers (locations where data is stored). On
    each cycle, the RoboRIO sends the registry out over serial, and the Arduino
    responds with the registry in-turn. Data can be added or extracted from the
    registry, which fulfills both transmission and reception of alphanumeric
    data. Below is a diagram of the 32-character registry, which contains
    8 registers each delimited with |:

    |x|xxxxx|xxxxx|xxxxx|xxxxx|xxxxx|xxxxx|x|
    A B     C     D     E     F     G     H

    A. Begin Register: This value should read 'A' when the Rio transmits the
        registry, and will read 'B' after the Arduino responds with the registry.
        Any other value at any other time is in error. This is used to ensure
        success over many serial transactions occurring at many times.
    B. Tx Register 0: This register is modified by the RoboRIO and then
        transmitted to the Arduino. It can be used to transmit any alphanumeric
        data, but it is logically used in modifying the behavior of whatevever
        device is returning data on Rx Register 0.
    C-D. Tx Register 1-Tx Register 2
    E. Rx Register 0: This register is modified by the Arduino and then
        transmitted to the RoboRIO. It can also be used to receive any
        alphanumeric data from the Arduino, but it is logically used
        to receive data from whatever sensor is transmitting on Register 0.
    F-G. Rx Register 1-Rx Register 2
    H. End Register: This value should read 'Y' when the Rio transmits, and
        'Z' on receive. This is to ensure that both ends of the registry
        were transmitted and received properly. Length is also checked.

    On the RIO, the registry is a WPI::StringRef object, allowing it to
        interface with the FRC Serial functions. It is usually referenced by a
        temporary string to be operated on, and then used as-is.


class Arduino

Constructors

    Arduino(const frc::SerialPort::Port&, const int&): Creates a serial
        interface called Arduino on the specified port at the specified
        baud rate. These default to frc::SerialPort::Port::kUSB1 and
        115200, respectively.

Public Methods

    std::string getRegister(const Register&): Takes a register as defined in
        enum Register and returns a string of the values found there. Does
        no error checking itself! :)
    bool setRegister(const Register&, const std::string&): Takes a register
        as defined in enum Register and sets its full contents to the most
        usable data of the passed string starting at the beginning. Returns
        true if that operation was possible, false if it was not. In case of
        failure, no data in the registry is updated!
    Note that these get/setters allow getting and setting of things that
        shouldn't necessarily be get or set - setting a received register
        is such an example, as is tampering with one to be transmitted. Due
        to this, the order is set, tx, rx, get. This allows a bit more control
        over the whole system. The entire thing is pretty low-level;
        be careful.
    bool tx(): Configures and transmits the registry, returns false if any of
        those events fail at any time. If false is returned, no data was sent!
    bool rx(): Tries to receive 32 bytes to place into the registry. If it
        gets all of them, sets it to the registry, if it doesn't, no data is
        updated! TODO: DOES NO ERROR CHECKING YET

    enum Registry: Used as a paramater for the get and set function to denote
        which register is being operated on in the registry.
*/

#pragma once

#include <array>

#include <frc/SerialPort.h>
#include <wpi/StringRef.h>

class Arduino {

    public:
        Arduino(const frc::SerialPort::Port &port = frc::SerialPort::Port::kUSB1, const int &baudRate = 115200) {

            arduino = new frc::SerialPort(baudRate, port);
            //Initialize the registry to its documented state
            m_registry = std::string("A0000000000000000000000000000000Y");
        }

        std::string getRegister(const int &regToGet);
        bool setRegister(const int &regToSet, const std::string &stringToSet);

        bool tx() {

            //Setup the registry to be transmitted and check its total length;
            //while doing so, return false if any of the operations fail, and
            //then transmit if they all pass.
            if (setRegister(Registry::kBegin, "A")) {

                if (setRegister(Registry::kEnd, "Y")) {

                    if (m_registry.length() == 32) {

                        if (arduino->Write(m_registryRef) == 32) {

                            return true;
                        }
                    }
                }
            }
            return false;
        }
        bool rx() {

            std::string receivedRegister;
            for (int currentPosition = 0; currentPosition != 32; ++currentPosition) {

                char *currentCharacter;
                if (arduino->Read(currentCharacter, 1)) {

                    receivedRegister += *currentCharacter;
                }
                else {

                    return false;
                }
            }
            m_registry = receivedRegister;
            return true;
        }

        //These values are used to pass registry locations to functions. kBegin, kEnd, and kRegistry are
        //arbitrary, whereas the rest of the register names indicate their own start positions
        //in the registry.
        enum Registry {

            kBegin,
            kRegisterTx0 = 1, kRegisterTx1 = 6, kRegisterTx2 = 11,
            kRegisterRx0 = 16, kRegisterRx1 = 21, kRegisterRx2 = 26,
            kEnd = 31,
            kRegistry
        };

    private:
        frc::SerialPort *arduino;

        //Setup a string to operate on the registry, and refer a StringRef to it for seial use
        std::string m_registry;
        wpi::StringRef m_registryRef = m_registry;
};
