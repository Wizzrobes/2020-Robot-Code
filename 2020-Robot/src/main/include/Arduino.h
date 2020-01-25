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
    12 registers each delimited with |:

    |x|xxx|xxx|xxx|xxx|xxx|xxx|xxx|xxx|xxx|xxx|x|
    A B   C   D   E   F   G   H   I   J   K   L

    A. Sanity Register: This value should read 0 when the Rio transmits the
        registry, and will read 1 after the Arduino respomds with the registry.
        Any other value at any other time is in error. This is used to ensure
        success over many serial transactions occurring at many times.
    B. Tx Register 0: This register is modified by the RoboRIO and then
        transmitted to the Arduino. It can be used to transmit any alphanumeric
        data, but it is logically used in modifying the behavior of whatevever
        device is returning data on Rx Register 0.
    C-F. Tx Register 1-Tx Register 4
    G. Rx Register 0: This register is modified by the Arduino and then
        transmitted to the RoboRIO. It can also be used to receive any
        alphanumeric data from the Arduino, but it is logically used
        to receive data from whatever sensor is transmitting on Register 0.
    H-K. Rx Register 1-Rx Register 4
    L. Extra Register: Unused. 32 is a nice number.

    On the RIO, the registry is a WPI::StringRef object, allowing it to
        interface with the FRC Serial functions. It is usually referenced by a
        temporary string to be operated on, and then used as-is.


class Arduino

Constructors

    Arduino(const frc::SerialPort::Port&, const int&): Creates a serial
        interface called Arduino on the specified port at the specified
        baud rate. These default to frc::SerialPort::Port::kUSB1 and
        9600, respectively.

Public Methods

    std::string getRegister(const int&): Takes a register as defined in
        enum Register and returns a string of the value found there.
*/

#pragma once

#include <frc/SerialPort.h>
#include <wpi/StringRef.h>

class Arduino {

    public:
        Arduino(const frc::SerialPort::Port &port = frc::SerialPort::Port::kUSB1, const int &baudRate = 9600) {

            arduino = new frc::SerialPort(baudRate, port);
            //Initialize the registry to 32 0 chars
            m_registry = std::string(32, '0');
        }

        std::string getRegister(const int &regToGet);

        //These values are used to pass registry locations to functions. kSanity and kRegistry are
        //arbitrary, whereas the rest of the register names indicate their own start positions
        //in the registry.
        enum Register {

            kSanity,
            kRegisterTx0, kRegisterTx1 = 4, kRegisterTx2 = 7, kRegisterTx3 = 10, kRegisterTx4 = 13,
            kRegisterRx0 = 16, kRegisterRx1 = 19, kRegisterRx2 = 22, kRegisterRx3 = 25, kRegisterRx4 = 28,
            kRegisterExtra = 31,
            kRegistry
        };

    private:
        frc::SerialPort *arduino;

        //Setup a string to operate on the registry, and refer a StringRef to it for seial use
        std::string m_registry;
        wpi::StringRef m_registryRef = m_registry;
};
