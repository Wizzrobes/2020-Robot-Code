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
    B. Option Register 0: This register is modified by the RoboRIO and then
        transmitted to the Arduino. It can be used to transmit any alphanumeric
        data, but it is logically used in modifying the behavior of whatevever
        device is returning data on Data Register 0.
    C-F. Option Register 1-Option Register 4
    G. Data Register 0: This register is modified by the Arduino and then
        transmitted to the RoboRIO. It can also be used to receive any
        alphanumeric data from the Arduino, but it is logically used
        to receive data from whatever sensor is transmitting on Register 0
    H-K. Data Register 1-Data Register 4
    L. Extra Register: Unused. 32 is a nice number.

    On the RIO, the registry is a WPI::StringRef object, allowing it to
        interface with the FRC Serial functions. It is usually referenced by a
        temporary string to be operated on, and then used as-is.
*/