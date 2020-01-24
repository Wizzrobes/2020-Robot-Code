FRC Team 4624's 2020-Robot-Code
 All software for the Rebel Alliance's 2020 season and its game, Infinite
  Recharge.

 2020-Robot
  The primary project for 2020: commanding Zion to play this year's game.
  Description
   4624's 2020 robot resides on a swerve drivetrain based on Team 2910's
    Swerve Drive Specialties MKII swerve modules. Controlling this is
    the first objective of the project, with movement and rotation in
    any direction and of any combination. Methods then exist for
    operating an indexer to store Power Cells, as well as a launcher
    for throwing them. Finally, this is all wrapped in a system of
    fully autonomous extension methods to complete tasks such as
    positioning, alignment, launching, and climbing either
    automatically or with autonomous driver assistance.
  Documentation
   The code in this project is fully documented, and a thorough
    description of each class and method exists at the beginning
    of its respective header file.
  Compiling/Usage
   This is a 2020 WPILib project which uses vendor libraries from
    REV and Kaui for motor controller and sensor functionality.
    With an up-to-date install of WPILib VSCode and these
    vendor libraries installed, the project can be imported
    and built automatically through GradleRIO. Otherwise, the
    code can be browsed on GitHub or locally.

 2020-Robot-Arduino
  The software running on the RoboRIO's coprocessor, an Arduino Uno R3.
  Description
   There are many sensors in use this year which require operation too
    quick and low-level to be effectively managed by the RIO or its
    libraries, which thus find themselves much more at home on the low-
    overhead and quick loop time of a microprocessor. These sensors
    are pinged by the Arduino, and then their results and tramsitted
    back to the RIO over USB over serial.
  Documentation
   The code in this project is also fully documented, with a complete
    write-up in the Arduino header file in 2020-Robot. Documentation
    follows the template laid out in 2020-Robot.
  Compiling-Usage
   This project is written and maintained in the Arduino IDE version
    1.8.4 - any version higher than 1.0 (and possibly older) should
    be able to both compile it and upload it to a board just fine.
