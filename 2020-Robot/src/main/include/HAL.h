#include "Arduino.h"
#include "Intake.h"
#include "Launcher.h"
#include "Limelight.h"
#include "NavX.h"
#include "RobotMap.h"
#include "SwerveTrain.h"

class HAL {

    public:
        HAL(Arduino &refArduino, Intake &refIntake, Launcher &refLauncher, Limelight &refLimelight, NavX &refNavX, SwerveTrain &refZion) {

            *arduino = refArduino;
            *intake = refIntake;
            *launcher = refLauncher;
            *limelight = refLimelight;
            *navX = refNavX;
            *zion = refZion;
        }

    private:
        Arduino *arduino;
        Intake *intake;
        Launcher *launcher;
        Limelight *limelight;
        NavX *navX;
        SwerveTrain *zion;
};
