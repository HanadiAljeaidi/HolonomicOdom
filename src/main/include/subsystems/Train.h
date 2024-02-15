#pragma once


#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc/Ultrasonic.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <chrono>
#include <ctime>   
#include "frc/AnalogInput.h"
#include "Constants.h"
#include "studica/TitanQuad.h"
#include "studica/TitanQuadEncoder.h"
#include "studica/Cobra.h"
#include "studica/Servo.h"
#include "studica/ServoContinuous.h"
#include <frc2/command/WaitCommand.h>


#include "AHRS.h"
#include <math.h>

struct odom
{
    double x;
    double y;
    double z;
};

class Train
{
    public:
        Train();
        void Periodic();
        double GetLeftEncoder(void);
        double GetBackEncoder(void);
        double GetRightEncoder(void);
        double GetLeftMPS();
        double GetBackMPS();
        double GetRightMPS();
        double GetAverageEncoderY(void);
        double GetAverageForwardEncoder(void);
        void SetMotorSpeed(double speed);
        double GetYaw(void);
        double GetAngle(void);
        void ResetYaw(void);
        void ResetEncoders(void);
        void HolonomicDrive(double x, double y, double z);
        void ForwardKinomatic();
        void ResetSpeed();
        
    private:
        std::chrono::time_point<std::chrono::high_resolution_clock> current_time = std::chrono::high_resolution_clock::now();
        std::chrono::time_point<std::chrono::high_resolution_clock> last_time = std::chrono::high_resolution_clock::now();
      
        double *speedX;
        double *speedY;
        double *speedTH;

        odom* odomatry;
        
        AHRS navX{frc::SPI::Port::kMXP};
        studica::TitanQuad RIGHT_MOTOR{constants::TITAN_ID, 15600, constants::RightMotor};
        studica::TitanQuad LEFT_MOTOR{constants::TITAN_ID, 15600, constants::LeftMotor};
        studica::TitanQuad BACK_MOTOR{constants::TITAN_ID, 15600, constants::BackMotor};
        studica::TitanQuadEncoder LeftEncoder{LEFT_MOTOR, constants::LeftMotor, constants::DIST_PER_TICK};
        studica::TitanQuadEncoder backEncoder{BACK_MOTOR, constants::BackMotor, constants::DIST_PER_TICK};
        studica::TitanQuadEncoder RightEncoder{RIGHT_MOTOR, constants::RightMotor, constants::DIST_PER_TICK};
        // AHRS navX{frc::SPI::Port::kMXP};

};