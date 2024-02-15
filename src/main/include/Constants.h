/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include<math.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
namespace constants{

    // titan ID
    static constexpr int TITAN_ID = 42;
    // Motor 
    static constexpr int RightMotor = 1;
    static constexpr int LeftMotor = 0;
    static constexpr int BackMotor = 2;
    // Encoder 
    static constexpr int WheelRadios = 51; //mm
    static constexpr int PULS_PER_REV = 1464;
    static constexpr int GEAR_RATIO = 1/1;
    static constexpr int ENCODER_PULS_RATIO = PULS_PER_REV * GEAR_RATIO;
    static constexpr int DIST_PER_TICK =  (M_PI * 2 * WheelRadios) / ENCODER_PULS_RATIO;

    static constexpr double ToRadian(double x){
    return ( (x * M_PI) / 180);
    }
}





