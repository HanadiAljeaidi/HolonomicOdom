#include "subsystems/Train.h"

#define DEBUG true 

Train::Train()
{
        //Motor Invert Flags comment out when needed
    //LEFT.SetInverted(true);
    LEFT_MOTOR.InvertRPM();
    //LeftEncoder.SetReverseDirection();

    //BACK.SetInverted(true);
    BACK_MOTOR.InvertRPM();
    backEncoder.SetReverseDirection();

    //RIGHT.SetInverted(true);
    RIGHT_MOTOR.InvertRPM();
    //RightEncoder.SetReverseDirection();
}

void Train::Periodic()
{
    current_time = std::chrono::high_resolution_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - last_time).count();
   /*
     double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
   */
    double delta_x = (*speedX * cos(constants::ToRadian(GetAngle())) - *speedY * sin(constants::ToRadian(GetAngle()))) * dt;
    double delta_y = (*speedX * sin(constants::ToRadian(GetAngle())) + *speedY * cos(constants::ToRadian(GetAngle()))) * dt;

    odomatry->x+=delta_x;
    odomatry->y+=delta_y;\
    odomatry->z = GetAngle();

    last_time = current_time;
    if (DEBUG == true)
    {
    
        ForwardKinomatic();
        frc::SmartDashboard::PutNumber("odomatry x", odomatry->x);
        frc::SmartDashboard::PutNumber("odomatry y", odomatry->y);
        frc::SmartDashboard::PutNumber("odomatry z", odomatry->z);

        // frc::SmartDashboard::PutNumber("Back  Encoder", GetBackEncoder());
        // frc::SmartDashboard::PutNumber("rigt  Encoder", GetRightEncoder());
        frc::SmartDashboard::PutNumber("Left RPM", LEFT_MOTOR.GetRPM());
        frc::SmartDashboard::PutNumber("Back  RPM", BACK_MOTOR.GetRPM());
        frc::SmartDashboard::PutNumber(" Right RPM", RIGHT_MOTOR.GetRPM());
    }
}


void Train::ResetEncoders()
{
    LeftEncoder.Reset();
    frc2::WaitCommand(0.003_s);
    backEncoder.Reset();
    frc2::WaitCommand(0.003_s);
    RightEncoder.Reset();
    frc2::WaitCommand(0.003_s);
}

double Train::GetLeftMPS()
{
    return( LEFT_MOTOR.GetRPM()*(((2 * M_PI) / 60) * constants::WheelRadios));
}

double Train::GetRightMPS()
{
    return (RIGHT_MOTOR.GetRPM()*(((2 * M_PI) / 60) * constants::WheelRadios));
}

double Train::GetBackMPS()
{
    return (BACK_MOTOR.GetRPM()*(((2 * M_PI) / 60) * constants::WheelRadios));
}

void Train::HolonomicDrive(double x, double y, double z)
{
    // double denomonator = fmax(fabs(y) + fabs(x) + fabs(z), 1.0);
    double LeftSpeed = (((x / 3) + (y / sqrt(3)) + z) * sqrt(3));
    double RightSpeed = (((x / 3) - (y / sqrt(3)) + z) * sqrt(3));
    double BackSpeed = (((-2 * x / 3) + z) * sqrt(3));
  
   double max = fabs(RightSpeed);
   if (fabs(LeftSpeed)> max) max = fabs(LeftSpeed);
   if (fabs(BackSpeed) > max) max = fabs(BackSpeed);

   if( max > 1)
   {
        RightSpeed /= max;
        LeftSpeed /= max;
        BackSpeed /= max;
   }

   LEFT_MOTOR.Set(LeftSpeed);
   frc2::WaitCommand(0.003_s);
   RIGHT_MOTOR.Set(RightSpeed);
   frc2::WaitCommand(0.003_s);
   BACK_MOTOR.Set(BackSpeed);
   frc2::WaitCommand(0.003_s);

}

void Train::ForwardKinomatic()
{
    //x = (-cos(60°) * w1 + cos(60°) * w3) * r
    *speedX = ((-GetRightMPS()/sqrt(3))+(GetLeftMPS()/sqrt(3)));
    //  y = (sin(60°) * w1 + sin(60°) * w3 - w2) * r
    *speedY =   (((2*GetBackMPS())/3) + (-GetRightMPS()/sqrt(3))+ (-GetLeftMPS()/sqrt(3))) ;
    //omega = (w1 + w3 - 2 * w2) * r / sqrt(3)
    *speedTH = ((GetBackMPS()/3) + (GetRightMPS()/sqrt(3))+ (GetLeftMPS()/sqrt(3)));
}

void Train::ResetSpeed(void)
{
    LEFT_MOTOR.Set(0);
    frc2::WaitCommand(0.005_s);
    RIGHT_MOTOR.Set(0);
    frc2::WaitCommand(0.005_s);
    BACK_MOTOR.Set(0);
    
}
void Train::ResetYaw()
{
    navX.ZeroYaw();
}

double Train::GetAngle()
{
    return (navX.GetAngle());
}