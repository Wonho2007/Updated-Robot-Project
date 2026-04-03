#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRCS.h>
#include <FEHSD.h>
#include <FEHRCS.h>

// Declarations for encoders & motors
AnalogInputPin cdsCell(FEHIO::Pin12);
DigitalEncoder right_encoder(FEHIO::Pin8);
DigitalEncoder left_encoder(FEHIO::Pin10);
FEHMotor right_motor(FEHMotor::Motor0, 9.0);
FEHMotor left_motor(FEHMotor::Motor1, 9.0);
FEHServo arm(FEHServo::Servo7);

const float countsPerInch = (318 / (PI * 3));
const float countsPerDegrees = (6.875 * PI / 360) * countsPerInch;

void driveTime(int percent, float seconds) // using encoders
{
    // Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);

    // While the timer is less than seconds,
    // keep running motors
    float timeStart = TimeNow();

    while ((TimeNow() - timeStart) < seconds)
        ;

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

// Assumes percent > 0
void driveDistance(int percent, int inches) // using encoders
{

    if (inches == 0)
    {
        // Set both motors to desired percent
        right_motor.SetPercent(percent);
        left_motor.SetPercent(percent);

        // while(input)

        right_motor.Stop();
        left_motor.Stop();
    }
    else
    {
        int counts = countsPerInch * abs(inches);
        // Reset encoder counts
        right_encoder.ResetCounts();
        left_encoder.ResetCounts();

        // If driving backwards, set negative percent
        if (inches < 0)
        {
            percent = -1 * percent;
        }

        // Set both motors to desired percent
        right_motor.SetPercent(percent);
        left_motor.SetPercent(percent);

        // While the average of the left and right encoder is less than counts,
        // keep running motors
        while ((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts)
            ;

        // Turn off motors
        right_motor.Stop();
        left_motor.Stop();
    }
}

// Assumes percent > 0;
void turnCenter(int percent, int degrees) // Positive degrees turns right. Negative turns left.

{
    // Find counts needed for degrees
    int counts = abs(degrees) * countsPerDegrees;

    // Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    left_motor.Stop();
    right_motor.Stop();

    // If degrees is positive, turn to right. If negative, turn left
    if (degrees > 0)
    {
        // Set both motors to desired percent
        right_motor.SetPercent(-percent);
        left_motor.SetPercent(percent);
    }
    else
    {
        // Set both motors to desired percent
        right_motor.SetPercent(percent);
        left_motor.SetPercent(-percent);
    }

    // Wait until the average of the left and right encoder is less than counts
    while ((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts)
        ;

    right_motor.Stop();
    left_motor.Stop();
}

// Assumes percent > 0;
void turnAboutWheel(int percent, int degrees, char wheelPivot) // using encoders

{
    // Find counts needed for degrees
    int counts = abs(degrees) * countsPerDegrees;

    // Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    left_motor.Stop();
    right_motor.Stop();

    // Check which wheel is pivot
    if (wheelPivot == 'R')
    {
        // If degrees is positive, turn to right. If negative, turn left
        if (degrees < 0)
        {
            percent = -1 * percent;
        }

        left_motor.SetPercent(percent);
    }
    else if (wheelPivot == 'L')
    {
        // If degrees is positive, turn to right. If negative, turn left
        if (degrees < 0)
        {
            percent = -1 * percent;
        }

        right_motor.SetPercent(percent);
    }
    else
    {
        LCD.Write("Not valid pivot");
    }

    // Wait until the average of the left and right encoder is less than counts
    while ((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts)
        ;

    right_motor.Stop();
    left_motor.Stop();
}

void pulse(int percent)
{
    // Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);

    Sleep(0.2);

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

void slowArmSetDegrees(float curentDegrees, float targetDegrees)
{
    if (targetDegrees < curentDegrees) // Raising arm
    {
        for (int i = curentDegrees; i < targetDegrees; i += 5)
        {
            arm.SetDegree(i - curentDegrees);
            Sleep(0.1);

            if(targetDegrees - i < 5)
            {
                i = targetDegrees;
                arm.SetDegree(i - curentDegrees);
            }
        }
    }
    else //Lowering arm
    {
        for (int i = curentDegrees; i < targetDegrees; i += 5)
        {
            arm.SetDegree(i + curentDegrees);
            Sleep(0.1);

            if(targetDegrees - i < 5)
            {
                i = targetDegrees;
                arm.SetDegree(i + curentDegrees);
            }
        }
    }
}

void ERCMain()
{
    const int slowMotorSpeed = 20; // Input power level here
    const int motorSpeed = 25;
    const int rampMotorSpeed = 50;
    const int fastMotorSpeed = 100;
    const float rampDistance = 27;
    const float tableToWindowBackDist = 11.5;
    const float windowForwardDist = 23;
    const float cdsRedHighThresh = 0.55;
    const float cdsBlueLowThresh = 0.55;
    const float cdsBlueHighThresh = 1.2;
    const float upDegrees = 50;
    const float appleUpDegrees = 95;
    const float parallelDegrees = 160;

    int x, y; // for touch screen

    // Initialize the screen
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    arm.SetMin(830);
    arm.SetMax(2500);
    arm.SetDegree(upDegrees);
    Sleep(1.0);

    //RCS.InitializeTouchMenu("0910B8VYV");

    //Wait for cds cell to read start light
    
    LCD.Clear();
    LCD.WriteLine("Waiting for start.");
    float cdsValue = cdsCell.Value();
    while (cdsValue > cdsRedHighThresh)
    {
        cdsValue = cdsCell.Value();
        LCD.Clear();
        LCD.WriteLine(cdsValue);
    }
    



    // Drive into button.
    LCD.Clear();
    LCD.WriteLine("Driving");
    Sleep(1);
    driveTime(-motorSpeed, 0.5);
    driveTime(motorSpeed, 0.5);

    //---Drive to apple bucket---
    // Turn slightly to right and move forward.
    turnCenter(motorSpeed, 19);
    driveDistance(motorSpeed, 16);

    // Turn to face apple bucket. Move forward to push arm under handle.
    LCD.Clear();
    LCD.WriteLine("Turning towards apple");
    turnCenter(motorSpeed, -62); //OG 61

    // Move back, lower arm, move forward
    driveDistance(motorSpeed, -2);
    Sleep(0.1);
    arm.SetDegree(parallelDegrees);
    Sleep(1.0);
    driveDistance(motorSpeed, 5);
    driveTime(motorSpeed, 0.1);

    Sleep(0.2);
    LCD.WriteLine("raise arm");
    
    LCD.WriteLine("raising");
    arm.SetDegree(appleUpDegrees);
  

    // Slightly turn and back up from tree
    turnCenter(motorSpeed, 25);
    driveDistance(motorSpeed, -17);

    //Finish turn to ramp
    turnCenter(motorSpeed, 80);

    driveDistance(rampMotorSpeed, rampDistance);
    turnCenter(motorSpeed, 30);
    driveTime(motorSpeed, 2);

    // Back up from table, drop off bucket
    arm.SetDegree(appleUpDegrees+40);
    Sleep(1.0);
    driveDistance(motorSpeed, -5);

    //Drive into table
    Sleep(1.0);
    arm.SetDegree(upDegrees);
    driveTime(motorSpeed, 1);

    //Back up from table, drive to levers
    driveDistance(motorSpeed, -5.75);

    turnCenter(motorSpeed, -90);
    driveTime(-motorSpeed, 1);
    driveDistance(motorSpeed, 8);
    turnCenter(motorSpeed, 48);
    driveDistance(motorSpeed, 11);

    //Lower arm
    arm.SetDegree(180);
    Sleep(7.0);

    driveDistance(motorSpeed, -4);

    arm.SetDegree(180);
    Sleep(1.0);
    driveDistance(motorSpeed, 4);
    arm.SetDegree(upDegrees+10);
    Sleep(0.5);
    driveDistance(motorSpeed, -3);

    // Get correct lever from the RCS
    int correctLever; //= RCS.GetLever();

    // Check which lever to flip and perform some action
    if (correctLever == 0)
    {
        // Perform actions to flip left lever
    }
    else if (correctLever == 1)
    {
        // Perform actions to flip middle lever
    }
    else if (correctLever == 2)
    {
        // Perform actions to flip right lever
    }
}