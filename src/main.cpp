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
FEHServo arm(FEHServo::Servo5);
FEHServo compost(FEHServo::Servo7);

const float countsPerInch = (318 / (PI * 3));
const float countsPerDegrees = (6.9 * PI / 360) * countsPerInch; // 6.875 og

void driveTime(int percent, float seconds) // using encoders
{
    
    // Set both motors to desired percent

    if(percent > 0)
    {
        //Forward
        right_motor.SetPercent(percent);
        left_motor.SetPercent(percent+1);
    } else
    {
        right_motor.SetPercent(percent);
        left_motor.SetPercent(percent-2);
    }
    

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
void driveDistance(int percent, float inches) // using encoders
{

    float counts = countsPerInch * abs(inches);
    // Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    // If driving backwards, set negative percent
    // Set both motors to desired percent
    if(inches > 0)
    {
        right_motor.SetPercent(percent);
        left_motor.SetPercent(percent+1);
    } else
    {
        right_motor.SetPercent(-percent);
        left_motor.SetPercent(-percent-2);
    }

    // While the average of the left and right encoder is less than counts,
    // keep running motors
    while ((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts)
        ;

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
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
        left_motor.SetPercent((percent+4));
    }
    else
    {
        // Set both motors to desired percent
        right_motor.SetPercent(percent);
        left_motor.SetPercent(-percent-4);
    }

    // Wait until the average of the left and right encoder is less than counts
    while ((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts+8)
        ;

    right_motor.Stop();
    left_motor.Stop();
    Sleep(0.1);
}

// Assumes percent > 0;
void turnCenterTime(int percent, float seconds) // Positive degrees turns right. Negative turns left.

{
    // Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    left_motor.Stop();
    right_motor.Stop();

    // If degrees is positive, turn to right. If negative, turn left
    right_motor.SetPercent(-percent);
    left_motor.SetPercent(percent);

    Sleep(seconds);

    right_motor.Stop();
    left_motor.Stop();
    Sleep(0.1);
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

            if (targetDegrees - i < 5)
            {
                i = targetDegrees;
                arm.SetDegree(i - curentDegrees);
            }
        }
    }
    else // Lowering arm
    {
        for (int i = curentDegrees; i < targetDegrees; i += 5)
        {
            arm.SetDegree(i + curentDegrees);
            Sleep(0.1);

            if (targetDegrees - i < 5)
            {
                i = targetDegrees;
                arm.SetDegree(i + curentDegrees);
            }
        }
    }
}

/*------------ PID VARIABLES ----------*/
const float inchesPerCount = 1 / countsPerInch;
float pastTimeLeft = 0;
float pastTimeRight = 0;
float changeInTime = 0;

float pastErrorLeft = 0;
float errorLeft = 0;
float errorSumLeft = 0;

float pastErrorRight = 0;
float errorRight = 0;
float errorSumRight = 0;

float pastLeftCounts = 0;
float pastRightCounts = 0;
float changeInLeftCounts = 0;
float changeInRightCounts = 0;
float actualVelocity;

float PTerm = 0;
float ITerm = 0;
float DTerm = 0;
float PConstant = 1.0;
float IConstant = 0.02;
float Dconstant = 0.25;
float oldMotorSpeed = 50;

void resetPIDVar()
{
    pastTimeLeft = 0;
    pastTimeRight = 0;
    changeInTime = 0;

    pastErrorLeft = 0;
    errorLeft = 0;
    errorSumLeft = 0;

    pastErrorRight = 0;
    errorRight = 0;
    errorSumRight = 0;

    pastLeftCounts = 0;
    pastRightCounts = 0;
    changeInLeftCounts = 0;
    changeInRightCounts = 0;
    actualVelocity;

    PTerm = 0;
    ITerm = 0;
    DTerm = 0;
    PConstant = 0.75;
    IConstant = 0.7;
    Dconstant = 0.25;
    oldMotorSpeed = 25;
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();

    pastTimeLeft = TimeNow();
    pastTimeRight = TimeNow();

    Sleep(0.1);
}

float LeftMotorPIDAdjustment(float expectedVelocity)
{
    changeInLeftCounts = left_encoder.Counts() - pastLeftCounts;
    changeInTime = TimeNow() - pastTimeLeft;
    actualVelocity = inchesPerCount * (changeInLeftCounts / changeInTime);
    errorLeft = expectedVelocity - actualVelocity;
    errorSumLeft += errorLeft;

    PTerm = errorLeft * PConstant;
    ITerm = errorSumLeft * IConstant;
    DTerm = (errorLeft - pastErrorLeft) * Dconstant;
    pastErrorLeft = errorLeft;
    pastLeftCounts = left_encoder.Counts();
    pastTimeLeft = TimeNow();

    return (oldMotorSpeed + PTerm + ITerm + DTerm);
}

float RightMotorPIDAdjustment(float expectedVelocity)
{
    changeInRightCounts = right_encoder.Counts() - pastRightCounts;
    changeInTime = TimeNow() - pastTimeRight;
    actualVelocity = inchesPerCount * (changeInRightCounts / changeInTime);
    errorRight = expectedVelocity - actualVelocity;
    errorSumRight += errorRight;

    PTerm = errorRight * PConstant;
    ITerm = errorSumRight * IConstant;
    DTerm = (errorRight - pastErrorRight) * Dconstant;
    pastErrorRight = errorRight;
    pastRightCounts = right_encoder.Counts();
    pastTimeRight = TimeNow();

    return (oldMotorSpeed + PTerm + ITerm + DTerm);
}

void driveDistancePID(float expectedVelocity, float distance)
{
    resetPIDVar();
    while (left_encoder.Counts() * inchesPerCount < distance && right_encoder.Counts() * inchesPerCount < distance)
    {
        left_motor.SetPercent(LeftMotorPIDAdjustment(expectedVelocity));
        right_motor.SetPercent(RightMotorPIDAdjustment(expectedVelocity));
        Sleep(0.05);
    }

    left_motor.SetPercent(0);
    right_motor.SetPercent(0);
}

void ERCMain()
{
    const int slowMotorSpeed = 20; // Input power level here
    const int motorSpeed = 25;
    const int rampMotorSpeed = 50;
    const int fastMotorSpeed = 60;
    const float rampDistance = 31;
    const float tableToWindowBackDist = 6.5;
    const float tableToLeverBack = 5;
    const float tableToHumidifierBack = 1.25;
    const float windowForwardDist = 23;
    const float cdsRedHighThresh = 0.55;
    const float cdsBlueLowThresh = 0.55;
    const float cdsBlueHighThresh = 1.2;
    const float upDegrees = 50;
    const float appleUpDegrees = 95;
    const float parallelDegrees = 160;
    const int compostOff = 84;
    const int compostForward = 0;
    const int compostBackward = 180;

    int x, y; // for touch screen

    // Initialize the screen
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    arm.SetMin(830);
    arm.SetMax(2500);
    arm.SetDegree(0);

    compost.SetMin(500);
    compost.SetMax(2500);
    compost.SetDegree(compostOff);

    Sleep(1.0);
    float cdsValue = cdsCell.Value();
    
    /*
    while (true)
    {
        LCD.Clear();
        LCD.Write("Touch to turn 90");
        while (!LCD.Touch(&x, &y))
        {
        }
        //driveDistancePID(7, 15);
        turnCenter(motorSpeed, 90);

        LCD.Clear();
        LCD.Write("Turn center -90");

        while (!LCD.Touch(&x, &y))
        {
        }

        LCD.WriteLine("Going");
        //driveDistancePID(7, 15);
        turnCenter(motorSpeed, -90);

        
        
    }
    */
        

    // RCS.InitializeTouchMenu("0910B8VYV");

    // Wait for cds cell to read start light

    /*
    LCD.Clear();
    LCD.WriteLine("Waiting for start.");
    cdsValue = cdsCell.Value();
    while (cdsValue > cdsRedHighThresh)
    {
        cdsValue = cdsCell.Value();
        LCD.Clear();
        LCD.WriteLine(cdsValue);
    }
    */

    // Drive into button.
    LCD.Clear();
    LCD.WriteLine("Driving");
    Sleep(1);
    driveTime(-motorSpeed, 0.5);
    driveTime(motorSpeed, 0.5);

    //---Drive to compost bin---
    // Drive forward
    driveDistance(motorSpeed, 2.5);

    // Turn to face compost bin, drive forward
    turnCenter(motorSpeed, -45);
    driveDistance(motorSpeed, 16);

    
    // Turn on motor
    compost.SetDegree(compostForward);

    // Turn wheel into compost bin
    turnCenterTime(-motorSpeed, 0.7);

    // Wait 1.5 seconds, reverse motor
    Sleep(1.25);
    compost.SetDegree(compostBackward);
    // Wait 2 seconds, turn off motor
    Sleep(1.25);
    compost.SetDegree(compostOff);
    // Go back to hit button.

    // Turn to apple stump and go forward slightly
    Sleep(1.0);
    turnCenter(motorSpeed, 30 + 90);
    driveDistance(motorSpeed, 2);

    // Turn to left wall to align
    turnCenter(motorSpeed, -90);
    driveTime(motorSpeed, 2);

    // Drive to back wall
    driveDistance(motorSpeed, -10); // og -24
    driveDistance(fastMotorSpeed, -14);
    driveTime(-motorSpeed, 2);

    // Drive to apple bucket
    driveDistance(motorSpeed, 15);
    turnCenter(motorSpeed, 90);
    driveDistance(motorSpeed, 6);
    turnCenter(motorSpeed, -90);

    // Pick up bucket
    arm.SetDegree(parallelDegrees);
    Sleep(0.2);
    driveDistance(motorSpeed, 3.5);

    Sleep(0.2);
    LCD.WriteLine("raise arm");

    LCD.WriteLine("raising");
    arm.SetDegree(appleUpDegrees);

    // Slightly turn and back up from tree
    turnCenter(motorSpeed, 25);
    driveDistance(motorSpeed, -17);

    // Finish turn to ramp
    turnCenter(motorSpeed, 65); // OG 75

    driveDistance(rampMotorSpeed, rampDistance);

    turnCenter(motorSpeed, -100);

    driveTime(-motorSpeed, 2);

    driveDistance(motorSpeed, 2);

    turnCenter(motorSpeed, 100);

    driveTime(motorSpeed, 2);

    // Back up from table, drop off bucket
    arm.SetDegree(appleUpDegrees + 40);
    Sleep(1.0);
    driveDistance(motorSpeed, -5);

    // Drive into table
    Sleep(1.0);
    arm.SetDegree(upDegrees);
    driveTime(motorSpeed, 1);

    // Back up from table, drive to levers
    driveDistance(motorSpeed, -tableToHumidifierBack);

    // Turn to humidifier.
    LCD.Clear();
    LCD.WriteLine("Turning");
    turnCenter(motorSpeed, -93);
    driveTime(-motorSpeed, 2);

    // Drive to humidifier light
    driveDistance(motorSpeed, 15);

    // Inch towards light
    cdsValue = cdsCell.Value();
    while (cdsValue > cdsBlueHighThresh)
    {
        pulse(slowMotorSpeed);
        cdsValue = cdsCell.Value();
        LCD.Clear();
        LCD.WriteLine(cdsValue);
        Sleep(0.2);

        // If the cds value is reading blue, inch forward and read again
        if (cdsValue > cdsBlueHighThresh)
        {
            pulse(slowMotorSpeed);
            cdsValue = cdsCell.Value();
            LCD.Clear();
            LCD.WriteLine(cdsValue);
            Sleep(0.2);
        }
    }

    // Check which light
    if (cdsValue > cdsRedHighThresh) // Blue
    {
        // LCD.Clear(BLUE);
        LCD.WriteLine("Blue");
        turnCenter(motorSpeed, -11);
        driveTime(35, 1);

        driveTime(-35, 1);
        turnCenter(motorSpeed, 11);
        
    }
    else // Red
    {
        // LCD.Clear(RED);
        LCD.WriteLine("Red");
        turnCenter(motorSpeed, 11);
        driveTime(35, 1);

        driveTime(-35, 1);
        turnCenter(motorSpeed, -11);
    }

    //Drive to back wall.
    driveDistance(fastMotorSpeed, -12);
    driveTime(-motorSpeed, 2);
    driveDistance(motorSpeed, 2);

    //Turn to align with table
    turnCenter(motorSpeed, 90);
    driveTime(motorSpeed, 1);

    // drive backwards away from table
    driveDistance(motorSpeed, -tableToWindowBackDist);

    //---Drive to window---

    // Turn to window.
    LCD.Clear();
    LCD.WriteLine("Turning");
    turnCenter(motorSpeed, -90);
    driveTime(-motorSpeed, 1);

    //Drive to window
    driveDistance(slowMotorSpeed, windowForwardDist);
    driveDistance(slowMotorSpeed, -windowForwardDist);
    

}