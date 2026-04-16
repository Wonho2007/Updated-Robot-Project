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

DigitalInputPin backLeftBumper(FEHIO::Pin7);
DigitalInputPin backRightBumper(FEHIO::Pin0);
DigitalInputPin frontLeftBumper(FEHIO::Pin6);
DigitalInputPin frontRightBumper(FEHIO::Pin2);

const float countsPerInch = (318 / (PI * 3));
const float countsPerDegrees = (6.9 * PI / 360) * countsPerInch; // 6.875 og

void driveTime(int percent, float seconds) // using encoders
{

    // Set both motors to desired percent

    if (percent > 0)
    {
        // Forward
        right_motor.SetPercent(percent);
        left_motor.SetPercent(percent + 1);
    }
    else
    {
        right_motor.SetPercent(percent);
        left_motor.SetPercent(percent - 2);
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
    if (inches > 0)
    {
        right_motor.SetPercent(percent);
        left_motor.SetPercent(percent + 1);
    }
    else
    {
        right_motor.SetPercent(-percent);
        left_motor.SetPercent(-percent - 2);
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
        left_motor.SetPercent((percent + 4));
    }
    else
    {
        // Set both motors to desired percent
        right_motor.SetPercent(percent);
        left_motor.SetPercent(-percent - 4);
    }

    // Wait until the average of the left and right encoder is less than counts
    while ((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts + 8)
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

void bumpDriveBack(int percent)
{
    right_motor.SetPercent(-percent);
    left_motor.SetPercent(-percent - 2);
    // Wait for back bumpers to get hit
    float startTime = TimeNow();
    while (backLeftBumper.Value() || backRightBumper.Value())
    {
        // If back left bumper gets hit
        if (!backLeftBumper.Value())
        {
            left_motor.SetPercent(0);
        }
        else
        {
            left_motor.SetPercent(-percent - 2);
        }

        if (!backRightBumper.Value())
        {
            right_motor.SetPercent(0);
        }
        else
        {
            right_motor.SetPercent(-percent);
        }

        // If either lever is down, wait 5 seconds before breaking out
        if (!backLeftBumper.Value() || !backRightBumper.Value())
        {
            if (TimeNow() - startTime > 5)
            {
                break;
            }
        }

        // break out if going backwards for 10 sec
        if (TimeNow() - startTime > 10)
        {
            break;
        }
    }

    right_motor.SetPercent(0);
    left_motor.SetPercent(0);
}

void bumpDriveBackLeft(int percent)
{
    right_motor.SetPercent(-percent);
    left_motor.SetPercent(-percent - 2);
    // Wait for back bumpers to get hit
    float startTime = TimeNow();
    while (backLeftBumper.Value())
    {
        // break out if going backwards for 5 sec
        if (TimeNow() - startTime > 5)
        {
            break;
        }
    }

    right_motor.SetPercent(0);
    left_motor.SetPercent(0);
}

void bumpDriveBackRight(int percent)
{
    right_motor.SetPercent(-percent);
    left_motor.SetPercent(-percent - 2);
    // Wait for back bumpers to get hit
    float startTime = TimeNow();
    while (backRightBumper.Value())
    {
        // break out if going backwards for 5 sec
        if (TimeNow() - startTime > 5)
        {
            break;
        }
    }

    right_motor.SetPercent(0);
    left_motor.SetPercent(0);
}

void bumpDriveForward(int percent)
{
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent + 2);
    // Wait for back bumpers to get hit
    float startTime = TimeNow();
    while (frontLeftBumper.Value() || frontRightBumper.Value())
    {
        // If back left bumper gets hit
        if (!frontLeftBumper.Value())
        {
            left_motor.SetPercent(0);
        }

        if (!frontRightBumper.Value())
        {
            right_motor.SetPercent(0);
        }

        // If either lever is down, wait 5 seconds before breaking out
        if (!frontLeftBumper.Value() || !frontRightBumper.Value())
        {
            if (TimeNow() - startTime > 5)
            {
                break;
            }
        }

        // break out if going backwards for 10 sec
        if (TimeNow() - startTime > 10)
        {
            break;
        }
    }

    right_motor.SetPercent(0);
    left_motor.SetPercent(0);
}

void bumpDriveFrontRight(int percent)
{
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent + 2);
    // Wait for back bumpers to get hit
    float startTime = TimeNow();
    while (frontRightBumper.Value())
    {
        // break out if going backwards for 3 sec
        if (TimeNow() - startTime > 3)
        {
            break;
        }
    }

    right_motor.SetPercent(0);
    left_motor.SetPercent(0);
}

void hitLeverA(int percent, int upDegrees)
{
    // Lower arm
    arm.SetDegree(180);
    Sleep(0.5);
    driveDistance(percent, -4);
    Sleep(4.2);

    arm.SetDegree(150);
    Sleep(0.1);
    arm.SetDegree(180);
    Sleep(0.1);

    // turn to get under lever
    turnCenter(percent, 5);
    driveDistance(percent, 4);
    turnCenter(percent, -12);

    arm.SetDegree(upDegrees);
    Sleep(0.1);
    driveTime(-percent, 1);
    arm.SetDegree(180);
    arm.SetDegree(50);
}

void hitLeverC(int percent, int upDegrees)
{
    // Lower arm
    arm.SetDegree(180);
    Sleep(0.5);
    driveDistance(percent, -4);
    Sleep(4.2);

    arm.SetDegree(150);
    Sleep(0.1);
    arm.SetDegree(180);
    Sleep(0.1);

    // turn to get under lever
    turnCenter(percent, 5);
    driveDistance(percent, 4);
    turnCenter(percent, -12);

    arm.SetDegree(upDegrees);
    Sleep(0.1);
    driveTime(-percent, 1);
    arm.SetDegree(180);
    driveDistance(percent, -2);
    arm.SetDegree(50);
}

void hitLeverB(int percent, int upDegrees)
{
    // Lower arm
    arm.SetDegree(180);
    Sleep(0.5);
    driveDistance(percent, -4);
    Sleep(4.2);

    arm.SetDegree(150);
    Sleep(0.1);
    arm.SetDegree(180);
    Sleep(0.1);

    // turn to get under lever
    turnCenter(percent, 5);
    driveDistance(percent, 4.5);
    turnCenter(percent, -15);

    arm.SetDegree(upDegrees);
    Sleep(0.1);
    driveTime(-percent, 1);
    arm.SetDegree(180);
    turnCenter(percent, 2);
    driveDistance(percent, -2);
    arm.SetDegree(50);
}

void openWindow(int percent)
{
    right_motor.SetPercent(-percent);
    left_motor.SetPercent(-percent-2);

    float startTime = TimeNow();

    
    while (!RCS.isWindowOpen())
    {
        if(TimeNow()-startTime > 5.0)
        {
            break;
        }
        Sleep(0.05);
    }
    

    right_motor.SetPercent(0);
    left_motor.SetPercent(0);
}

void closeWindow(int percent)
{
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent+2);

    float startTime = TimeNow();

    
    while (RCS.isWindowOpen())
    {
        /*
        if(TimeNow()-startTime > 3.0)
        {
            break;
        }
        */
    }
    

    right_motor.SetPercent(0);
    left_motor.SetPercent(0);
}

void driveCompost(int percent, float inches) // using encoders
{

    float counts = countsPerInch * abs(inches);
    // Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    // If driving backwards, set negative percent
    // Set both motors to desired percent
    if (inches > 0)
    {
        right_motor.SetPercent(percent);
        left_motor.SetPercent(percent + 1);
    }

    int oldLeftCounts = left_encoder.Counts();
    int oldRightCounts = right_encoder.Counts();
    // While the average of the left and right encoder is less than counts,
    // keep running motors
    while ((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts)
    {
        Sleep(0.1);
        oldLeftCounts = left_encoder.Counts();
        oldRightCounts = right_encoder.Counts();
        if((left_encoder.Counts() - oldLeftCounts) < 5 || (right_encoder.Counts() - oldRightCounts) < 5)
        {
            right_motor.Stop();
            left_motor.Stop();

            driveDistance(percent, -3);
            turnCenter(percent, 90);
            driveDistance(percent, 1);
            turnCenter(percent, -90);

            driveDistance(percent, 6);
            break;
        }
    }

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
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
    const int windowSpeed = 35;
    const int rampMotorSpeed = 70;
    const int fastMotorSpeed = 60;
    const float rampDistance = 34.5;
    const float tableToWindowBackDist = 12;
    const float tableToLeverBack = 7;
    const float tableToHumidifierBack = 1.25;
    const float windowOpenDist = 21;
    const float windowCloseDist = 7;
    const float cdsRedHighThresh = 0.55;
    const float cdsBlueHighThresh = 1.2;
    const float upDegrees = 43;        // og 50
    const float appleUpDegrees = 95;   // OG 95
    const float parallelDegrees = 150; // OG 160
    const int compostOff = 84;
    const int compostForward = 0;
    const int compostBackward = 180;

    int x, y; // for touch screen

    // Initialize the screen
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    arm.SetMin(800);
    arm.SetMax(2500);
    arm.SetDegree(0);

    compost.SetMin(500);
    compost.SetMax(2500);
    compost.SetDegree(compostOff);

    Sleep(1.0);
    float cdsValue = cdsCell.Value();

    RCS.InitializeTouchMenu("0910B8VYV");

    WaitForFinalAction();

    // Wait for cds cell to read start light

    LCD.Clear();
    LCD.WriteLine("Waiting for start.");
    cdsValue = cdsCell.Value();
    boolean lightGoodTwice = false;

    
    while (!lightGoodTwice)
    {
        cdsValue = cdsCell.Value();
        LCD.Clear();
        LCD.WriteLine(cdsValue);
        if (cdsValue < cdsRedHighThresh)
        {
            Sleep(0.1);
            cdsValue = cdsCell.Value();
            if (cdsValue < cdsRedHighThresh)
            {
                lightGoodTwice = true;
            }
        }
    }

    

    // Drive into button.
    LCD.Clear();
    LCD.WriteLine("Driving");

    driveTime(-motorSpeed, 0.3);
    driveTime(motorSpeed, 0.3);
    

    //---Drive to compost bin---
    // Drive forward
    driveDistance(motorSpeed, 3.2);
    

    // Turn to face compost bin, drive forward
    turnCenter(motorSpeed, -44);
    driveDistance(motorSpeed, 15);

    LCD.WriteLine("driving to compost");
    //driveCompost(motorSpeed, 15);
    LCD.WriteLine("done");

    // Turn on motor
    compost.SetDegree(compostForward);

    // Turn wheel into compost bin
    turnCenterTime(-motorSpeed, 0.7);

    // Wait 1.5 seconds, reverse motor
    Sleep(1.25);
    turnCenterTime(motorSpeed, 0.1);
    compost.SetDegree(compostBackward);
    turnCenterTime(-motorSpeed, 0.2);

    
    // Wait 2 seconds, turn off motor
    Sleep(1.5);
    compost.SetDegree(compostOff);

    //-------APPLE BUCKET---------
    // Turn to apple stump and go forward slightly
    turnCenter(motorSpeed, 23 + 90);
    driveDistance(motorSpeed, 2.5); // OG 2

    // Turn to left wall to align
    turnCenter(motorSpeed, -90);
    // bumpDriveForward(motorSpeed + 10);
    driveTime(motorSpeed + 10, 1);

    // Drive to back wall
    driveDistance(motorSpeed, -13);

    // Drive to apple bucket
    turnCenter(motorSpeed, 90);
    driveDistance(motorSpeed, 6.6);
    turnCenter(motorSpeed, -90);

    // Pick up bucket
    arm.SetDegree(parallelDegrees);
    Sleep(0.2);
    driveDistance(motorSpeed, 3.9); // OG 3.5

    Sleep(0.2);
    LCD.WriteLine("raise arm");

    LCD.WriteLine("raising");
    arm.SetDegree(appleUpDegrees);
    Sleep(0.5);
    arm.SetDegree(upDegrees);
    Sleep(0.4);

    // Slightly turn and back up from tree
    turnCenter(motorSpeed, 25);
    driveDistance(motorSpeed, -17);

    // Finish turn to ramp
    turnCenter(motorSpeed, 63); // OG 65

    driveDistance(rampMotorSpeed, rampDistance);

    turnCenter(motorSpeed, -100);

    // Align with back wall

    arm.SetDegree(appleUpDegrees);

    bumpDriveBack(motorSpeed);
    driveDistance(motorSpeed, 2);

    // Turn to table

    turnCenter(motorSpeed, 98);
    // bumpDriveFrontRight(motorSpeed);
    driveTime(motorSpeed, 2);

    // Back up from table, drop off bucket
    arm.SetDegree(appleUpDegrees + 40);
    Sleep(0.2);
    driveDistance(motorSpeed, -5);

    // Drive into table
    Sleep(0.2);
    arm.SetDegree(upDegrees);
    Sleep(0.2);
    driveTime(motorSpeed, 1.5);

    // Back up from table, drive to humidifier
    driveDistance(motorSpeed, -tableToHumidifierBack);

    // Turn to humidifier.
    LCD.Clear();
    LCD.WriteLine("Turning");
    turnCenter(motorSpeed, -90);
    bumpDriveBack(motorSpeed);

    // Drive to humidifier light
    driveDistance(motorSpeed, 18);

    cdsValue = cdsCell.Value();

    // Check left and right
    float leftCdsValue = 5;
    float rightCdsValue = 5;
    // If reading not red, turn left and check
    if (cdsValue > cdsRedHighThresh)
    {
        turnCenter(slowMotorSpeed, -2);
        leftCdsValue = cdsCell.Value();
        turnCenter(slowMotorSpeed, 2);

        // If left wiggle wasn't red either, check right
        if (leftCdsValue > cdsRedHighThresh)
        {
            turnCenter(slowMotorSpeed, 2);
            rightCdsValue = cdsCell.Value();
            turnCenter(slowMotorSpeed, -2);
        }

        // If to the right of the light
        if (cdsValue > leftCdsValue)
        {
            cdsValue = leftCdsValue;
            turnCenter(motorSpeed, -2);
            driveDistance(motorSpeed, 0.4);
            turnCenter(motorSpeed, 2);
        }

        // If to the left of the light
        if (cdsValue > rightCdsValue)
        {
            cdsValue = rightCdsValue;
            turnCenter(motorSpeed, 2);
            driveDistance(motorSpeed, 0.4);
            turnCenter(motorSpeed, -2);
        }
    }

    if (cdsValue > cdsBlueHighThresh)
    {
        LCD.Write("TIME OUT: GOING RED");
        cdsValue = 0.20;
        turnCenter(motorSpeed, 4);
        driveDistance(motorSpeed, 0.4);
        turnCenter(motorSpeed, -4);
    }

    // CHECK WHICH SIDE WIGGLE WORKED

    // Check which light
    if (cdsValue > cdsRedHighThresh) // Blue
    {
        // LCD.Clear(BLUE);
        LCD.WriteLine("Blue");
        turnCenter(motorSpeed, -11);
        driveTime(37, 1);

        driveTime(-37, 1);
        turnCenter(motorSpeed, 11);
    }
    else // Red
    {
        // LCD.Clear(RED);
        LCD.WriteLine("Red");
        turnCenter(motorSpeed, 11);
        driveTime(37, 1.2);

        driveTime(-37, 1.2);
    }

    // Drive to back wall.
    driveDistance(fastMotorSpeed, -10);
    bumpDriveBack(motorSpeed);






    // align with table
    //  Drive off wall, drive into table
    driveDistance(motorSpeed, 4.3);
    turnCenter(motorSpeed, 91);
    driveTime(motorSpeed, 3);
    // back off table
    driveDistance(motorSpeed, -tableToLeverBack);
    // Turn to face levers, drive to levers
    turnCenter(motorSpeed, -38);
    driveDistance(motorSpeed, 15.5);

    // Get correct lever from the RCS
    int correctLever = RCS.GetLever();

    // Check which lever to flip and perform some action
    if (correctLever == 0)
    {
        // Perform actions to flip left lever A
        turnCenter(motorSpeed, -26);
        driveDistance(motorSpeed, 2);
        hitLeverA(motorSpeed, parallelDegrees);

        // Turn back and align with wall for button
        turnCenter(motorSpeed, 30);
        driveDistance(motorSpeed, -10);
        turnCenter(motorSpeed, -52);

        bumpDriveBack(motorSpeed);
    }
    else if (correctLever == 1)
    {
        // Perform actions to flip middle lever B
        turnCenter(motorSpeed, -1);
        hitLeverB(motorSpeed, parallelDegrees);

        // Turn back and align with wall for button
        driveDistance(motorSpeed, -9);
        turnCenter(motorSpeed, -52);

        bumpDriveBack(motorSpeed);
    }
    else if (correctLever == 2)
    {
        // Perform actions to flip right lever C
        turnCenter(motorSpeed, 10);
        driveDistance(motorSpeed, 1);
        hitLeverB(motorSpeed, parallelDegrees);

        // Turn back and align with wall for button
        turnCenter(motorSpeed, -10);
        driveDistance(motorSpeed, -8);
        turnCenter(motorSpeed, -52);

        bumpDriveBack(motorSpeed);
    }






    // Drive off wall, turn back to window
    driveDistance(motorSpeed, 4.3);

    turnCenter(motorSpeed, -91);
    bumpDriveBackLeft(motorSpeed);

    // drive forward towards window
    driveDistance(motorSpeed, tableToWindowBackDist);

    //---Drive to window---

    // Turn to window.
    LCD.Clear();
    LCD.WriteLine("Turning");
    turnCenter(motorSpeed, -90);
    driveTime(motorSpeed, 1);

    // Open window
    driveDistance(windowSpeed, -windowOpenDist);


    
    // Turn to realign with back wall
    driveDistance(motorSpeed, 4);
    turnCenter(motorSpeed, 20);

    driveTime(-fastMotorSpeed, 3);
    
    driveDistance(motorSpeed, 1);

    turnCenter(motorSpeed, 10);
    driveDistance(motorSpeed, 3);
    turnCenter(motorSpeed, -5);

    driveDistance(motorSpeed, windowCloseDist);


    driveDistance(motorSpeed, -3);
    turnCenter(motorSpeed, -17);
    


    // align with back wall for final button
    turnCenter(motorSpeed, -180);
    driveDistance(fastMotorSpeed, -10);
    bumpDriveBack(motorSpeed);

    // Back off wall, drive to hit button
    driveDistance(motorSpeed, 3);

    turnCenter(motorSpeed, -92);
    driveTime(fastMotorSpeed, 4);
}