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

void ERCMain()
{
    const int slowMotorSpeed = 20; // Input power level here
    const int motorSpeed = 25;
    const int rampMotorSpeed = 70; // Originially 70
    const int fastMotorSpeed = 100;
    const float rampDistance = 35;
    const float tableToWindowBackDist = 11.5;
    const float windowForwardDist = 23;
    const float cdsRedHighThresh = 0.55;
    const float cdsBlueLowThresh = 0.55;
    const float cdsBlueHighThresh = 1.2;

    int x, y; // for touch screen

    // Initialize the screen
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);


    RCS.InitializeTouchMenu("0910B8VYV");

    // Wait for cds cell to read start light
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
    driveTime(motorSpeed, 0.5);
    driveTime(-motorSpeed, 0.5);

    // Turn 45 degrees to face ramp
    LCD.Clear();
    LCD.WriteLine("Turning towards ramp");
    turnCenter(motorSpeed, 45);
    Sleep(0.1);

    // Turn to right of ramp and move forward to align.
    turnCenter(motorSpeed, 45);
    Sleep(0.1);
    driveDistance(motorSpeed, -7.5); // Used to be 6.5
    turnCenter(motorSpeed, -45);
    Sleep(0.1);

    // Fly up ramp 😎
    LCD.Clear();
    LCD.WriteLine("Going up ramp");
    driveDistance(rampMotorSpeed, -rampDistance);
    // ram into table
    driveTime(-slowMotorSpeed, 5);

    // drive backwards away from table
    driveDistance(motorSpeed, tableToWindowBackDist);

    //---Drive to window---

    // Turn to face window.
    LCD.Clear();
    LCD.WriteLine("Turning");
    turnCenter(motorSpeed, 93);
    driveTime(-motorSpeed, 2);

    // Drive to levers
    /*
    Code
    Code
    */

    // Get correct lever from the RCS
    int correctLever = RCS.GetLever();

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