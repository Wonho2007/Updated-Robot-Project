#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRCS.h>
#include <FEHSD.h>

// Declarations for encoders & motors
AnalogInputPin cdsCell(FEHIO::Pin3);
DigitalEncoder right_encoder(FEHIO::Pin8);
DigitalEncoder left_encoder(FEHIO::Pin10);
FEHMotor right_motor(FEHMotor::Motor0, 9.0);
FEHMotor left_motor(FEHMotor::Motor1, 9.0);


void driveTime(int percent, int seconds) // using encoders
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
    const float countsPerInch = 40.5;

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
    const float countsPerDegrees = (20.42 / 360) * 40.5;
    int counts = abs(degrees) * countsPerDegrees;
    LCD.WriteLine("Turning about center. Counts needed: ");
    LCD.WriteLine(counts);

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
    const float countsPerDegrees = (20.42 / 360) * 40.5;
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


void ERCMain()
{
    const int motorSpeed = 25; // Input power level here
    const int fastMotorSpeed = 100;
    const int rampDistance = 35;
    const int cdsRedHighThresh = 1;
    const int cdsBlueLowThresh = 1;
    const int cdsBlueHighThresh = 3;

    int x, y; // for touch screen

    // Initialize the screen
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    while (!LCD.Touch(&x, &y))
            ; // Wait for screen to be pressed
    while (LCD.Touch(&x, &y))
            ; // Wait for screen to be unpressed

    /*
    // Wait for cds cell to read start light
    float cdsValue = cdsCell.Value();
    while (cdsValue > cdsRedHighThresh)
    {
        cdsValue = cdsCell.Value();
    }
     */

    // Drive into button.
    driveTime(motorSpeed, 1);
    driveTime(-motorSpeed, 1);

    // Turn 45 degrees to face ramp
    turnCenter(motorSpeed, 45);
    Sleep(0.1);

    // Fly up ramp 😎
    driveDistance(fastMotorSpeed, rampDistance);

    //---Drive to humidifier light---

    /*
    // Read and display the color.
    cdsValue = cdsCell.Value();
    char lightColor = 'N';
    while (cdsValue > cdsBlueHighThresh)
    {
        cdsValue = cdsCell.Value();
    }

    if (cdsValue > cdsBlueLowThresh)
    {
        lightColor = 'B';
        LCD.Clear(BLUE);
    }
    else if (cdsValue < cdsRedHighThresh)
    {
        lightColor = 'R';
        LCD.Clear(RED);
    }

    // Hit humidifier button.
    if (lightColor == 'B')
    {
        //---Insert code---
    }
    else if (lightColor == 'R')
    {
        //---Insert code---
    }
    else // No valid light color stored.
    {
        LCD.Clear(BLACK);
    }

    */
}
