#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRCS.h>
#include <FEHSD.h>
#include <FEHRCS.h>
#include <math.h>

// Declarations for encoders & motors
AnalogInputPin cdsCell(FEHIO::Pin12);
DigitalEncoder right_encoder(FEHIO::Pin8);
DigitalEncoder left_encoder(FEHIO::Pin10);
FEHMotor right_motor(FEHMotor::Motor0, 9.0);
FEHMotor left_motor(FEHMotor::Motor1, 9.0);
FEHServo arm(FEHServo::Servo0);
FEHServo compost(FEHServo::Servo7);
FEHServo windowServo(FEHServo::Servo3);

DigitalInputPin backLeftBumper(FEHIO::Pin7);
DigitalInputPin backRightBumper(FEHIO::Pin0);
DigitalInputPin frontLeftBumper(FEHIO::Pin5);
DigitalInputPin frontRightBumper(FEHIO::Pin2);

const float countsPerInch = (318 / (PI * 3));
const float countsPerDegrees = (6.9 * PI / 360) * countsPerInch; // 6.875 og
const float countsAdjustmentDrive = -16.87;
const float countsAdjustmentTurn = 10.158;

void driveTime(int percent, float seconds) // using encoders
{

    // Set both motors to desired percent

    if (percent > 0)
    {
        int adjustedPercentForward = percent + ceil(percent * 0.04);
        right_motor.SetPercent(percent);
        left_motor.SetPercent(adjustedPercentForward);
        
        
    }
    else
    {
        int adjustedPercentForward = percent + ceil(percent * 0.08);
        right_motor.SetPercent(percent);
        left_motor.SetPercent(adjustedPercentForward);
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
        int adjustedPercentForward = percent + ceil(percent * 0.04);
        right_motor.SetPercent(percent);
        left_motor.SetPercent(adjustedPercentForward);
        
        
    }
    else
    {
        int adjustedPercentForward = percent + ceil(percent * 0.08);
        right_motor.SetPercent(-percent);
        left_motor.SetPercent(-adjustedPercentForward);
        
    }

    // While the average of the left and right encoder is less than counts,
    // keep running motors
    while ((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts + countsAdjustmentDrive)
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
    while ((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts + 8 + countsAdjustmentTurn)
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
    while ((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts + countsAdjustmentTurn)
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
            if (TimeNow() - startTime > 4)
            {
                break;
            }
        }

        // break out if going backwards for 10 sec
        if (TimeNow() - startTime > 6)
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
        if (TimeNow() - startTime > 3)
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
            right_motor.SetPercent(percent+20);
        } else
        {
            left_motor.SetPercent(percent+2);
        }

        if (!frontRightBumper.Value())
        {
            right_motor.SetPercent(0);
            left_motor.SetPercent(percent+20);
        } else
        {
            right_motor.SetPercent(percent);

        }

        

        // If either bump is down, wait 2 seconds before breaking out
        if (!frontLeftBumper.Value() || !frontRightBumper.Value())
        {
            if (TimeNow() - startTime > 2)
            {
                break;
            }
        }

        // break out if going backwards for 4 sec
        if (TimeNow() - startTime > 4)
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

void bumpDriveFrontLeft(int percent)
{
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent + 2);
    // Wait for front left bumper to get hit
    float startTime = TimeNow();
    while (frontLeftBumper.Value())
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
    Sleep(3.1);

    arm.SetDegree(150);
    Sleep(0.1);
    arm.SetDegree(180);
    Sleep(0.1);

    // turn to get under lever
    turnCenter(percent, 5);
    driveDistance(percent, 4);
    turnCenter(percent, -18);

    arm.SetDegree(upDegrees);
    Sleep(0.1);
    turnCenter(25, 10);
    driveTime(-percent-20, 0.5);
    arm.SetDegree(180);
    arm.SetDegree(50);
}

void hitLeverB(int percent, int upDegrees)
{
    // Lower arm
    arm.SetDegree(180);
    Sleep(0.5);
    driveDistance(percent, -4);
    Sleep(3.1);

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
    driveDistance(percent+20, -2);
    arm.SetDegree(50);
}

void hitLeverC(int percent, int upDegrees)
{
    // Lower arm
    arm.SetDegree(180);
    Sleep(0.5);
    driveDistance(percent, -4);
    Sleep(3.1);

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
    driveDistance(percent+20, -2);
    arm.SetDegree(50);
}

void driveCompost(int percent, float inches) // using encoders
{

    float counts = countsPerInch * abs(inches);
    // Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    // Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent + 1);
    
    float startTime = TimeNow();

    // While the average of the left and right encoder is less than counts,
    // keep running motors
    while ((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts + countsAdjustmentDrive && (TimeNow() - startTime) < 5.0)
    {
    }

    //If the robot did not reach the distance in 4 seconds, turn right and go forward
    if(TimeNow() - startTime >= 4.0)
    {
        right_motor.Stop();
        left_motor.Stop();

        driveDistance(percent, -3);
        turnCenter(percent, 90);
        driveDistance(percent, 1);
        turnCenter(percent, -90);

        driveDistance(percent, 6);
    }

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

void windowDrive(int percent, float inches, char openOrClose)
{
    float counts = countsPerInch * abs(inches);
    // Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    if(openOrClose == 'o')
    {
        //Opening Backwards
        right_motor.SetPercent(-(percent)-26);
        left_motor.SetPercent(-percent);
    } else
    {
        //Closing Forwards
        right_motor.SetPercent(percent+16); //og 7
        left_motor.SetPercent(percent);
    }
        
    
    // While the average of the left and right encoder is less than counts,
    // keep running motors
    while ((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts + countsAdjustmentDrive)
        ;

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();   
}

void ERCMain() //12 Volts
{
    const int turnMotorSpeed = 25;
    const int slowMotorSpeed = 20; // Input power level here
    const int motorSpeed = 25;
    const int midMotorSpeed = 50;
    const int windowSpeed = 35;
    const int rampMotorSpeed = 70;
    const int fastMotorSpeed = 60;
    const float rampDistance = 34;
    const float tableToLeverBack = 6.3; //OG 7
    const float tableToHumidifierBack = 1.75;
    const float windowCloseDist = 9.5;
    const float startThresh = 1.5;
    const float cdsRedHighThresh = 0.60;
    const float cdsBlueHighThresh = 1.5;
    const float upDegrees = 43;        // og 43
    const float appleUpDegrees = 95;   // OG 95
    const float parallelDegrees = 150; // OG 150
    const int compostOff = 84;
    const int compostForward = 0;
    const int compostBackward = 180;

    const int windowServoClose = 162;
    const int windowServoOpen = 50;
    const int windowServoHide = 0;

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

    windowServo.SetMin(700);
    windowServo.SetMax(2200);
    windowServo.SetDegree(windowServoHide);

    Sleep(1.0);
    float cdsValue = cdsCell.Value();



    RCS.InitializeTouchMenu("0910B8VYV");

    WaitForFinalAction();
    char courseLetter = RCS.CurrentRegionLetter();





    // Wait for cds cell to read start light

    LCD.Clear();
    LCD.WriteLine("Waiting for start.");
    cdsValue = cdsCell.Value();
    boolean lightGoodTwice = false;

    
    while (!lightGoodTwice)
    {
        cdsValue = cdsCell.Value();
        if (cdsValue < startThresh)
        {
            Sleep(0.1);
            cdsValue = cdsCell.Value();
            if (cdsValue < startThresh)
            {
                lightGoodTwice = true;
            }
        }
    }

    

    // Drive into button.
    LCD.Clear();
    LCD.WriteLine("Driving");
    LCD.WriteLine(courseLetter);

    driveTime(-motorSpeed, 0.3);
    driveTime(motorSpeed, 0.3);
    

    //---Drive to compost bin---
    // Drive forward
    float compostForwardDistance = 3.5;
    float tableToWindowBackDist = 12.1;
    float appleBucketDistance = 7.5;
    float rampAngle = 62;
    float windowOpenDist = 10;
    float wallWindowForward = 6.9;
    float appleBucketPickupForward = 3.9;

    switch (courseLetter)
    {
    case 'A': //A little chopped? Ramp angle lowered. Table window lowered.
        appleBucketDistance = 6.9;
        rampAngle = 60;
        tableToWindowBackDist = 11.9;
        wallWindowForward = 7.1;
        break;
    case 'B': //Perfect
        compostForwardDistance = 3.7; //OG none
        appleBucketDistance = 7.2; //OG 6.9
        wallWindowForward = 7.1;
        break;
    case 'C': //Perfect
        compostForwardDistance = 3.2;
        appleBucketDistance = 6.8;
        rampAngle = 61;
        break;
    case 'D': //Perfect
        appleBucketDistance = 7.2;
        appleBucketPickupForward = 3.7;
        rampAngle = 63;
        break;
    case 'E': //88, ramp angle lowered
        compostForwardDistance = 4.3;
        appleBucketDistance = 6.5;
        rampAngle = 60;
        break;
    case 'F':
        appleBucketDistance = 7.2;
        break;
    case 'G':
        compostForwardDistance = 3.6;
        break;
    case 'H':
        appleBucketDistance = 7.4;
        wallWindowForward = 7.1;
        break;
    }

    driveDistance(motorSpeed, compostForwardDistance);
    

    // Turn to face compost bin, drive forward
    turnCenter(turnMotorSpeed, -44);    

    LCD.WriteLine("driving to compost");
    driveCompost(motorSpeed, 15);
    LCD.WriteLine("done");

    // Turn on motor
    compost.SetDegree(compostForward);

    // Turn wheel into compost bin
    turnCenterTime(-turnMotorSpeed, 0.7);

    // Wait 1.5 seconds, reverse motor
    Sleep(0.7);
    turnCenterTime(turnMotorSpeed, 0.1);
    compost.SetDegree(compostBackward);
    turnCenterTime(-turnMotorSpeed, 0.2);

    
    // Wait, turn off motor
    Sleep(1.4);
    compost.SetDegree(compostOff);

    //-------APPLE BUCKET---------
    // Turn to apple stump and go forward slightly
    turnCenter(turnMotorSpeed, 23 + 90);
    driveDistance(motorSpeed, 2.5); // OG 2

    // Turn to left wall to align
    turnCenter(turnMotorSpeed, -90);
    bumpDriveForward(motorSpeed);
    driveTime(fastMotorSpeed, 0.5);

    // Drive back
    driveDistance(motorSpeed, -13);

    // Drive to apple bucket
    turnCenter(turnMotorSpeed, 90);
    driveDistance(motorSpeed, appleBucketDistance);
    turnCenter(turnMotorSpeed, -90);

    // Pick up bucket
    arm.SetDegree(180);
    Sleep(0.3);
    arm.SetDegree(parallelDegrees);
    Sleep(0.3);
    arm.SetDegree(180);
    Sleep(0.3);
    arm.SetDegree(parallelDegrees);
    Sleep(0.3);
    driveDistance(motorSpeed, appleBucketPickupForward); // OG 3.5

    Sleep(0.2);
    LCD.WriteLine("raise arm");

    LCD.WriteLine("raising");
    arm.SetDegree(appleUpDegrees);
    Sleep(0.5);
    arm.SetDegree(upDegrees);
    Sleep(0.4);

    // Slightly turn and back up from tree
    turnCenter(turnMotorSpeed, 25);
    driveDistance(motorSpeed, -16.5);

    // Finish turn to ramp
    turnCenter(turnMotorSpeed, rampAngle); // OG 65

    driveDistance(rampMotorSpeed, rampDistance);

    turnCenter(turnMotorSpeed, -115);

    // Align with back wall

    arm.SetDegree(appleUpDegrees);

    bumpDriveBack(motorSpeed);
    driveDistance(motorSpeed, 2);

    // Turn to table

    turnCenter(turnMotorSpeed, 91);
    bumpDriveForward(motorSpeed);
    //driveTime(motorSpeed, 1);

    // Back up from table, drop off bucket
    arm.SetDegree(appleUpDegrees + 40);
    Sleep(0.2);
    driveDistance(motorSpeed, -5);

    // Drive into table
    Sleep(0.2);
    arm.SetDegree(upDegrees);
    Sleep(0.2);
    //driveTime(motorSpeed, 1.5);
    bumpDriveForward(motorSpeed);

    // Back up from table, drive to humidifier
    driveDistance(motorSpeed, -tableToHumidifierBack);

    // Turn to humidifier.
    LCD.Clear();
    LCD.WriteLine("Turning");
    turnCenter(turnMotorSpeed, -90);
    bumpDriveBack(motorSpeed);

    driveDistance(motorSpeed, 19);

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
            turnCenter(turnMotorSpeed, -2);
            driveDistance(motorSpeed, 0.4);
            turnCenter(turnMotorSpeed, 2);
        }

        // If to the left of the light
        if (cdsValue > rightCdsValue)
        {
            cdsValue = rightCdsValue;
            turnCenter(turnMotorSpeed, 2);
            driveDistance(motorSpeed, 0.4);
            turnCenter(turnMotorSpeed, -2);
        }
    }

    if (cdsValue > cdsBlueHighThresh)
    {
        LCD.Write("TIME OUT: GOING RED");
        cdsValue = 0.20;
        turnCenter(turnMotorSpeed, 4);
        driveDistance(motorSpeed, 0.4);
        turnCenter(turnMotorSpeed, -4);
    }

    // CHECK WHICH SIDE WIGGLE WORKED

    // Check which light
    if (cdsValue > cdsRedHighThresh) // Blue
    {
        // LCD.Clear(BLUE);
        LCD.WriteLine("Blue");
        turnCenter(turnMotorSpeed, -11);
        driveTime(37, 1.5);

        driveDistance(motorSpeed, -1);
        turnCenter(turnMotorSpeed, 11);
    }
    else // Red
    {
        // LCD.Clear(RED);
        LCD.WriteLine("Red");
        turnCenter(turnMotorSpeed, 11);
        driveTime(27, 1.2);
        driveTime(-27, 1.2);
        turnCenter(motorSpeed, 3);
    }

    // Drive to back wall.
    driveDistance(fastMotorSpeed, -12);
    bumpDriveBack(motorSpeed);



    



    // align with table
    //  Drive off wall, drive into table
    driveDistance(motorSpeed, 4.3);
    turnCenter(turnMotorSpeed, 90);
    driveTime(motorSpeed, 2);
    // back off table
    driveDistance(motorSpeed, -tableToLeverBack);
    // Turn to face levers, drive to levers
    turnCenter(turnMotorSpeed, -38);
    driveDistance(motorSpeed, 16.5);

    // Get correct lever from the RCS
    int correctLever = RCS.GetLever();

    // Check which lever to flip and perform some action
    if (correctLever == 0)
    {
        // Perform actions to flip left lever A
        turnCenter(turnMotorSpeed, -19);
        driveDistance(motorSpeed, 1);
        hitLeverA(motorSpeed, parallelDegrees);

        // Turn back and align with wall for button
        turnCenter(turnMotorSpeed, 52);
        driveDistance(fastMotorSpeed, -8);
        turnCenter(turnMotorSpeed, -62);

        bumpDriveBack(motorSpeed);
    }
    else if (correctLever == 1)
    {
        // Perform actions to flip middle lever B
        turnCenter(turnMotorSpeed, -2);
        hitLeverB(motorSpeed, parallelDegrees);

        // Turn back and align with wall for button
        driveDistance(fastMotorSpeed, -9);
        turnCenter(turnMotorSpeed, -52);

        bumpDriveBack(motorSpeed);
    }
    else if (correctLever == 2)
    {
        // Perform actions to flip right lever C
        turnCenter(turnMotorSpeed, 10);
        driveDistance(motorSpeed, 1);
        hitLeverC(motorSpeed, parallelDegrees);

        // Turn back and align with wall for button
        turnCenter(turnMotorSpeed, -8);
        driveDistance(fastMotorSpeed, -5);
        turnCenter(turnMotorSpeed, -52);

        bumpDriveBack(motorSpeed);
    }



    






    //---Drive to window---
    // Drive off wall, turn back to window
    driveDistance(motorSpeed, wallWindowForward);
    turnCenter(motorSpeed, -91);
    bumpDriveBackLeft(motorSpeed);

    // drive forward towards window
    driveDistance(slowMotorSpeed, tableToWindowBackDist);

    // Turn to window.
    LCD.Clear();
    LCD.WriteLine("Turning");
    turnCenter(motorSpeed, -85);
    //driveTime(motorSpeed, 1);
    bumpDriveForward(motorSpeed);
    driveTime(fastMotorSpeed, 0.5);
    Sleep(0.1);

    // Drive to window, open servo, open window
    driveDistance(motorSpeed, -13);
    windowServo.SetDegree(windowServoOpen);
    //turnCenter(motorSpeed, -2);
    windowDrive(windowSpeed, -windowOpenDist, 'o');

    windowServo.SetDegree(windowServoClose);
    turnAboutWheel(motorSpeed, 10, 'R');
    //turnCenter(motorSpeed, 7);
    Sleep(0.2);

    windowDrive(windowSpeed, windowCloseDist, 'c');
    Sleep(0.3);

    //Try to close window again
    if(RCS.isWindowOpen())
    {
        driveDistance(motorSpeed, -2);
        turnAboutWheel(motorSpeed, -15, 'L');
        windowDrive(windowSpeed, 8, 'c');
        Sleep(0.3);
    }
    

    // Turn to realign with back wall
    driveDistance(motorSpeed, -1);
    driveDistance(fastMotorSpeed, -2);
    windowServo.SetDegree(windowServoHide);
    turnCenter(motorSpeed, -33);

    // align with back wall for final button
    //turnCenter(motorSpeed, -180);
    driveDistance(fastMotorSpeed, 10);
    bumpDriveForward(motorSpeed);


    

    // Back off wall, drive to hit button
    driveDistance(motorSpeed, -1);

    turnCenter(motorSpeed, 83);
    driveTime(fastMotorSpeed, 3);


    //Try to hit the button again
    driveDistance(motorSpeed, -4);
    turnCenter(motorSpeed, -25);
    driveTime(fastMotorSpeed, 2);

    driveDistance(motorSpeed, -3);
    turnCenter(motorSpeed, -10);
    driveTime(fastMotorSpeed, 2);
}