#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRCS.h>
#include <FEHSD.h>

// RCS Delay time
#define RCS_WAIT_TIME_IN_SEC 0.35

// Shaft encoding counts for CrayolaBots
#define COUNTS_PER_INCH 40.5
#define COUNTS_PER_DEGREE 2.48

// Defines for pulsing the robot
#define PULSE_TIME 0.15
#define PULSE_POWER 20

// Define for the motor power
#define POWER 25

// Orientation of AruCo Code
#define PLUS 0
#define MINUS 1

// Declarations for encoders & motors
DigitalEncoder right_encoder(FEHIO::Pin8);
DigitalEncoder left_encoder(FEHIO::Pin9);
FEHMotor right_motor(FEHMotor::Motor0, 9.0);
FEHMotor left_motor(FEHMotor::Motor1, 9.0);

/*
 * Pulse forward a short distance using time
 */
void pulse_forward(int percent, float seconds)
{
    // Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);

    // Wait for the correct number of seconds
    Sleep(seconds);

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

/*
 * Pulse counterclockwise a short distance using time
 */
void pulse_counterclockwise(int percent, float seconds)
{
    // Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(-percent);

    // Wait for the correct number of seconds
    Sleep(seconds);

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

/*
 * Move forward using shaft encoders where percent is the motor percent and counts is the distance to travel
 */
void move_forward(int percent, int counts) // using encoders
{
    // Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    // Set both motors to desired percent
    left_motor.SetPercent(percent);
    right_motor.SetPercent(percent);

    // While the average of the left and right encoder are less than counts,
    // keep running motors
    while ((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts)
        ;

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

/*
 * Turn counterclockwise using shaft encoders where percent is the motor percent and counts is the distance to travel
 */
void turn_counterclockwise(int percent, int counts)
{
    // Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    // Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(-percent);

    // While the average of the left and right encoder are less than counts,
    // keep running motors
    while ((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts)
        ;

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

/*
 * Use RCS to move to the desired x_coordinate based on the orientation of the AruCo code
 */
void check_x(float x_coordinate, int orientation)
{
    // Determine the direction of the motors based on the orientation of the AruCo code
    int power = PULSE_POWER;
    if (orientation == MINUS)
    {
        power = -PULSE_POWER;
    }

    RCSPose *pose = RCS.RequestPosition();

    // Check if receiving proper RCS coordinates and whether the robot is within an acceptable range
    for (int i = 0; i < 10; i++)
    {
        if (pose->x >= 0 && (pose->x < x_coordinate - 1 || pose->x > x_coordinate + 1))
        {
            if (pose->x > x_coordinate)
            {
                // Pulse the motors for a short duration in the correct direction
                pulse_forward(-power, PULSE_TIME);
            }
            else if (pose->x < x_coordinate)
            {
                // Pulse the motors for a short duration in the correct direction
                pulse_forward(power, PULSE_TIME);
            }
            Sleep(RCS_WAIT_TIME_IN_SEC);

            pose = RCS.RequestPosition();
        }
    }
}

/*
 * Use RCS to move to the desired y_coordinate based on the orientation of the QR code
 */
void check_y(float y_coordinate, int orientation)
{
    // Determine the direction of the motors based on the orientation of the QR code
    int power = PULSE_POWER;
    if (orientation == MINUS)
    {
        power = -PULSE_POWER;
    }

    RCSPose *pose = RCS.RequestPosition();

    int yTarget = y_coordinate;
    LCD.WriteLine("Going To: " + yTarget);

    // Check if receiving proper RCS coordinates and whether the robot is within an acceptable range
    for (int i = 0; i < 10; i++)
    {
        while (pose->y >= 0 && (pose->y < y_coordinate - 1 || pose->y > y_coordinate + 1))
        {
            int currentY = pose->y;
            LCD.WriteLine("Current y: " + currentY);
            if (pose->y > y_coordinate)
            {
                // Pulse the motors for a short duration in the correct direction
                pulse_forward(-power, PULSE_TIME);
            }
            else if (pose->y < y_coordinate)
            {
                // Pulse the motors for a short duration in the correct direction
                pulse_forward(power, PULSE_TIME);
            }
            Sleep(RCS_WAIT_TIME_IN_SEC);

            pose = RCS.RequestPosition();
        }
    }
}

/*
 * Use RCS to move to the desired heading
 */
void check_heading(float heading)
{
    // You will need to fill out this one yourself and take into account
    // checking for proper RCS data and the edge conditions
    //(when you want the robot to go to 0 degrees or close to 0 degrees)

    /*
        SUGGESTED ALGORITHM:
        1. Check the current orientation of the QR code and the desired orientation of the QR code
        2. Check if the robot is within the desired threshold for the heading based on the orientation
        3. Pulse in the correct direction based on the orientation
    */

    // Determine the direction of the motors based on the orientation of the QR code
    RCSPose *pose = RCS.RequestPosition();

    // Check if receiving proper RCS heading and whether the robot is within an acceptable range
    for (int i = 0; i < 10; i++)
    {
        while (pose->heading >= 0 && (pose->heading < heading - 3 || pose->heading > heading + 3))
        {
            if (pose->heading > heading)
            {
                pulse_counterclockwise(-PULSE_POWER, PULSE_TIME);
            }
            else if (pose->heading < heading)
            {
                pulse_counterclockwise(PULSE_POWER, PULSE_TIME);
            }
            Sleep(RCS_WAIT_TIME_IN_SEC);

            pose = RCS.RequestPosition();
        }
    }
}

void ERCMain()
{
    int touch_x, touch_y;
    float A_x, A_y, B_x, B_y, C_x, C_y, D_x, D_y;
    float A_heading, B_heading, C_heading, D_heading;
    int B_C_counts, C_D_counts, turn_90_counts;

    RCS.InitializeTouchMenu("Z1TESTING");

    LCD.WriteLine("RCS & Data Logging Test");
    LCD.WriteLine("Press Screen To Start");
    while (!LCD.Touch(&touch_x, &touch_y))
        ;
    while (LCD.Touch(&touch_x, &touch_y))
        ;

    // COMPLETE CODE HERE TO READ SD CARD FOR LOGGED X AND Y DATA POINTS
    A_x = 25.6;
    A_y = 40.19;
    B_x = 27.77;
    B_y = 55.54;
    C_x = 10;
    C_y = 55.35;
    D_x = 10.59;
    D_y = 42.9;

    // WRITE CODE HERE TO SET THE HEADING DEGREES AND COUNTS VALUES
    A_heading = 90;
    B_heading = 180;
    C_heading = 270;
    D_heading = 356;

    B_C_counts = 10 * COUNTS_PER_INCH;
    C_D_counts = 10 * COUNTS_PER_INCH;

    turn_90_counts = 90 * COUNTS_PER_DEGREE;

    // A --> B
    LCD.Clear();
    LCD.WriteLine("Pulsing");
    check_y(B_y, PLUS);
    Sleep(1.0);
    LCD.Clear();
    LCD.WriteLine("Turning");
    check_heading(B_heading);

    // B --> C
    LCD.Clear();
    LCD.WriteLine("Moving Forward");
    move_forward(POWER, B_C_counts);
    LCD.Clear();
    LCD.WriteLine("Pulsing");
    check_x(C_x, MINUS);
    LCD.Clear();
    LCD.WriteLine("Turning");
    turn_counterclockwise(POWER, turn_90_counts);
    check_heading(C_heading);

    // C --> D
    move_forward(POWER, C_D_counts);
    check_y(D_y, MINUS);
    turn_counterclockwise(POWER, turn_90_counts);
    check_heading(D_heading);
}