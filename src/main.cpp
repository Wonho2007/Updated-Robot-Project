#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>

// Declarations for encoders & motors
DigitalEncoder right_encoder(FEHIO::Pin8);
DigitalEncoder left_encoder(FEHIO::Pin9);
FEHMotor right_motor(FEHMotor::Motor0, 9.0);
FEHMotor left_motor(FEHMotor::Motor1, 9.0);

enum
{
    MIDDLE,
    LEFT,
    RIGHT
};

void move_forward(int percent, int counts) // using encoders
{
    // Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    // Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(-percent);

    // While the average of the left and right encoder is less than counts,
    // keep running motors
    while ((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts)
        ;

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

void turn_right(int percent, int counts) // using encoders

{

    // Reset encoder counts

    right_encoder.ResetCounts();

    left_encoder.ResetCounts();

    // Set both motors to desired percent
    right_motor.SetPercent(-percent);
    left_motor.SetPercent(-percent);
    // hint: set right motor backwards, left motor forwards

    //<ADD CODE HERE>

    // While the average of the left and right encoder is less than counts,
    while ((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts)
        ;
    // keep running motors

    //<ADD CODE HERE>

    // Turn off motors

    right_motor.Stop();

    left_motor.Stop();
}

void turn_left(int percent, int counts) // using encoders

{

    // Reset encoder counts

    right_encoder.ResetCounts();

    left_encoder.ResetCounts();

    // Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);
    // hint: set right motor backwards, left motor forwards

    //<ADD CODE HERE>

    // While the average of the left and right encoder is less than counts,
    while ((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts)
        ;
    // keep running motors

    //<ADD CODE HERE>

    // Turn off motors

    right_motor.Stop();

    left_motor.Stop();
}

void ERCMain()
{
    int motor_percent = 25;             // Input power level here
    int expected_counts_fourteen = 567; // Input theoretical counts here
    int expected_counts_ten = 405;
    int expected_counts_four = 162;
    int expected_counts_right_turn = 239;
    int expected_counts_left_turn = 239;

    int x, y; // for touch screen

    // Initialize the screen
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    LCD.WriteLine("Shaft Encoder Exploration Test");
    LCD.WriteLine("Touch the screen");
    while (!LCD.Touch(&x, &y))
        ; // Wait for screen to be pressed
    while (LCD.Touch(&x, &y))
        ; // Wait for screen to be unpressed

    move_forward(motor_percent, expected_counts_fourteen);
    Sleep(2.0);
    turn_left(motor_percent, expected_counts_left_turn);
    Sleep(2.0);
    move_forward(motor_percent, expected_counts_ten);
    Sleep(2.0);
    turn_right(motor_percent, expected_counts_right_turn);
    Sleep(2.0);
    move_forward(motor_percent, expected_counts_four);

    Sleep(2.0); // Wait for counts to stabilize
}

void lineFolowing()
{

    FEHMotor rightDrive(FEHMotor::Motor0, 9.0);
    FEHMotor leftDrive(FEHMotor::Motor1, 9.0);
    DigitalInputPin backRightBumper(FEHIO::Pin0);
    AnalogInputPin rightOpt(FEHIO::Pin8);
    AnalogInputPin middleOpt(FEHIO::Pin9);
    AnalogInputPin leftOpt(FEHIO::Pin10);

    int state = MIDDLE;

    while (backRightBumper.Value())
        ;

    while (true)
    { // I will follow this line forever!

        switch (state)
        {

            // If I am in the middle of the line...

        case MIDDLE:

            // Set motor powers for driving straight

            rightDrive.SetPercent(25);
            leftDrive.SetPercent(25);

            if (rightOpt.Value() > 3)
            {
                state = RIGHT; // update a new state
            }

            /* Code for if left sensor is on the line */
            if (leftOpt.Value() > 2.5)
            {
                state = LEFT; // update a new state
            }

            break;

            // If the right sensor is on the line...

        case RIGHT:

            // Set motor powers for right turn

            rightDrive.SetPercent(10);
            leftDrive.SetPercent(25);

            if (rightOpt.Value() < 2)
            {
                state = MIDDLE;
            }

            break;

            // If the left sensor is on the line...

        case LEFT:

            /* Mirror operation of RIGHT state */
            rightDrive.SetPercent(25);
            leftDrive.SetPercent(10);

            if (leftOpt.Value() < 2)
            {
                state = MIDDLE;
            }

            break;

        default: // Error. Something is very wrong.

            break;
        }

        // Sleep a bit
    }
}

void explorationOne()
{
    FEHMotor rightDrive(FEHMotor::Motor0, 9.0);
    FEHMotor leftDrive(FEHMotor::Motor1, 9.0);
    DigitalInputPin backLeftBumper(FEHIO::Pin8);
    DigitalInputPin backRightBumper(FEHIO::Pin1);
    DigitalInputPin frontLeftBumper(FEHIO::Pin14);
    DigitalInputPin frontRightBumper(FEHIO::Pin3);

    // Wait for input
    while (backLeftBumper.Value())
        ;

    Sleep(1.);

    rightDrive.SetPercent(25);
    leftDrive.SetPercent(25);

    // Wait for front bumpers to get hit
    while (frontLeftBumper.Value() || frontRightBumper.Value())
        ;

    // Turn right
    leftDrive.SetPercent(0);
    rightDrive.SetPercent(-25);

    // Wait for back bumper to hit wal
    while (backRightBumper.Value())
        ;

    leftDrive.SetPercent(0);
    rightDrive.SetPercent(0);

    Sleep(1.);

    leftDrive.SetPercent(-25);
    rightDrive.SetPercent(0);

    while (backLeftBumper.Value())
        ;

    // Drive forward
    leftDrive.SetPercent(25);
    rightDrive.SetPercent(25);

    // PART 2

    // Wait for front bumpers to get hit
    while (frontLeftBumper.Value() || frontRightBumper.Value())
        ;

    // Stop
    leftDrive.SetPercent(0);
    rightDrive.SetPercent(0);
    Sleep(0.5);

    // Turn back left
    leftDrive.SetPercent(-25);
    rightDrive.SetPercent(0);

    // Wait for back bumper to hit wal
    while (backLeftBumper.Value())
        ;

    // Align with wall
    leftDrive.SetPercent(0);
    rightDrive.SetPercent(0);
    Sleep(0.5);

    leftDrive.SetPercent(0);
    rightDrive.SetPercent(-25);

    while (backRightBumper.Value())
        ;

    // Stop
    leftDrive.SetPercent(0);
    rightDrive.SetPercent(0);
    Sleep(0.5);

    // forward
    rightDrive.SetPercent(25);
    leftDrive.SetPercent(25);

    // Wait for front bumpers to get hit
    while (frontLeftBumper.Value() || frontRightBumper.Value())
        ;
    // Stop
    leftDrive.SetPercent(0);
    rightDrive.SetPercent(0);
    Sleep(0.5);

    leftDrive.SetPercent(-25);
    rightDrive.SetPercent(-25);

    Sleep(1.0);

    leftDrive.SetPercent(0);
    rightDrive.SetPercent(0);

    Sleep(1.0);
    leftDrive.SetPercent(25);
    rightDrive.SetPercent(-25);
}