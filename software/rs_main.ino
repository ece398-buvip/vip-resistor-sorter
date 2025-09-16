#include <Servo.h>

// X stepper motor pins
#define X_DIR_PIN 5  // 2 originally
#define X_STEP_PIN 6 // pwm  3 originally

// Y stepper motor pins
#define Y_DIR_PIN 9
#define Y_STEP_PIN 10 // pwm

// Feeder stepper motor pins
// #define FEEDER_DIR_PIN  7
// #define FEEDER_STEP_PIN 6       //pwm

// X limit switches
#define XLIMIT_SWITCH_PINLEFT 2 // Define the limit switch pin originally 8
#define XLIMIT_SWITCH_PINRIGHT 11

// Y limit switches
#define YLIMIT_SWITCH_TOP 13
#define YLIMIT_SWITCH_BOT 8

// Prox Sensor inputs
#define FEEDER_PROX A0
#define GANTRY_PROX A1

// Gantry Servos pins
// #define FORKS_SERVO_PWM 9       //pwm
// #define SLIDE_SERVO_PWM 10      //pwm

// Voltage Division Pins
#define V_Div_Output A2
// #define V_Div_Supply A3     //This will connect to the voltage supply to use in calculations. Wont be exactly 5V.

// Magnet Relay
#define MAGNET_RELAY A4

const int STEPS_PER_INCH = 172; // Steps per inch for both X and Y axes
const int V_Div_Supply = A3;

int t_flag = 0;
int current_x = 0, current_y = 0; // Start at origin (0,0)

Servo SlideServo; // Creates Servo Objects
Servo ForksServo;

const float R1 = 10000.0;
const float R2 = 1000.0;

// HIGH is Clockwise

void xstepping_seq_fast()
{
    digitalWrite(X_STEP_PIN, HIGH);
    delayMicroseconds(800);
    digitalWrite(X_STEP_PIN, LOW);
    delayMicroseconds(800);
}

void xstepping_seq_medium()
{
    digitalWrite(X_STEP_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(X_STEP_PIN, LOW);
    delayMicroseconds(1000);
}

void xstepping_seq_slow()
{
    digitalWrite(X_STEP_PIN, HIGH);
    delayMicroseconds(2500);
    digitalWrite(X_STEP_PIN, LOW);
    delayMicroseconds(2500);
}

void ystepping_seq_fast()
{
    digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(800);
    digitalWrite(Y_STEP_PIN, LOW);
    delayMicroseconds(800);
}

void ystepping_seq_medium()
{
    digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(Y_STEP_PIN, LOW);
    delayMicroseconds(1000);
}

void ystepping_seq_slow()
{
    digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(2500);
    digitalWrite(Y_STEP_PIN, LOW);
    delayMicroseconds(2500);
}

void AutoHome()
{                                                 // Function to home gantry, X axis first
    pinMode(XLIMIT_SWITCH_PINLEFT, INPUT_PULLUP); // Making

    digitalWrite(X_DIR_PIN, HIGH);
    Serial.println("Beginning auto home for X-Axis...");

    while (digitalRead(XLIMIT_SWITCH_PINLEFT) == 1) // waits for limit switch to be pressed (held low)
    {
        xstepping_seq_fast();
    }

    digitalWrite(X_DIR_PIN, LOW); // throw in reverse

    for (int i = 0; i <= 150; i++)
    {
        xstepping_seq_medium();
    }

    digitalWrite(X_DIR_PIN, HIGH);                  // reset direction
    while (digitalRead(XLIMIT_SWITCH_PINLEFT) == 1) // now marches back waiting for same condition as first.
    {
        xstepping_seq_slow();
    }

    Serial.println("Finished!");
    digitalWrite(X_DIR_PIN, LOW);

    // This is for moving it to bin 0
    // move to true 0,0 function
    for (int i = 0; i <= 690; i++) // move X to 1,0 box this is arbetrary 172 steps per inch. (off of true zero)
    {
        digitalWrite(X_STEP_PIN, HIGH);
        delayMicroseconds(500);
        digitalWrite(X_STEP_PIN, LOW);
        delayMicroseconds(500);
    }

    Serial.println("moving y in 500 ms");
    delay(500);

    /////////////////////Homing Y Axis///////////////////

    digitalWrite(Y_DIR_PIN, HIGH); // throw in reverse

    while (digitalRead(YLIMIT_SWITCH_BOT) == 1)
    {
        ystepping_seq_fast();
    }

    digitalWrite(Y_DIR_PIN, LOW);

    for (int i = 0; i <= 150; i++)
    {
        ystepping_seq_medium();
    }

    Serial.println("Finished!");
}

// Function to move X stepper motor with limit switch check
void moveStepper1(int stepPin, int dirPin, int steps, bool direction)
{
    digitalWrite(dirPin, direction ? LOW : HIGH); // Set direction

    for (int i = 0; i < abs(steps); i++)
    {
        if (digitalRead(XLIMIT_SWITCH_PINLEFT) == LOW)
        { // Check if the switch is engaged
            Serial.println("Limit switch activated! Stopping X movement.");
            return; // Stop movement immediately
        }

        digitalWrite(stepPin, HIGH);
        delayMicroseconds(800); // Adjust speed
        digitalWrite(stepPin, LOW);
        delayMicroseconds(800);
    }
}

// Function to move Y stepper motor
void moveStepper2(int stepPin, int dirPin, int steps, bool direction)
{
    digitalWrite(dirPin, direction ? HIGH : HIGH); // Set direction (inverted Y)

    for (int i = 0; i < abs(steps); i++)
    {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(200); // Adjust speed
        digitalWrite(stepPin, LOW);
        delayMicroseconds(200);
    }
}

// Function for gantry sequence

void start_gantry_seq()
{
    // Move forward
    SlideServo.writeMicroseconds(2000); // forward full speed
    delay(2000);                        // adjust time for distance
    SlideServo.writeMicroseconds(1500); // stop servo

    // Turn on magnet
    digitalWrite(MAGNET_RELAY, HIGH);
    delay(300);

    // Move back until prox is triggered
    while (digitalRead(GANTRY_PROX) == LOW)
    {
        SlideServo.writeMicroseconds(1000); // reverse
    }
    SlideServo.writeMicroseconds(1500); // stop

    // Open forks
    ForksServo.write(0); // open claws
    delay(300);

    // Close forks
    ForksServo.write(90); // close claws
    delay(300);
}

// cont servo moves forward x ms
// magnet relay is turned on
// cont servo moves backward until prox sensor is high
// sgservo is 90deg
// small delay
// sgservo is 0deg
// cont servo is high for x seconds
// cont servo is low until prox is high

// Add function that takes inches and moves the stepper
// Handle target-current here in units of steps

// Function to move to a specific position (inches)
// Units should be in inches
void moveTo(float target_x, float target_y)
{
    // Only deal in inches from here
    int x_steps, y_steps;

    // x_inches = box_index_x * INCHES_PER_BOX_WIDTH
    // y_inches = box_index_y * INCHES_PER_BOX_HEIGHT
    // if (box_index_x > 7)
    // then add edge offset to x (x += <some offset here>)

    if (target_x >= 0 && target_x <= 7 && target_y >= 0 && target_y < 6)
    {
        Serial.print("current x: ");
        Serial.println(current_x);
        if ((current_x >= 8 && current_x <= 15))
        {
            // digitalWrite(X_DIR_PIN, LOW);
            x_steps = ((target_x - current_x) * 2.29 * STEPS_PER_INCH - (1.32 * STEPS_PER_INCH) - 35);
            y_steps = (target_y - current_y) * 2.7 * STEPS_PER_INCH;
            t_flag = 0;
            Serial.print("X Steps back over: ");
            Serial.println(x_steps);
            Serial.print("flag = ");
            Serial.println(t_flag);
        }
        else
        {
            x_steps = (target_x - current_x) * 2.29 * STEPS_PER_INCH;
            y_steps = (target_y - current_y) * 2.7 * STEPS_PER_INCH;
            Serial.print("X Steps before threshold: ");
            Serial.println(x_steps);
        }
    }
    else if (target_x > 7 && target_x <= 15 && target_y >= 0 && target_y <= 6 && t_flag == 0)
    {
        x_steps = (target_x - current_x) * 2.29 * STEPS_PER_INCH + (1.32 * STEPS_PER_INCH) + 35;
        y_steps = (target_y - current_y) * 2.7 * STEPS_PER_INCH;
        t_flag = 1;
        Serial.print("X Steps (crossing threshold): ");
        Serial.println(x_steps);
        Serial.print("flag = ");
        Serial.println(t_flag);

        /*    if (target_x > 7 && target_x<=15  && target_y >= 0 && target_y <= 6 && t_flag == 1) {// need this
            x_steps = (target_x - current_x) * 2.29 * STEPS_PER_INCH + (1.32 * STEPS_PER_INCH);
            y_steps = (target_y - current_y) * 2.7 * STEPS_PER_INCH;
            Serial.print("X Steps (over threshold) 1 loop: ");
            Serial.println(x_steps);
        } */
    }
    else if (target_x >= 8 && target_x <= 15 && target_y >= 0 && target_y <= 6)
    {
        x_steps = (target_x - current_x) * 2.29 * STEPS_PER_INCH;
        y_steps = (target_y - current_y) * 2.7 * STEPS_PER_INCH;
        // Serial.print("X Steps back over: ");
        // Serial.println(x_steps);
        Serial.print("X Steps (over threshold)2 loop: ");
        Serial.println(x_steps);
    }
    else if (current_x > 8 && current_x <= 15 && target_x >= 0 && target_x <= 7)
    {
        x_steps = (target_x - current_x) * 2.29 * STEPS_PER_INCH + (1.32 * STEPS_PER_INCH);
        y_steps = (target_y - current_y) * 2.7 * STEPS_PER_INCH;
        // Serial.print("X Steps back over: ");
        // Serial.println(x_steps); not here
    }

    else if ((current_x > 8 && current_x <= 15) && (target_x < 8 && target_x >= 0) && (target_y >= 0 && target_y <= 6))
    {
        x_steps = (target_x - current_x) * 2.29 * STEPS_PER_INCH + (1.32 * STEPS_PER_INCH) + 40;
        y_steps = (target_y - current_y) * 2.7 * STEPS_PER_INCH;
        // Serial.print("X Steps back over: ");
        // Serial.println(x_steps);   // not here
    }
    else
    {
        Serial.println("Error: Target position out of range!");
        return; // Stop execution if the position is invalid
    }

    // Move X-axis
    if (x_steps != 0)
    {
        moveStepper1(X_STEP_PIN, X_DIR_PIN, x_steps, x_steps > 0);
        current_x = target_x;
    }

    // Move Y-axis
    if (y_steps != 0)
    {
        moveStepper2(Y_STEP_PIN, Y_DIR_PIN, y_steps, y_steps > 0);
        current_y = target_y;
    }
}

// Function to measure resistance
float measureResistance()
{
    // Read voltage divider output (across R2)
    int adcOutput = analogRead(V_Div_Output);

    // Convert ADC reading to volts using 2.56V reference
    float Vout = (adcOutput / 1023.0) * 2.56;

    // Supply to divider is actual Vcc (5V rail)
    float Vsupply = 5.0; // or measure with the bandgap trick if you want accuracy

    // Sanity check
    if (Vout <= 0 || Vout >= Vsupply)
    {
        return -1; // Out of range (open or short)
    }

    // Solve for unknown resistor (R2)
    float R_unknown = R1 * (Vout / (Vsupply - Vout));

    return R_unknown;
}

struct ResistorBin
{
    int value;
    int tolerance;
    int x;
    int y;
};

ResistorBin bins[] = {
    {1000, 50, 2, 3},  // 1000 ohms ±50
    {2200, 100, 4, 1}, // 2200 ohms ±100
    {4700, 200, 5, 6}, // 4700 ohms ±200
    {330, 10, 0, 0}    // 330 ohms ±10
};

const int NUM_BINS = sizeof(bins) / sizeof(bins[0]);

int findBin(float measuredValue)
{
    for (int i = 0; i < NUM_BINS; i++)
    {
        if (measuredValue >= bins[i].value - bins[i].tolerance &&
            measuredValue <= bins[i].value + bins[i].tolerance)
        {
            return i; // return index of matching bin
        }
    }
    return -1; // no matching bin
}

void setup()
{

    pinMode(X_STEP_PIN, OUTPUT);
    pinMode(X_DIR_PIN, OUTPUT);
    pinMode(Y_STEP_PIN, OUTPUT);
    pinMode(Y_DIR_PIN, OUTPUT);
    pinMode(XLIMIT_SWITCH_PINLEFT, INPUT_PULLUP); // Set limit switch as input with pull-up
    pinMode(YLIMIT_SWITCH_TOP, INPUT_PULLUP);
    pinMode(YLIMIT_SWITCH_BOT, INPUT_PULLUP);
    // SlideServo.attach(SLIDE_SERVO_PWM);
    // ForksServo.attach(FORKS_SERVO_PWM);

    // analogReference(INTERNAL1V1); // Use internal 1.1V reference

    Serial.begin(9600);
    AutoHome();
}

///////////////////Loop/////////////////////////
void loop()
{
    float measuredValue = measureResistance(); // your voltage divider function
/*
    if (measuredValue != -1) {
    int binIndex = findBin(measuredValue);

    if (binIndex != -1) {
        Serial.print("Resistor ");
        Serial.print(measuredValue);
        Serial.print(" ohms → Bin (");
        Serial.print(bins[binIndex].x);
        Serial.print(", ");
        Serial.print(bins[binIndex].y);
        Serial.println(")");

        // Move gantry & drop resistor
        moveTo(bins[binIndex].x, bins[binIndex].y);
        start_gantry_seq();
    } else {
        Serial.print("No matching bin found for ");
        Serial.print(measuredValue);
        Serial.println(" ohms.");
    }
    } else {
    Serial.println("Measured value out of range.");
    }

    delay(1000); // for testing


*/

    // Read serial input for target position
    if (Serial.available())
    {
        float target_x = Serial.parseFloat(); // Read floating-point input for inches
        float target_y = Serial.parseFloat(); // Read floating-point input for inches

        if (Serial.read() == '\n')
        { // Ensure full input received
            Serial.print("Moving to: ");
            Serial.print(target_x);
            Serial.print(", ");
            Serial.println(target_y);
            Serial.println(digitalRead(Y_DIR_PIN));

            moveTo(target_x, target_y);

            Serial.println("Movement complete.");
        }
    }

    delay(500); // Small delay for stability
}