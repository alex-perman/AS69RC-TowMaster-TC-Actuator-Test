/*
 *  
 *  AS69RC TowMaster TC -- Clutch Test Actuator Firmware
 *  BD Diesel
 *  Alex Perman, 2025
 * 
 */


// Include Libraries
#include <Arduino.h>
//#include <LinearActuator.h>           //I'll do it myself LMAO
#include <HX711.h>
#include <PID_v1.h>
#include <math.h>

// Definitions
#define TARGET_LOAD 30                  // SET THIS AS YOUR TARGET LOAD

#define DIR_PIN 7                       // DIRECTION PIN ON MCU
#define PWM_PIN 5                       // PWM DRIVE PIN ON MCU
#define FEEDBACK_PIN 6                  // POT FEEDBACK PIN 
#define SLEEP_PIN 4                     // MOTOR DRIVER SLEEP PIN

#define LOADCELL_DOUT_PIN 8             // LOAD CELL DOUT PIN ON MCU
#define LOADCELL_SCK_PIN 3              // LOAD CELL CLOCK PIN ON MCU

// Load Cell Calibration 
bool scaleCal = 0;                      // Set to TRUE for calibration program.
float calibration_factor = 14750;       // Adjusted for the load cell in question.
float units;
float pounds;

// PID Vars
double Setpoint, Input, Output;
double Kp=20.0, Ki=0.0, Kd=0.5;         // SET KP KI KD VALUES HERE

double rampedSetpoint = Setpoint;       // use this instead of jumping Setpoint directly -- avoid current spikes!
const double SETPOINT_STEP = 10;         // units per ramp interval
const unsigned long SETPOINT_RAMP_MS = 50; // interval to apply step
unsigned long lastRampMs = 0;

PID myPID(&Input, &Output, &rampedSetpoint, Kp, Ki, Kd, DIRECT);

// Create Actuator Instance
//LinearActuator actuator(EXTEND_PIN, RETRACT_PIN, FEEDBACK_PIN);
HX711 loadCell;
bool actTest = 0;                       // Set to TRUE for actuator steady state testing / driver troubleshooting (see actTestLoop() fcn)

// Timing
const int maxPWMForward = 255;          // Set forward PWM limit
const int maxPWMReverse = 255;          // Set reverse PWM limit
bool holdFlag = false;
bool forwardFlag = false;
unsigned long holdStart = 0;
unsigned long holdDuration = 2000;      // hold for Xe-3 seconds
bool loading = true;
int cycleCount = 0;

void scaleCalSetup() {
    Serial.println("HX711 calibration sketch");
    Serial.println("Remove all weight from scale");
    Serial.println("After readings begin, place known weight on scale");
    Serial.println("Press + or a to increase calibration factor");
    Serial.println("Press - or z to decrease calibration factor");

    loadCell.set_scale();
    loadCell.tare();                    //Reset the scale to 0

    long zero_factor = loadCell.read_average(); //Get a baseline reading
    Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
    Serial.println(zero_factor);
}

void scaleCalLoop() {
    loadCell.set_scale(calibration_factor); //Adjust to this calibration factor

    Serial.print("Reading: ");
    units = loadCell.get_units(), 10;
    if (units < 0)
    {
        units = 0.00;
    }
    pounds = units * 2.20462;
    Serial.print(pounds);
    Serial.print(" lbs"); 
    Serial.print(" calibration_factor: ");
    Serial.print(calibration_factor);
    Serial.println();

    if(Serial.available())
    {
        char temp = Serial.read();
        if(temp == '+' || temp == 'a')
        calibration_factor += 1;
        else if(temp == '-' || temp == 'z')
        calibration_factor -= 1;
    }
}

void actTestLoop() {
    Serial.println("EXTEND");
    digitalWrite(DIR_PIN, HIGH); //actuator.extend();
    analogWrite(PWM_PIN, 220);
    delay(4000);
    Serial.println("Wait");
    digitalWrite(DIR_PIN, HIGH);
    analogWrite(PWM_PIN, 0);
    delay(4000);
    Serial.println("Retract");
    digitalWrite(DIR_PIN, LOW);
    analogWrite(PWM_PIN, 220);
    delay(4000);
    Serial.println("Wait");
    digitalWrite(DIR_PIN, HIGH);
    analogWrite(PWM_PIN, 0);
    delay(4000);
}

void rampSetpointTo(double target) {
    if (millis() - lastRampMs < SETPOINT_RAMP_MS) return;
    lastRampMs = millis();
    if (abs(rampedSetpoint - target) <= SETPOINT_STEP) {
        rampedSetpoint = target;
    } else if (rampedSetpoint < target) {
        rampedSetpoint += SETPOINT_STEP;
    } else {
        rampedSetpoint -= SETPOINT_STEP;
    }
}

void mainLoop() {
    // Set desired load to 500 lbs, PID to actuator 
    // HOLD for X seconds
    // Set desired load to 0 lbs, PID to actuator
    // HOLD for Y seconds
    Input = loadCell.get_units(), 10;  // Smoothed average of 5 samples

    rampSetpointTo(Setpoint);
    myPID.Compute();

    //float ctrl = Setpoint - Input;

    Serial.print("Setpoint:");
    Serial.print(Setpoint);
    Serial.print(",");          // Delim
    Serial.print("Load:");
    Serial.print(Input);
    Serial.print(",");
    Serial.print("PID_Output:");
    Serial.println(Output);

    // Actuator control logic
    double error = abs(Setpoint - Input);

    if (abs(Output) > 5) {  // tolerance band of ±5 lbs
        if ((Output) > 0) {
            forwardFlag = true;
            digitalWrite(DIR_PIN, HIGH); //actuator.extend();
            if (abs(Output) < maxPWMForward) { analogWrite(PWM_PIN, abs(Output)); }
            else { analogWrite(PWM_PIN, maxPWMForward); }
        }
        else {
            digitalWrite(DIR_PIN, LOW); //actuator.retract();
            if (abs(Output) < maxPWMReverse) { analogWrite(PWM_PIN, abs(Output)); }
            else { analogWrite(PWM_PIN, maxPWMReverse); }
        }

    } else {
        digitalWrite(DIR_PIN, HIGH);
        analogWrite(PWM_PIN, 0); //actuator.stop();
        delay(50);
    }

    // Hold duration logic
    if (error < 1) {
        if (!holdFlag) {
            holdStart = millis();
            holdFlag = true;
        }

        if ((millis() - holdStart >= holdDuration)) {
            loading = !loading;
            Setpoint = loading ? TARGET_LOAD : 0;  // Toggle between target and 0 lbs
            if (loading) {
                cycleCount++;
                Serial.print("CycleCount: ");
                Serial.println(cycleCount);
            }
            holdFlag = false;
            Serial.print("Switched setpoint to: ");
            Serial.println(Setpoint);
        }
    }
    else {
        holdFlag = false; 
    }
    
    delay(10);
}


void setup() {
    Serial.begin(115200);
    //actuator.begin();
    pinMode(DIR_PIN, OUTPUT);
    pinMode(PWM_PIN, OUTPUT);
    pinMode(FEEDBACK_PIN, INPUT);
    pinMode(SLEEP_PIN, OUTPUT);
    loadCell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

    if (scaleCal) {
        scaleCalSetup();
    }
    else {
        // Make sure actuator is at position with zero load
        loadCell.set_scale(calibration_factor);
        loadCell.tare();      // want absolute??

        Setpoint = TARGET_LOAD; // Initial target load (lbs)
        myPID.SetMode(AUTOMATIC);
        myPID.SetOutputLimits(-maxPWMReverse, maxPWMForward);  // Full reverse to full forward

        digitalWrite(SLEEP_PIN, LOW);           // RESET DRIVER
        delay(10);
        digitalWrite(SLEEP_PIN, HIGH);

        Serial.println("System Initialized");
    }
    
}

void loop() {
    if (scaleCal) {
        scaleCalLoop();
    }
    else if (actTest) {
        actTestLoop();
    }
    else {
        mainLoop();
    }

}

