 // NASA Rover V2 with Enhanced Obstacle Avoidance and Machine Learning

#include <Wire.h> // Talk to sensors using two magic wires (I2C)
#include <MLX90393.h> // Compass (a.k.a. Magnetometer) to figure out which way is North-ish
#include <TinyGPS++.h> // GPS for figuring out where in the world we are
#include <NewPing.h>  // Ultrasonic Sensors for not bumping into things (like robot eyes)
#include <Stepper.h>  // Stepper Motors for moving the arm precisely
#include <ArduinoTensorFlowLite.h> // Tiny bit of AI magic to help our rover make smarter choices

// Defining constants to keep the rover on track, speed limits, and avoid collisions
#define TOLERANCE_RADIUS 1.0 // How close is "close enough" to a waypoint
#define MAX_HEADING_ANGLE 180 // Maximum angle that means we need to do a U-turn
#define MIN_HEADING_ANGLE 5 // If we're off by less than this, just go with it
#define ANGLE_RANGE_DIV 0.25
#define MOTOR_VEL 255 // Full speed
#define PIVOT_WHEEL_VEL 50 // Speed for turning on a dime
#define PIVOT_WHEEL_VEL_SLOW 150 // Slow and careful pivoting
#define TRIGGER_PIN1 12 // Ultrasonic trigger pin for sensor 1 (front)
#define ECHO_PIN1 11 // Ultrasonic echo pin for sensor 1 (front)
#define TRIGGER_PIN2 10 // Ultrasonic trigger pin for sensor 2 (left side)
#define ECHO_PIN2 9 // Ultrasonic echo pin for sensor 2 (left side)
#define TRIGGER_PIN3 8 // Ultrasonic trigger pin for sensor 3 (right side)
#define ECHO_PIN3 7 // Ultrasonic echo pin for sensor 3 (right side)
#define MAX_DISTANCE 200 // Maximum distance our sensors can see (in centimeters)
#define STEPS_PER_REVOLUTION 200 // Steps for a full circle with our stepper motor (arm control)
#define BOARD_TWO_SERIAL Serial1 // Special channel for talking to our other gadgets

// Motor control pins to make our rover go zoom zoom
const int AIN1 = 5; // Motor A control pin 1
const int AIN2 = 6; // Motor A control pin 2
const int PWMA = 9; // Motor A speed control
const int PWMB = 10; // Motor B speed control
const int BIN2 = 7; // Motor B control pin 2
const int BIN1 = 8; // Motor B control pin 1

// Magnetometer calibration (fancy term for making sure our compass points in the right direction)
float hard_iron_bias_x = 8.111; // X-axis bias
float hard_iron_bias_y = 11.795; // Y-axis bias
float hard_iron_bias_z = -41.963; // Z-axis bias

float soft_iron_bias_xx = 18.467; // Calibration factor for X
float soft_iron_bias_yy = 19.582; // Calibration factor for Y
float soft_iron_bias_zz = 25.990; // Calibration factor for Z

const float mag_declination = 12.9; // Earth's magnetic field adjustment - because compasses aren't perfect :(

// GPS, Magnetometer, Ultrasonic Sensors, and Stepper Motors setup
TinyGPSPlus gps; // GPS module to know where we are (like Google Maps, but for robots)
MLX90393 mag; // Magnetometer to figure out which way we're facing
MLX90393::txyz data;
NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE); // Ultrasonic sensor at the front (to keep us from bumping into things)
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE); // Left-side ultrasonic sensor (so we can peek around corners)
NewPing sonar3(TRIGGER_PIN3, ECHO_PIN3, MAX_DISTANCE); // Right-side ultrasonic sensor (because symmetry is key)
Stepper stepperArm(STEPS_PER_REVOLUTION, 4, 5, 6, 7); // Arm to do stuff like digging (like a tiny construction worker)

// TensorFlow Lite model data (tiny AI brain goes here)
#include "model_data.h"

// Structure for our GPS waypoints (because we need to know where to go)
struct t_waypoint {
    float lat; // Latitude (how far up or down on the planet)
    float lon; // Longitude (how far left or right on the planet)
};

// List of waypoints (places our little rover wants to visit)
t_waypoint nav_waypoints[] = {
    {-17.4176, -66.13265}, // Example waypoint (somewhere cool, we hope)
};
byte waypoint_index = 0; // Keeps track of which waypoint we're heading to
int num_waypoints = sizeof(nav_waypoints) / sizeof(nav_waypoints[0]); // How many waypoints do we have?

// Machine Learning inputs and outputs (to help our rover make decisions)
float input_data[3]; // Ultrasonic sensor distances (the robot's eyes)
float output_data[2]; // AI's decisions: [0] pivot left/right, [1] go forward/stop

void setup() {
    Serial.begin(115200); // Let's talk to our computer
    BOARD_TWO_SERIAL.begin(9600); // Set up a secret communication line
    Wire.begin(); // Start the I2C communication with our sensors

    // Set up motor control pins so we can start moving
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    // Initialize the compass (Magnetometer) to figure out which direction we're facing
    mag.begin();
    mag.setOverSampling(3);
    mag.setDigitalFiltering(4);
    mag.setGainSel(5);
    mag.setResolution(0, 0, 0);

    // Set speed for the stepper motor (controls the arm)
    stepperArm.setSpeed(60);

    // Load our tiny AI brain (TensorFlow Lite model)
    tflite::MicroErrorReporter micro_error_reporter;
    static tflite::MicroInterpreter static_interpreter(
        model, tensor_arena, kTensorArenaSize, &micro_error_reporter);
    static_interpreter.AllocateTensors();
}

void loop() {
    float compass_angle;
    float waypoint_angle;

    // Read data from the magnetometer (which direction are we pointing?)
    mag.readData(data);
    float xm_off = data.x - hard_iron_bias_x;
    float ym_off = data.y - hard_iron_bias_y;
    float zm_off = data.z - hard_iron_bias_z;

    float xm_cal = xm_off * soft_iron_bias_xx;
    float ym_cal = ym_off * soft_iron_bias_yy;

    compass_angle = compute_heading(xm_cal, ym_cal); // Calculate our heading based on compass data

    // If GPS data is available, let's figure out where we need to go
    if (query_gps()) {
        t_waypoint gps_reading = get_gps_reading();
        waypoint_angle = compute_navigation_vector(gps_reading.lat, gps_reading.lon);

        // If we've reached our waypoint, move on to the next one
        if (waypoint_reached(gps_reading.lat, gps_reading.lon)) {
            waypoint_index++;
            if (waypoint_index == num_waypoints) {
                stop_motors(); // If we've reached all the waypoints, time for a nap
                while (1); // Endless nap (until someone wakes us up)
            }
        } else {
            navigate_to_waypoint(compass_angle, waypoint_angle); // Head toward our next destination
        }
    }

    // Check for any commands from the other board (because teamwork makes the dream work)
    if (BOARD_TWO_SERIAL.available()) {
        char cmd = BOARD_TWO_SERIAL.read();
        switch (cmd) {
            case 'r':
                pivot_right(); // Turn right if we're told to
                delay(1000);
                break;
            case 'l':
                pivot_left(); // Turn left if we're told to
                delay(1000);
                break;
        }
    }

    // Time for some smart obstacle detection with machine learning
    enhanced_obstacle_detection_with_ml();
}

float compute_heading(float xm_cal, float ym_cal) {
    // Calculate which way we're pointing based on magnetometer data
    float heading_rad = atan2(ym_cal, xm_cal);
    heading_rad += (mag_declination * DEG_TO_RAD); // Adjust for magnetic declination (so we don't get lost)
    float heading_deg = heading_rad * RAD_TO_DEG;
    return (heading_deg < 0) ? heading_deg + 360 : heading_deg; // Make sure the heading is always positive
}

bool query_gps() {
    // Check if there's GPS data for us to read
       if (Serial.available() > 0) {
        return gps.encode(Serial.read()); // Try to decode the GPS data 
    }
    return false; // No GPS data? Oh well, maybe next time.
}

t_waypoint get_gps_reading() {
    // Get the current GPS coordinates (let's see where we are)
    t_waypoint current;
    current.lat = gps.location.lat(); // Grab our current latitude (how far up or down we are)
    current.lon = gps.location.lng(); // Grab our current longitude (how far left or right we are)
    return current; // Let's keep going
}

float compute_navigation_vector(float current_lat, float current_lon) {
    // Figure out where we need to go compared to where we are now 
    t_waypoint target = nav_waypoints[waypoint_index]; // Get our target waypoint
    float delta_lon = target.lon - current_lon; // Difference in longitude
    float y = sin(delta_lon) * cos(target.lat);
    float x = cos(current_lat) * sin(target.lat) - sin(current_lat) * cos(target.lat) * cos(delta_lon);
    float theta = atan2(y, x); // Calculate the angle to our target
    return fmod((theta * RAD_TO_DEG + 360), 360); // Make sure the angle is between 0 and 360 degrees
}

bool waypoint_reached(float current_lat, float current_lon) {
    // Are we close enough to our target to say "Mission Accomplished"?
    t_waypoint target = nav_waypoints[waypoint_index];
    float distance = sqrt(pow(current_lat - target.lat, 2) + pow(current_lon - target.lon, 2));
    return (distance < TOLERANCE_RADIUS); // If we're within tolerance, we did it
}

void navigate_to_waypoint(float compass_angle, float waypoint_angle) {
    // Figure out which way to turn, or if we can just drive straight to our goal
    float angle_difference = compass_angle - waypoint_angle;
    if (angle_difference > MAX_HEADING_ANGLE) {
        pivot_left(); // Whoops, way off - turn left
    } else if (angle_difference < -MAX_HEADING_ANGLE) {
        pivot_right(); // Whoops, way off - turn right
    } else if (angle_difference > MIN_HEADING_ANGLE && angle_difference <= MAX_HEADING_ANGLE) {
        pivot_left_slowly(); // Close enough, but still needs a little left turn
    } else if (angle_difference < -MIN_HEADING_ANGLE && angle_difference >= -MAX_HEADING_ANGLE) {
        pivot_right_slowly(); // Close enough, but still needs a little right turn
    } else {
        go_straight(); // We’re good to go, move forward
    }
}

void enhanced_obstacle_detection_with_ml() {
    // Use our sensors and AI to avoid obstacles like a pro
    int distance1 = sonar1.ping_cm(); // Get distance from front sensor
    int distance2 = sonar2.ping_cm(); // Get distance from left sensor
    int distance3 = sonar3.ping_cm(); // Get distance from right sensor

    // Feed sensor data into our tiny AI brain
    input_data[0] = distance1;
    input_data[1] = distance2;
    input_data[2] = distance3;

    // Ask our tiny AI brain what to do next
    tflite::MicroInterpreter static_interpreter;
    TfLiteTensor* input_tensor = static_interpreter.input(0);
    for (int i = 0; i < 3; i++) {
        input_tensor->data.f[i] = input_data[i];
    }
    TfLiteStatus invoke_status = static_interpreter.Invoke(); // Run the model (aka "think really hard")
    if (invoke_status != kTfLiteOk) {
        Serial.println("Error: Invoke failed"); // Uh-oh, our tiny brain had an error
        return;
    }

    // Read AI's decision
    TfLiteTensor* output_tensor = static_interpreter.output(0);
    output_data[0] = output_tensor->data.f[0]; // Pivot decision: should we turn left or right?
    output_data[1] = output_tensor->data.f[1]; // Movement decision: go forward or stop?

    // Interpret AI's decision and act accordingly
    if (output_data[0] > 0.5) {
        pivot_left(); // AI thinks we should turn left
    } else if (output_data[0] < -0.5) {
        pivot_right(); // AI says turn right
    } else if (output_data[1] > 0.5) {
        go_straight(); // AI is happy - go forward
    } else {
        stop_motors(); // Stop and think about what to do next
        perform_scan_and_navigate(); // Scan surroundings and find a new direction
    }
}

void perform_scan_and_navigate() {
    // Spin around to find a clear path if we're stuck
    for (int angle = 0; angle < 360; angle += 15) {
        pivot_left_slowly(); // Turn left a little
        delay(100); // Wait for a moment (like when you peek around a corner)
        int distance1 = sonar1.ping_cm(); // Check if there's anything in front
        int distance2 = sonar2.ping_cm(); // Check left
        int distance3 = sonar3.ping_cm(); // Check right

        // If we find a clear path, head that way
        if (distance1 > 50 && distance2 > 50 && distance3 > 50) {
            go_straight(); // All clear, let's go
            return;
        }
    }

    // No clear path found? Keep turning until we do
    pivot_right();
    delay(1000);
}

void go_straight() {
    // Move forward at full speed
    analogWrite(PWMA, MOTOR_VEL);
    analogWrite(PWMB, MOTOR_VEL);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
}

void pivot_left() {
    // Spin to the left (like doing a little dance move)
    analogWrite(PWMA, PIVOT_WHEEL_VEL);
    analogWrite(PWMB, PIVOT_WHEEL_VEL);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
}

void pivot_right() {
    // Spin to the right (another dance move)
    analogWrite(PWMA, PIVOT_WHEEL_VEL);
    analogWrite(PWMB, PIVOT_WHEEL_VEL);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
}

void pivot_left_slowly() {
    // Slow turn to the left (careful and precise)
    analogWrite(PWMA, PIVOT_WHEEL_VEL_SLOW);
    analogWrite(PWMB, PIVOT_WHEEL_VEL_SLOW);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
}

void pivot_right_slowly() {
    // Slow turn to the right (precision is key)
    analogWrite(PWMA, PIVOT_WHEEL_VEL_SLOW);
    analogWrite(PWMB, PIVOT_WHEEL_VEL_SLOW);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
}

void stop_motors() {
    // Stop everything
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
}