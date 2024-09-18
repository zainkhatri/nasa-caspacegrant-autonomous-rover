// PART ONE : AUTONMOUS ROVER

// I found a very similar code on Git, and I've been tweaking it here and there 
// to fit the needs of our parts. I numbered off the steps for each process as well.
// All of it won't fit on one board, so we have to split it up.

// Board ONE:
// The first microcontroller board is dedicated to the GPS module, the magnetometer, and the two stepper motors. 
// This board handles navigation (both the calculation and execution of the rover's path).
// This is code blocks [3], [4], [7], [8], [9] and part of [10].

// Board TWO:
// The second board is for the three ultrasonic sensors. These sensors are used to detect obstacles and avoid bumping into stuff.
// The processing and decision making based on the ultrasonic sensor data is here. 
// This is code blocks [1], [5], [6] and the other part of [10] that wasn't used before.

// Both boards communicate with each other to share sensor readings, make decisions, and coordinate the rover's actions.
// For example, if the second board detects an obstacle via the ultrasonic sensors, it would send a signal to the first board to halt or change the rover's direction.

// [0] Required libraries for the rover's components
#include <Wire.h>  // I2C communication (part of Arduino core)
#include <Adafruit_LSM303DLHC.h>  // Updated library for LSM303DLHC Magnetometer (similar to LSM303DLH)
#include <Adafruit_GPS.h>  // GPS Module
#include <Adafruit_MotorShield.h>  // Motor control (Adafruit Motor Shield V2)

// [1] Define the trigger and echo pins for the ultrasonic sensors
#define TRIGGER_PIN_1 7
#define ECHO_PIN_1 8
#define TRIGGER_PIN_2 9  
#define ECHO_PIN_2 10
#define TRIGGER_PIN_3 11  
#define ECHO_PIN_3 12

// [2] Set the rate at which GPS updates, the distance threshold for obstacle detection, and the compass threshold for direction correction
#define GPS_UPDATE_RATE 1000
#define DISTANCE_THRESHOLD 50
#define COMPASS_THRESHOLD 10

// [3] Setup for the motor shield and stepper motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *stepper_motor_1 = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *stepper_motor_2 = AFMS.getStepper(200, 2);

// [4] Setup for the GPS module and magnetometer
Adafruit_GPS GPS(&Wire);
Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345);

// [5] Function to read from ultrasonic sensor
long read_distance_sensor(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = (duration / 2) / 29.1;
  return distance;
}

// [6] Function to stop the rover
void stop_rover() {
  stepper_motor_1->release();
  stepper_motor_2->release();
  Serial.println("Rover has been stopped.");
}

// [7] Function to rotate the rover
void rotate(int angle) {
  int steps = abs(angle) * 200 / 180; 

  if (angle > 0) {
    for(int i=0; i<steps; i++) {
      stepper_motor_1->onestep(BACKWARD, INTERLEAVE);
      stepper_motor_2->onestep(FORWARD, INTERLEAVE);
    }
  } else {
    for(int i=0; i<steps; i++) {
      stepper_motor_1->onestep(FORWARD, INTERLEAVE);
      stepper_motor_2->onestep(BACKWARD, INTERLEAVE);
    }
  }
  
  stepper_motor_1->release();
  stepper_motor_2->release();
}

// [8] Function to move the rover forward
void move_forward() {
  int steps = 100; 

  for(int i=0; i<steps; i++) {
    stepper_motor_1->onestep(FORWARD, INTERLEAVE);
    stepper_motor_2->onestep(FORWARD, INTERLEAVE);
  }

  stepper_motor_1->release();
  stepper_motor_2->release();
}

// [9] Setup function to initialize the components
void setup() {
  AFMS.begin(); // Initialize the motor shield
  
  // Initialize the GPS module
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  
  mag.begin(); // Initialize the magnetometer

  // Initialize the pins for the ultrasonic sensors
  pinMode(TRIGGER_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIGGER_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  pinMode(TRIGGER_PIN_3, OUTPUT);
  pinMode(ECHO_PIN_3, INPUT);
}

// [10] Main loop function where the rover's logic is implemented
void loop() {
  static unsigned long prevUpdateTime = 0;
  static unsigned long rotateTime = 0;
  int current_direction = 0;

  // [10.1] Check for GPS updates
  if (millis() - prevUpdateTime >= GPS_UPDATE_RATE) {
    prevUpdateTime = millis();
    while (GPS.available()) {
      char c = GPS.read();
      if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA())) {
          continue;
        }
        if (GPS.fix) {
          current_direction = GPS.track_angle_deg; // Update the current direction based on GPS
        } else {
          stop_rover();
          Serial.println("ERROR: Lost GPS Fix. Rover halted.");

          // Attempt to re-establish GPS fix for a certain time
          unsigned long gpsFixAttemptStart = millis();
          while(millis() - gpsFixAttemptStart < GPS_FIX_ATTEMPT_DURATION) {
            // Try to re-establish GPS fix here
            GPS.begin(9600);
            GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
            GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
            delay(2000); // give the GPS some time to obtain a fix
            if(GPS.fix){
              Serial.println("GPS Fix Re-established");
              break;
          }
        }
      }
    }
  }

  // [10.2] Read distances from the three ultrasonic sensors
  int distance_left = read_distance_sensor(TRIGGER_PIN_1, ECHO_PIN_1);
  int distance_center = read_distance_sensor(TRIGGER_PIN_2, ECHO_PIN_2);
  int distance_right = read_distance_sensor(TRIGGER_PIN_3, ECHO_PIN_3);

  // [10.3] If an obstacle is detected in front (center sensor), rotate and check left and right distances
  if (distance_center < DISTANCE_THRESHOLD) {
    if (millis() - rotateTime > ROTATE_INTERVAL) {  // Non-blocking delay
      rotate(90);
      rotateTime = millis();

      distance_left = read_distance_sensor(TRIGGER_PIN_1, ECHO_PIN_1);
      distance_right = read_distance_sensor(TRIGGER_PIN_3, ECHO_PIN_3);

      int target_direction = 0;
      if (distance_left > distance_right) {
        target_direction = 90;
      } else {
        target_direction = -90;
      }

      rotate(target_direction);

      // Communicate with second board to stop the rover due to obstacle
      Serial2.println("STOP");  // Send "STOP" command over Serial2 to the second board
    }
  }

  // [10.4] Check if the rover's heading (from magnetometer) is different from the GPS heading, if so, rotate to align with GPS heading
  int heading = mag.readHeading();
  if (abs(heading - current_direction) > COMPASS_THRESHOLD) {
    int target_direction = heading;
    rotate(target_direction);
  }

  // [10.5] Move the rover forward
  move_forward();

  delay(100);
}



// PART ONE : AUTONMOUS ROVER

#include <Wire.h>  // I2C communication (part of Arduino core)
#include <Adafruit_LSM303DLHC.h>  // Updated library for LSM303DLHC Magnetometer (similar to LSM303DLH)
#include <Adafruit_GPS.h>  // GPS Module
#include <Adafruit_MotorShield.h>  // Motor control (Adafruit Motor Shield V2)

// Ultrasonic Sensor Pins
#define TRIGGER_PIN_1 7
#define ECHO_PIN_1 8
#define TRIGGER_PIN_2 9  
#define ECHO_PIN_2 10
#define TRIGGER_PIN_3 11  
#define ECHO_PIN_3 12

// Waypoints to navigate to
struct t_waypoint {
    float lat;
    float lon;
};

t_waypoint nav_waypoints[] = {
    {-17.4176, -66.13265},
    {-17.4180, -66.13270},
    {-17.4185, -66.13280}
};

int waypoint_index = 0; // Current waypoint
int num_waypoints = sizeof(nav_waypoints) / sizeof(nav_waypoints[0]);

// Distance threshold for obstacle detection
#define DISTANCE_THRESHOLD 50  // Arbitrary units

// Functions for ultrasonic sensors
long read_distance_sensor(int triggerPin, int echoPin) {
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    long distance = (duration / 2) / 29.1;
    return distance;
}

// Function to stop the rover
void stop_rover() {
    Serial.println("Rover has been stopped due to obstacle.");
    // Add code to stop motors
}

// Function to compute navigation vector (simplified)
float compute_navigation_vector(float current_lat, float current_lon) {
    t_waypoint target = nav_waypoints[waypoint_index];
    float delta_lon = target.lon - current_lon;
    float theta = atan2(delta_lon, target.lat - current_lat);
    return theta * RAD_TO_DEG;
}

// Main setup routine
void setup() {
    Serial.begin(115200);  // Start serial communication for debugging
    pinMode(TRIGGER_PIN_1, OUTPUT);
    pinMode(ECHO_PIN_1, INPUT);
    pinMode(TRIGGER_PIN_2, OUTPUT);
    pinMode(ECHO_PIN_2, INPUT);
    pinMode(TRIGGER_PIN_3, OUTPUT);
    pinMode(ECHO_PIN_3, INPUT);
}

// Main loop
void loop() {
    // Read distances from ultrasonic sensors
    int distance_left = read_distance_sensor(TRIGGER_PIN_1, ECHO_PIN_1);
    int distance_center = read_distance_sensor(TRIGGER_PIN_2, ECHO_PIN_2);
    int distance_right = read_distance_sensor(TRIGGER_PIN_3, ECHO_PIN_3);

    // Check for obstacles
    if (distance_center < DISTANCE_THRESHOLD || distance_left < DISTANCE_THRESHOLD || distance_right < DISTANCE_THRESHOLD) {
        stop_rover();  // Stop the rover if an obstacle is detected
        delay(1000);  // Wait a bit before rechecking
        return;  // Skip the rest of the loop and stop movement
    }

    // If no obstacle, continue navigating to waypoint
    if (waypoint_index < num_waypoints) {
        // Simulate current GPS position (you will replace this with actual GPS data)
        float current_lat = -17.4170;  // Replace with actual GPS reading
        float current_lon = -66.13250;  // Replace with actual GPS reading

        float navigation_angle = compute_navigation_vector(current_lat, current_lon);
        
        // Add code to control motors based on navigation_angle
        Serial.print("Navigating towards waypoint ");
        Serial.println(waypoint_index + 1);
        
        // Check if the waypoint is reached
        if (sqrt(pow(current_lat - nav_waypoints[waypoint_index].lat, 2) + pow(current_lon - nav_waypoints[waypoint_index].lon, 2)) < 0.0001) {
            Serial.println("Waypoint reached!");
            waypoint_index++;  // Move to the next waypoint
        }
    } else {
        stop_rover();  // Stop rover if all waypoints are completed
        while (1);  // Stop execution
    }

    delay(100);  // Small delay before next loop iteration
}

// Board Two: Ultrasonic Sensors

// The second board is for the three ultrasonic sensors. These sensors are used to detect obstacles and avoid bumping into stuff.
// The processing and decision making based on the ultrasonic sensor data is here. 
// This is code blocks [1], [5], [6] and the other part of [10] that wasn't used before.

// [1] Define the trigger and echo pins for the ultrasonic sensors
#define TRIGGER_PIN_1 7
#define ECHO_PIN_1 8
#define TRIGGER_PIN_2 9  
#define ECHO_PIN_2 10
#define TRIGGER_PIN_3 11  
#define ECHO_PIN_3 12

// [2] Set the rate at which GPS updates, the distance threshold for obstacle detection, and the compass threshold for direction correction
#define DISTANCE_THRESHOLD 50

// [5] Function to read from ultrasonic sensor
long read_distance_sensor(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = (duration / 2) / 29.1;
  return distance;
}

// [6] Function to stop the rover
void stop_rover() {
  Serial.println("Rover has been stopped.");
  Serial2.println("STOP");  // Send "STOP" command over Serial2 to the first board
}

void setup() {
  // Initialize the pins for the ultrasonic sensors
  pinMode(TRIGGER_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIGGER_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  pinMode(TRIGGER_PIN_3, OUTPUT);
  pinMode(ECHO_PIN_3, INPUT);

  // Initialize Serial for debug output
  Serial.begin(9600);

  // Initialize Serial2 for communication with the first board
  Serial2.begin(9600);
}

void loop() {
  // Read distances from the three ultrasonic sensors
  int distance_left = read_distance_sensor(TRIGGER_PIN_1, ECHO_PIN_1);
  int distance_center = read_distance_sensor(TRIGGER_PIN_2, ECHO_PIN_2);
  int distance_right = read_distance_sensor(TRIGGER_PIN_3, ECHO_PIN_3);

  // If an obstacle is detected in front (center sensor), stop the rover
  if (distance_center < DISTANCE_THRESHOLD) {
    stop_rover();
  }

  delay(100);
}