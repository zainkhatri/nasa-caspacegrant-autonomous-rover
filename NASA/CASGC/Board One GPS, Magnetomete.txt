// Board One and Two: GPS, Magnetometer, Gear Motors, and Obstacle Avoidance

/* 

Rover Navigation Program Overview:
The program drives an Arduino-based rover equipped with a GPS and magnetometer. The aim is to autonomously navigate to specified waypoints.

1. Library and Constants: Imports libraries and defines constants for GPS and compass navigation. [1] [2]
2. Calibration: Sets up calibration parameters for the magnetometer, correcting readings for accuracy. [2]
3. Motor Setup: Defines constants and pins for motor control, essential for movement commands. [3]
4. Modules Initialization: Initializes objects and structures for the GPS module and magnetometer. [4]
5. Waypoint Management: Defines a structure for waypoints and sets up an array of target locations. [5] [6]
6. Setup Routine: Initializes serial, I2C, and devices, prepping the rover for operation. [7]
7. Main Loop: Controls the core logic, adjusting the rover's movement based on GPS and compass data. [8]
8. Rover Functions: Houses functionalities for heading computation, GPS querying, navigation vector calculation, and waypoint reach check. [9]
9. Motor Commands: Defines functions to control the motor, driving the rover's movement. [10]

*/

// [1] Required libraries for the rover's components
#include <Wire.h>
#include <MLX90393.h> // Magnetometer Library
#include <TinyGPS++.h> // GPS Library

// [2] Define constants for GPS and compass
#define TOLERANCE_RADIUS   1.0
#define MAX_HEADING_ANGLE  180
#define MIN_HEADING_ANGLE  5
#define ANGLE_RANGE_DIV    0.25

// Calibration parameters for magnetometer
float hard_iron_bias_x = 8.111879646901427;
float hard_iron_bias_y = 11.79485854875362;
float hard_iron_bias_z = -41.962932761829954;

double soft_iron_bias_xx = 18.466676975474822;
double soft_iron_bias_xy = 0.7238690079705554;
double soft_iron_bias_xz = 0.6324488038115554;
double soft_iron_bias_yx = 0.723869007970556;
double soft_iron_bias_yy = 19.582264549644204;
double soft_iron_bias_yz = -0.6148877177457805;
double soft_iron_bias_zx = 0.6324488038115559;
double soft_iron_bias_zy = -0.6148877177457804;
double soft_iron_bias_zz = 25.989834368482438

const float mag_declination = 12.9; // Magnetic declination for the specific location

// [3] Define velocities and pins for the motors
#define MOTOR_VEL 255
#define PIVOT_WHEEL_VEL 50
#define PIVOT_WHEEL_VEL_SLOW 150

// Motor control pins
const int AIN1 = 5; // control pin 1 for the right motor
const int AIN2 = 6; // control pin 2 for the right motor
const int PWMA = 9; // speed control pin for the right motor
const int PWMB = 10; // speed control pin for the left motor
const int BIN2 = 7;  // control pin 2 for the left motor
const int BIN1 = 8;  // control pin 1 for the left motor

// [4] Setup for the GPS module and magnetometer
TinyGPSPlus gps; // Object for GPS
MLX90393 mag; // Object for magnetometer
MLX90393::txyz data; // Data structure for magnetometer readings

// [5] Structure to store waypoints
struct t_waypoint {
  float lat;
  float lon;
};

// [6] Store waypoints to navigate to
t_waypoint nav_waypoints[] = {
  {-17.4176, -66.13265}, // Location 1
};
byte waypoint_index = 0; // Current waypoint index
int num_waypoints = sizeof(nav_waypoints) / sizeof(nav_waypoints[0]); // Total number of waypoints

void setup() {
  Serial.begin(115200);
  BOARD_TWO_SERIAL.begin(9600);
  Wire.begin();

  // [7] Initialize the motors
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // [8] Initialize the magnetometer
  mag.begin();
  mag.setOverSampling(3);
  mag.setDigitalFiltering(4);
  mag.setGainSel(5);
  mag.setResolution(0, 0, 0);
}

void loop() {
  float compass_angle;
  float waypoint_angle;
  
  mag.readData(data);
  // Apply calibration to magnetometer readings
  float xm_off = data.x - hard_iron_bias_x;
  float ym_off = data.y - hard_iron_bias_y;
  float zm_off = data.z - hard_iron_bias_z;

  float xm_cal = xm_off * soft_iron_bias_xx + ym_off * soft_iron_bias_yx + zm_off * soft_iron_bias_zx;
  float ym_cal = xm_off * soft_iron_bias_xy + ym_off * soft_iron_bias_yy + zm_off * soft_iron_bias_zy;
  
  compass_angle = compute_heading(xm_cal, ym_cal);

  if (query_gps()) {
    t_waypoint gps_reading = get_gps_reading();
    waypoint_angle = compute_navigation_vector(gps_reading.lat, gps_reading.lon);

    if (waypoint_reached(gps_reading.lat, gps_reading.lon)) {
      waypoint_index++;
      if (waypoint_index == num_waypoints) {
        stop_motors();
        while (1); 
      }
    } else {
      float angle_difference = compass_angle - waypoint_angle;
      if (angle_difference > MAX_HEADING_ANGLE) {
        pivot_left();
      } else if (angle_difference < -MAX_HEADING_ANGLE) {
        pivot_right();
      } else if (angle_difference > MIN_HEADING_ANGLE && angle_difference <= MAX_HEADING_ANGLE) {
        pivot_left_slowly();
      } else if (angle_difference < -MIN_HEADING_ANGLE && angle_difference >= -MAX_HEADING_ANGLE) {
        pivot_right_slowly();
      } else {
        go_straight();
      }
    }
  }
}

 if (BOARD_TWO_SERIAL.available()) {
    char cmd = BOARD_TWO_SERIAL.read();
    switch (cmd) {
      case 'r':
        pivot_right(); // Pivot right when obstacle detected on the left
        delay(1000);  // Arbitrary delay to clear obstacle
        break;
      case 'l':
        pivot_left(); // Pivot left when obstacle detected on the right
        delay(1000);  // Arbitrary delay to clear obstacle
        break;
      // Add more cases as needed
    }
  }

  if (waypoint_reached(gps_reading.lat, gps_reading.lon)) {
    BOARD_TWO_SERIAL.println("d"); // "d" command for dig
    delay(10000);  // Arbitrary delay to allow digging process before moving to the next waypoint
    waypoint_index++;
    if (waypoint_index == num_waypoints) {
      stop_motors();
      while (1);
    }
  } 
// [10] Heading Calculation
float compute_heading(float xm_cal, float ym_cal) {
  float heading_rad = atan2(ym_cal, xm_cal);
  heading_rad += (mag_declination * DEG_TO_RAD);
  float heading_deg = heading_rad * RAD_TO_DEG;
  return (heading_deg < 0) ? heading_deg + 360 : heading_deg;
}

bool query_gps() {
  if (Serial.available() > 0) {
    return gps.encode(Serial.read());
  }
  return false;
}

// [11] Compute Navigation Direction to Waypoint
float compute_navigation_vector(float current_lat, float current_lon) {
  t_waypoint target = nav_waypoints[waypoint_index];
  float delta_lon = target.lon - current_lon;
  float y = sin(delta_lon) * cos(target.lat);
  float x = cos(current_lat) * sin(target.lat) - sin(current_lat) * cos(target.lat) * cos(delta_lon);
  float theta = atan2(y, x);
  return fmod((theta * RAD_TO_DEG + 360), 360); // Ensure result is in [0, 360] range
}

bool waypoint_reached(float current_lat, float current_lon) {
  t_waypoint target = nav_waypoints[waypoint_index];
  float distance = sqrt(pow(current_lat - target.lat, 2) + pow(current_lon - target.lon, 2));
  return (distance < TOLERANCE_RADIUS);
}

// [12] Motor Control for Direction Adjustments
void go_straight() {
  analogWrite(PWMA, MOTOR_VEL);
  analogWrite(PWMB, MOTOR_VEL);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void pivot_left() {
  analogWrite(PWMA, PIVOT_WHEEL_VEL);
  analogWrite(PWMB, PIVOT_WHEEL_VEL);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void pivot_right() {
  analogWrite(PWMA, PIVOT_WHEEL_VEL);
  analogWrite(PWMB, PIVOT_WHEEL_VEL);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void pivot_left_slowly() {
  analogWrite(PWMA, PIVOT_WHEEL_VEL_SLOW);
  analogWrite(PWMB, PIVOT_WHEEL_VEL_SLOW);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void pivot_right_slowly() {
  analogWrite(PWMA, PIVOT_WHEEL_VEL_SLOW);
  analogWrite(PWMB, PIVOT_WHEEL_VEL_SLOW);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void stop_motors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

// [13] For communication with Board Two
#define BOARD_TWO_SERIAL Serial1  // Assuming Board Two is connected to Serial1