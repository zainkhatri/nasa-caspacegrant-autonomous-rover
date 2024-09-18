// Board One: GPS, Magnetometer, Stepper

// [1] Required libraries for the rover's components
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <SparkFun_MLX90393_Arduino_Library.h>
#include <AFMotor.h>

// [2] Setup the GPS
SFE_UBLOX_GNSS myGNSS;

// [3] Setup the magnetometer
MLX90393 mag;

// [4] Setup the Gear Motors
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);
AF_DCMotor motor5(5);
AF_DCMotor motor6(6);

// [5] Define the rotation speed constant (please set this value based on your rover's specifications)
const float ROTATION_SPEED_CONSTANT = ...;

void setup() {

  // [6] Begin Serial Communication at a baud rate of 9600
  Serial.begin(9600);

  // [7] Set the speed of the motors
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);
  motor5.setSpeed(255);
  motor6.setSpeed(255);

  // [8] Begin communication with the GPS over I2C
  if (!myGNSS.begin()) {
    Serial.println("Failed to initialize the GPS module. Check the wiring.");
    while (1);
  }

  // [9] Begin communication with the magnetometer over I2C
  if (!mag.begin()) {
    Serial.println("Failed to initialize the magnetometer module. Check the wiring.");
    while (1);
  }
}

void loop() {
  // [10] Check if there is a new command available from the Serial Port
  if (Serial.available()) {
    parse_and_execute_command();
  }
}

void parse_and_execute_command() {
  String command = Serial.readStringUntil('\n').trim(); // Trim to remove potential whitespace
  int separatorIndex = command.indexOf(',');
  String commandType = command.substring(0, separatorIndex).trim();
  command = command.substring(separatorIndex + 1).trim();

  if (commandType == "r") {
    float target_bearing = command.toFloat();
    rotate_to_bearing(target_bearing);
  }
  else if (commandType == "m") {
    separatorIndex = command.indexOf(',');
    float destination_lat = command.substring(0, separatorIndex).toFloat();
    float destination_lon = command.substring(separatorIndex + 1).toFloat();
    move_to_location(destination_lat, destination_lon);
  }
  else {
    Serial.println("Unknown command type!");
  }
}

// [11] Rotate to a specified bearing
void rotate_to_bearing(float target_bearing) {
    float current_bearing = mag.readHeading();
    float rotation_angle = target_bearing - current_bearing;
    rotation_angle = fmod(rotation_angle + 540, 360) - 180; 

    // Calculate the time based on your robot's known rotation speed. This is a placeholder and might need adjustments.
    float rotation_time = rotation_angle / ROTATION_SPEED_CONSTANT; 
    rotate(rotation_time);
    
    // Keep adjusting the rover's heading until it matches the target bearing
    while (abs(mag.readHeading() - target_bearing) > 1) {
        delay(100);
    }
}

// [12] Function to stop the rover
void stop_rover() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  motor5.run(RELEASE);
  motor6.run(RELEASE);
}

// [13] Function to move the rover forward for a specified amount of time (seconds)
void move_forward(float time) {
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    motor5.run(FORWARD);
    motor6.run(FORWARD);
    delay(time * 1000);  // Convert time to milliseconds
    stop_rover();
}

// [14] Rotate the rover for a given time (seconds)
void rotate(float time) {
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
    delay(time * 1000); // Convert time to milliseconds
    stop_rover();
}

// [15] Function to calculate distance between two GPS points using the Haversine formula
double haversine_km(double lat1, double long1, double lat2, double long2) {
  // [15.1] Calculate the differences in coordinates and convert from degrees to radians
  double dLat = (lat2 - lat1) * M_PI / 180.0;
  double dLon = (long2 - long1) * M_PI / 180.0;

  // [15.2] Convert the original coordinates from degrees to radians
  lat1 = lat1 * M_PI / 180.0;
  lat2 = lat2 * M_PI / 180.0;

  // [15.3] Apply the Haversine formula to calculate the great-circle distance
  double a = pow(sin(dLat / 2), 2) +
             pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
  double rad = 6371;  // Earth's radius in kilometers
  double c = 2 * asin(sqrt(a));
  return rad * c;
}

// [16] Function to calculate distance and bearing from current location to destination
void calculate_distance_and_bearing(float lat1, float lon1, float lat2, float lon2, double &distance, float &bearing) {
  // [16.1] Calculate the distance between the current location and the destination using the haversine function
  distance = haversine_km(lat1, lon1, lat2, lon2);

  // [16.2] Calculate bearing using the formula
  double y = sin(lon2-lon1) * cos(lat2);
  double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(lon2-lon1);
  bearing = atan2(y, x) * 180 / M_PI;

  // [16.3] Adjust the bearing to be within the range of 0 to 360 degrees
  if (bearing < 0) {
    bearing += 360;
  }
}

// [17] Function to move the rover to a specific GPS location
void move_to_location(float destination_lat, float destination_lon) {
    double distance;
    float bearing;

    // Ensure that the GPS has a valid fix before proceeding
    if (!myGNSS.isConnected() || !myGNSS.getFix()) {
        Serial.println("Waiting for a valid GPS fix...");
        return;
    }

    calculate_distance_and_bearing(myGNSS.getLatitude(), myGNSS.getLongitude(), destination_lat, destination_lon, distance, bearing);
  
    rotate_to_bearing(bearing);  // Rotate the rover in the direction of the destination

    float v = 0.01;  // Assume a constant speed for the rover in km/h
    float time_hours = distance / v;
    unsigned long time_ms = time_hours * 60 * 60 * 1000;

    // This loop ensures we don't delay for too long, potentially causing a rollover in the delay function
    while (time_ms > 0) {
        if (time_ms > 60000) {
            move_forward(60);  // Move the rover forward for 1 minute
            time_ms -= 60000;
        } else {
            move_forward(time_ms / 1000.0);  // Move the rover forward for the remaining time
            time_ms = 0;
        }
    }

    // Keep checking the distance to the destination and stop when close enough
    while (haversine_km(myGNSS.getLatitude(), myGNSS.getLongitude(), destination_lat, destination_lon) > 0.005) { // Assuming you want to stop when 5 meters away
        delay(1000);  // Check every second
    }
    stop_rover();
}

// Main loop
void loop() {
    // Check if GPS has a valid fix
    if (!myGNSS.isConnected() || !myGNSS.getFix()) {
        Serial.println("Waiting for a valid GPS fix...");
        delay(1000); // Delay for 1 second and then recheck
        return;
    }
    
    float current_lat = myGNSS.getLatitude();
    float current_lon = myGNSS.getLongitude();
    
    // Example: move to a predefined GPS location
    // You can modify these coordinates to your desired destination
    float destination_lat = 40.748817; 
    float destination_lon = -73.985428;
    
    move_to_location(destination_lat, destination_lon);
    
    delay(10000); // Delay for 10 seconds before next loop iteration. Adjust as needed.
}


