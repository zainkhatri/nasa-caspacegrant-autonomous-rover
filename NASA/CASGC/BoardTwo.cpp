// Board Two: Robot Navigation and Digging Control

// [1] Include the NewPing library to control the ultrasonic sensors
#include <NewPing.h>
// [2] Include the Stepper library for controlling the motors
#include <Stepper.h>

// [3] Define the pins for the ultrasonic sensors
#define TRIGGER_PIN1  12  // Trigger Pin of the Front Ultrasonic Sensor
#define ECHO_PIN1     11  // Echo Pin of the Front Ultrasonic Sensor
#define TRIGGER_PIN2  10  // Trigger Pin of the Side Ultrasonic Sensor
#define ECHO_PIN2     9   // Echo Pin of the Side Ultrasonic Sensor

// [4] Define the maximum distance (in cm) we want to ping
#define MAX_DISTANCE 200

// [5] Initialize the ultrasonic sensors
NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE); // Front Ultrasonic Sensor 
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE); // Side Ultrasonic Sensor 

// [6] Define the number of steps per revolution and pins for the stepper motor
#define STEPS_PER_REVOLUTION 200
#define MOTOR_PIN1 4
#define MOTOR_PIN2 5
#define MOTOR_PIN3 6
#define MOTOR_PIN4 7

// [7] Initialize the stepper motor
Stepper stepperArm(STEPS_PER_REVOLUTION, MOTOR_PIN1, MOTOR_PIN2, MOTOR_PIN3, MOTOR_PIN4);
Stepper stepperShoulder(STEPS_PER_REVOLUTION, MOTOR_PIN1, MOTOR_PIN2, MOTOR_PIN3, MOTOR_PIN4);

void setup() {
  // [8] Begin Serial Communication at a baud rate of 9600
  Serial.begin(9600);
  
  // [9] Set the speed for the stepper motors
  stepperArm.setSpeed(60);
  stepperShoulder.setSpeed(60);
}

void loop() {
  // [10] Check for any communication from Board One. E.g., if it sends 'g' for "GO", start the obstacle detection
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'g') {
      detectObstacles();
    }
  }
}

// [11] Define the function for obstacle detection and navigation commands
void detectObstacles() {
  // [12] Check distance on sonar 1
  delay(50); // Wait 50ms between pings
  int distance1 = sonar1.ping_cm();

  // [13] Check distance on sonar 2
  delay(50); // Wait 50ms between pings
  int distance2 = sonar2.ping_cm();

  // [14] If an obstacle is detected within 50cm by sensor1, send command to turn right and dig
  if(distance1 > 0 && distance1 < 50){
    Serial.println("r");
    digWithArm();
  }
  // [15] If an obstacle is detected within 50cm by sensor2, send command to turn left
  else if(distance2 > 0 && distance2 < 50){
    Serial.println("l");
  }
  // [16] If no obstacle is detected by both sensors, send command to move forward
  else{
    Serial.println("f");
  }
}

// [17] Define the function for the arm digging movement
void digWithArm() {
  // Dig down into the ground
  stepperArm.step(-45);  
  delay(1000);

  // Swing upwards to deposit the dirt into a bucket
  stepperArm.step(90);  
  delay(1000);

  // Return the arm to its original position
  stepperArm.step(-45);
}

// [18] Similarly, define the function for the shoulder digging movement
void digWithShoulder() {
  stepperShoulder.step(100);
  delay(1000);
  stepperShoulder.step(-100);
}