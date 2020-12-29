/*
    ROS node to subscribe to motor steps and drive stepper motors
    Code compiled in Arduino 1.8.13 in Ubuntu 18.04
*/
#include <PWM.h>
#include <ros.h>
#include <std_msgs/Int32.h>

#define HALF_DUTY_CYCLE 127
#define ZERO_DUTY_CYCLE 0
#define LEFT_STEP_PIN 2
#define BUG_EXTRA_LEFT_STEP_PIN 5 // An Arduino (or IDE) bug that required PIN5 to be written while writing to PIN2
#define RIGHT_STEP_PIN 11
#define LEFT_DIR 52
#define RIGHT_DIR 53
#define A4988_MS2 51
#define ON_BOARD_LED_PIN 13

unsigned long previousMillis_ros_spin = 0;
unsigned long previousMillis_LED = 0;
const long interval_ros_spin = 1;
const long interval_LED = 1000;
int empty_loop_count = 0;

ros::NodeHandle nh;

void left_motor_speed_callback(const std_msgs::Int32& s) {
  int32_t v = 1;

  if (s.data < 0) { // forward speed
    digitalWrite(LEFT_DIR, HIGH);
    pwmWrite(LEFT_STEP_PIN, HALF_DUTY_CYCLE);
    pwmWrite(BUG_EXTRA_LEFT_STEP_PIN, HALF_DUTY_CYCLE);
    v = - (int32_t) s.data;
  } else if (s.data > 0) { // reverse speed
    digitalWrite(LEFT_DIR, LOW);
    pwmWrite(LEFT_STEP_PIN, HALF_DUTY_CYCLE);
    pwmWrite(BUG_EXTRA_LEFT_STEP_PIN, HALF_DUTY_CYCLE);
    v = (int32_t) s.data;
  } else { // Zero speed
    digitalWrite(LEFT_DIR, LOW); // Though not needed, set to low to save power
    pwmWrite(LEFT_STEP_PIN, ZERO_DUTY_CYCLE);
    pwmWrite(BUG_EXTRA_LEFT_STEP_PIN, ZERO_DUTY_CYCLE);
    v = 1; // Frequency cannot be 0 Hz
  }
  bool success = SetPinFrequencySafe(LEFT_STEP_PIN, v);
  if (!success) {
    nh.logerror("L_CB fail");
  }
}

void right_motor_speed_callback(const std_msgs::Int32& s) {
  int32_t v = 1;

  if (s.data < 0) { // forward speed
    digitalWrite(RIGHT_DIR, HIGH);
    pwmWrite(RIGHT_STEP_PIN, HALF_DUTY_CYCLE);
    v = - (int32_t) s.data;
  } else if (s.data > 0) { // reverse speed
    digitalWrite(RIGHT_DIR, LOW);
    pwmWrite(RIGHT_STEP_PIN, HALF_DUTY_CYCLE);
    v = (int32_t) s.data;
  } else { // Zero speed
    digitalWrite(RIGHT_DIR, LOW); // Though not needed, set to low to save power
    pwmWrite(RIGHT_STEP_PIN, ZERO_DUTY_CYCLE);
    v = 1; // Frequency cannot be 0 Hz
  }
  bool success = SetPinFrequencySafe(RIGHT_STEP_PIN, v);
  if (!success) {
    nh.logerror("R_CB fail"); // Display 
  }
}

ros::Subscriber<std_msgs::Int32> sub_left("left_motor_hz", &left_motor_speed_callback );
ros::Subscriber<std_msgs::Int32> sub_right("right_motor_hz", &right_motor_speed_callback );

void setup()
{
  nh.initNode();
  nh.subscribe(sub_left);
  nh.subscribe(sub_right);

  //initialize all timers except for 0, to save time keeping functions
  InitTimersSafe();
  pinMode(A4988_MS2, OUTPUT); // For 1/4 micro-step operation of A4988
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(ON_BOARD_LED_PIN, OUTPUT);
}

void loop()
{
  unsigned long currentMillis = millis();

  // Use faster loop to drive the motor
  if (currentMillis - previousMillis_ros_spin >= interval_ros_spin) {
    previousMillis_ros_spin = currentMillis;
    nh.spinOnce();
    empty_loop_count = 0;
  } else {
    empty_loop_count ++;
  }

  // Use slow loop to drive LED to indicate run status
  if (currentMillis - previousMillis_LED >= interval_LED) {
    previousMillis_LED = currentMillis;
    digitalWrite(ON_BOARD_LED_PIN, HIGH - digitalRead(ON_BOARD_LED_PIN)); // blink the on-board led as a status indicator
    digitalWrite(A4988_MS2, HIGH); // Set it in a slow loop to save computation power
  }
}
