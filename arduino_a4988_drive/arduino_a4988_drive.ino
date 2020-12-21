/*
*/

#include <PWM.h>

#define HALF_DUTY_CYCLE 127

#define LEFT_STEP_PIN 2
#define BUG_EXTRA_LEFT_STEP_PIN 5
#define RIGHT_STEP_PIN 11
#define LEFT_DIR 52
#define RIGHT_DIR 53
#define A4988_MS2 51
#define TEST_GREEN_LED 24


//use pin 11 on the Mega instead, otherwise there is a frequency cap at 31 Hz
int32_t left_frequency = 1; //frequency (in Hz)
int32_t right_frequency = 1; //frequency (in Hz)

int music_i = 0;
const int music_note_freq[30] = {
  0, 523, 587, 659, 698, 784, 880, 988, 0, 0,
  0, 262, 294, 330, 349, 392, 440, 494, 0, 0
};
#define music_right_motor_PIN 50
#define music_N 220
const int32_t ode_of_joy[music_N] = {
  3, 0, 3, 0, 4, 0, 5, 0,
  5, 0, 4, 0, 3, 0, 2, 0,
  1, 0, 1, 0, 2, 0, 3, 0,
  3, 0, 0, 2, 2, 0, 0, 0,
  3, 0, 3, 0, 4, 0, 5, 0,
  5, 0, 4, 0, 3, 0, 2, 0,
  1, 0, 1, 0, 2, 0, 3, 0,
  2, 0, 0, 1, 1, 0, 0, 0,

  2, 0, 2, 0, 3, 0, 1, 0,
  2, 0, 3, 4, 3, 0, 1, 0,
  2, 0, 3, 4, 3, 0, 2, 0,
  1, 0, 2, 0, 15, 0, 3, 0,
  0, 0, 3, 0, 4, 0, 5, 0,
  5, 0, 4, 0, 3, 0, 2, 0,
  1, 0, 1, 0, 2, 0, 3, 0,
  2, 0, 0, 1, 1, 0, 0, 0,

  2, 0, 2, 0, 3, 0, 1, 0,
  2, 0, 3, 4, 3, 0, 1, 0,
  2, 0, 3, 4, 3, 0, 2, 0,
  1, 0, 2, 0, 15, 0, 3, 0,
  0, 0, 3, 0, 4, 0, 5, 0,
  5, 0, 4, 0, 3, 0, 2, 0,
  1, 0, 1, 0, 2, 0, 3, 0,
  2, 0, 0, 1, 1, 0, 0, 0};

int test_led_status = 0;
long incoming_int = 0;
unsigned long previousMillis = 0;
const long interval = 250;

void setup()
{
  //initialize all timers except for 0, to save time keeping functions
  InitTimersSafe();
  pinMode(TEST_GREEN_LED, OUTPUT);
  pinMode(music_right_motor_PIN, OUTPUT);
  pinMode(A4988_MS2, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  Serial.begin(115200);
}

void loop()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;



    //sets the frequency for the specified pin

    if (music_i < 192) {
      if (ode_of_joy[music_i] > 0) {
        pwmWrite(LEFT_STEP_PIN, HALF_DUTY_CYCLE);
        pwmWrite(BUG_EXTRA_LEFT_STEP_PIN, HALF_DUTY_CYCLE);

        pwmWrite(RIGHT_STEP_PIN, HALF_DUTY_CYCLE);
        digitalWrite(music_right_motor_PIN, HIGH);
        bool success_left = SetPinFrequencySafe(LEFT_STEP_PIN, music_note_freq[ode_of_joy[music_i]]);
        Serial.print(music_i); Serial.print(", ");
        Serial.println(music_note_freq[ode_of_joy[music_i]]);
        bool success_right = SetPinFrequencySafe(RIGHT_STEP_PIN, right_frequency);
        delay(50);
        digitalWrite(music_right_motor_PIN, LOW);
      }
    } else {
      pwmWrite(LEFT_STEP_PIN, 0);
      pwmWrite(BUG_EXTRA_LEFT_STEP_PIN, 0);

      pwmWrite(RIGHT_STEP_PIN, 0);
    }
    music_i ++;
    if (music_i == music_N) {
      music_i = 0;
    }

    digitalWrite(A4988_MS2, HIGH);
    digitalWrite(LEFT_DIR, HIGH);
    digitalWrite(RIGHT_DIR, HIGH);

    if (test_led_status == 0) {
      digitalWrite(TEST_GREEN_LED, HIGH);
      test_led_status = 1;
    } else {
      digitalWrite(TEST_GREEN_LED, LOW);
      test_led_status = 0;
    }


  }

  if (Serial.available() > 0) {
    // read the incoming byte:
    incoming_int = Serial.parseInt();
    //    left_frequency = (int32_t) incoming_int;

    // say what you got:
    Serial.print("I received: ");
    Serial.println(incoming_int);
  }

}
