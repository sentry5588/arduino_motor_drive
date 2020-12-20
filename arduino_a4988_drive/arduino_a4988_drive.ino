/*

 Mimics the fade example but with an extra parameter for frequency. It should dim but with a flicker 
 because the frequency has been set low enough for the human eye to detect. This flicker is easiest to see when 
 the LED is moving with respect to the eye and when it is between about 20% - 60% brighness. The library 
 allows for a frequency range from 1Hz - 2MHz on 16 bit timers and 31Hz - 2 MHz on 8 bit timers. When 
 SetPinFrequency()/SetPinFrequencySafe() is called, a bool is returned which can be tested to verify the 
 frequency was actually changed.
 
 This example runs on mega and uno.
 */

#include <PWM.h>

//use pin 11 on the Mega instead, otherwise there is a frequency cap at 31 Hz
int left_step_LED_PIN = 11;                // the pin that the LED is attached to
int32_t left_frequency = 10; //frequency (in Hz)
int right_step_LED_PIN = 2;                // the pin that the LED is attached to
int extra_LED_PIN = 5;
int32_t right_frequency = 20; //frequency (in Hz)

void setup()
{
  //initialize all timers except for 0, to save time keeping functions
  InitTimersSafe(); 
}

void loop()
{
  pwmWrite(left_step_LED_PIN, 127);
  pwmWrite(right_step_LED_PIN, 127);
  pwmWrite(extra_LED_PIN, 127);
  //sets the frequency for the specified pin
  bool success_left = SetPinFrequencySafe(left_step_LED_PIN, left_frequency);
  bool success_right = SetPinFrequencySafe(right_step_LED_PIN, right_frequency);

  delay(1000);      
} 
