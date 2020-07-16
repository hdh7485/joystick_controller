
/*
  SBUS_example.ino
  Brian R Taylor
  brian.taylor@bolderflight.com

  Copyright (c) 2016 Bolder Flight Systems

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
  and associated documentation files (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge, publish, distribute,
  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or
  substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
  DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "SBUS.h"
#include <Servo.h>

const int user_joy_mode_pin = 12;     // the number of the pushbutton pin
const int installed_joy_mode_pin = 12;     // the number of the pushbutton pin
const int operating_mode_pin = 12;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin
int user_joy_mode = 2;
int installed_joy_mode = 2;
String operating_mode = "SBUS";

Servo left_roll;
Servo left_pitch;
Servo right_roll;
Servo right_pitch;

// a SBUS object, which is on hardware
// serial port 1
SBUS x8r(Serial1);
// channel, fail safe, and lost frames data
uint16_t channels[16];
bool failSafe;
bool lostFrame;

uint16_t sbus_throttle;
uint16_t sbus_roll;
uint16_t sbus_pitch;
uint16_t sbus_yaw;


void setup() {
  pinMode(user_joy_mode_pin, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  left_roll.attach(5);
  left_pitch.attach(6);
  right_roll.attach(9);
  right_pitch.attach(10);

  // begin the SBUS communication
  x8r.begin();

  Serial.begin(9600);
}

int convert_sbus2servo(uint16_t subs_signal, bool convert_sign = false, float range_gain = 1.0, int middle_offset = 0.0) {
  if (convert_sign)
    return (int)(1030 + middle_offset + ((1030 + middle_offset - (int)subs_signal)) * range_gain);
  else
    return (int)(1030 + middle_offset - ((1030 + middle_offset - (int)subs_signal)) * range_gain);
}

void loop() {
  if (digitalRead(user_joy_mode_pin) == HIGH) {
    user_joy_mode = 2;
  } else {
    user_joy_mode = 1;
  }
  if (digitalRead(installed_joy_mode_pin) == HIGH) {
    installed_joy_mode = 2;
  } else {
    installed_joy_mode = 1;
  }
  if (digitalRead(operating_mode_pin) == HIGH) {
    operating_mode = "SBUS";
  } else {
    operating_mode = "ROS";
  }

  if (operating_mode == "SBUS") {
    if (x8r.read(&channels[0], &failSafe, &lostFrame)) {
      sbus_throttle = *(channels + 2);
      sbus_roll = *(channels + 0);
      sbus_pitch = *(channels + 1);
      sbus_yaw = *(channels + 3);

      int left_roll_value = 0;
      int left_pitch_value = 0;
      int right_roll_value = 0;
      int right_pitch_value = 0;
      if (user_joy_mode == 2) { //User Controller MODE 2
        left_roll_value = convert_sbus2servo(sbus_yaw, false, 0.6);
        left_pitch_value = convert_sbus2servo(sbus_throttle, true, 0.6);
        right_roll_value = convert_sbus2servo(sbus_roll, true, 0.6);
        right_pitch_value = convert_sbus2servo(sbus_pitch, false, 0.6);
      } else { //User Controller MODE 1
        left_roll_value = convert_sbus2servo(sbus_yaw, false, 0.6);
        left_pitch_value = convert_sbus2servo(sbus_throttle, true, 0.6);
        right_roll_value = convert_sbus2servo(sbus_roll, false, 0.6);
        right_pitch_value = convert_sbus2servo(sbus_pitch, true, 0.6);
      }

      left_roll.writeMicroseconds(left_roll_value);
      left_pitch.writeMicroseconds(left_pitch_value);
      right_roll.writeMicroseconds(right_roll_value);
      right_pitch.writeMicroseconds(right_pitch_value);
      //    Serial.println(sbus_throttle);
    }
  }
  else if (operating_mode == "ROS") {

  }
}
