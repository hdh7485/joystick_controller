
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

// This example reads an SBUS packet from an
// SBUS receiver (FrSky X8R) and then takes that
// packet and writes it back to an SBUS
// compatible servo. The SBUS out capability (i.e.
// writing a command to the servo) could be generated
// independently; however, the packet timing would need
// to be controlled by the programmer, the write function
// simply generates an SBUS packet and writes it to the
// servos. In this case the packet timing is handled by the
// SBUS receiver and waiting for a good packet read.

#include "SBUS.h"
#include <Servo.h>

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
  left_roll.attach(5);
  left_pitch.attach(6);
  right_roll.attach(9);
  right_pitch.attach(10);
  // begin the SBUS communication
  x8r.begin();
  Serial.begin(9600);
}

int convert_sbus2servo(uint16_t subs_signal, float range_gain = 1.0, bool convert_sign = false) {
  if (convert_sign)
    return (int)(1030 + ((1030 - (int)subs_signal)) * range_gain);
  else
    return (int)(1030 - ((1030 - (int)subs_signal)) * range_gain);
}

void loop() {
  // look for a good SBUS packet from the receiver
  if (x8r.read(&channels[0], &failSafe, &lostFrame)) {
    sbus_throttle = *(channels + 2);
    sbus_roll = *(channels + 0);
    sbus_pitch = *(channels + 1);
    sbus_yaw = *(channels + 3);

    Serial.println(sbus_throttle);

    int left_roll_value = convert_sbus2servo(sbus_yaw, 0.6, false);
    int left_pitch_value = convert_sbus2servo(sbus_throttle, 0.6, true);
    int right_roll_value = convert_sbus2servo(sbus_roll, 0.6, true);
    int right_pitch_value = convert_sbus2servo(sbus_pitch, 0.6, false);

    left_roll.writeMicroseconds(left_roll_value);
    left_pitch.writeMicroseconds(left_pitch_value);
    right_roll.writeMicroseconds(right_roll_value);
    right_pitch.writeMicroseconds(right_pitch_value);
  }
}
