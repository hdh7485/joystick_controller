
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

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// a SBUS object, which is on hardware
// serial port 1
SBUS x8r(Serial1);

// channel, fail safe, and lost frames data
uint16_t channels[16];
bool failSafe;
bool lostFrame;

void setup() {
  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(5);
  servo4.attach(6);
  // begin the SBUS communication
  x8r.begin();
  Serial.begin(9600);
}

void loop() {

  // look for a good SBUS packet from the receiver
  if(x8r.read(&channels[0], &failSafe, &lostFrame)){
    Serial.println(*(channels+0));
    Serial.println((int)(1030-((1030-(int)*(channels+0)))*0.5));
    Serial.println((int)(1030-((1030-(int)*(channels+1)))*0.5));
    Serial.println((int)(1030-((1030-(int)*(channels+2)))*0.5));
    Serial.println((int)(1030+((1030-(int)*(channels+3)))*0.5));
    servo1.writeMicroseconds((int)(1030-((1030-(int)*(channels+0)))*0.5));
    servo2.writeMicroseconds((int)(1030-((1030-(int)*(channels+1)))*0.5));
    servo3.writeMicroseconds((int)(1030-((1030-(int)*(channels+2)))*0.5));
    servo4.writeMicroseconds((int)(1030+((1030-(int)*(channels+3)))*0.5));
  }
}
