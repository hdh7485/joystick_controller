#include "SBUS.h"
#include <Servo.h>
//#include <ros.h>
//#include <sensor_msgs/Joy.h>

//ros::NodeHandle  nh;

const int user_joy_mode_pin = 12;     // the number of the pushbutton pin
const int installed_joy_mode_pin = 24;     // the number of the pushbutton pin
const int operating_mode_pin = 28;     // the number of the pushbutton pin
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


//void joyCb(const sensor_msgs::Joy& joy_msg) {
//  uint16_t value1 = uint16_t((joy_msg.axes[3]) * -500 + 1030);
//  uint16_t value2 = uint16_t((joy_msg.axes[4]) * 500 + 1030);
//  uint16_t value3 = uint16_t((joy_msg.axes[1]) * -500 + 1030);
//  uint16_t value4 = uint16_t((joy_msg.axes[0]) * -500 + 1030);
//  left_roll.writeMicroseconds(value4);
//  left_pitch.writeMicroseconds(value3);
//  right_roll.writeMicroseconds(value1);
//  right_pitch.writeMicroseconds(value2);
//}

//ros::Subscriber<sensor_msgs::Joy> sub("/joy", &joyCb );

void setup() {
  pinMode(user_joy_mode_pin, INPUT);
  pinMode(operating_mode_pin, INPUT);
  pinMode(ledPin, OUTPUT);


  left_roll.attach(5);
  left_pitch.attach(6);
  right_roll.attach(9);
  right_pitch.attach(10);

  // begin the SBUS communication
  x8r.begin();

  //  nh.initNode();
  //  nh.subscribe(sub);

  Serial.begin(9600);
}

int convert_sbus2servo(uint16_t subs_signal, bool convert_sign = false, float range_gain = 1.0, int middle_offset = 0, int default_val = 1030) {
  if (convert_sign)
    return (int)(1030 + middle_offset + ((default_val + middle_offset - (int)subs_signal)) * range_gain);
  else
    return (int)(1030 + middle_offset - ((default_val + middle_offset - (int)subs_signal)) * range_gain);
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
  digitalWrite(ledPin, digitalRead(operating_mode_pin));
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
        left_roll_value = convert_sbus2servo(sbus_yaw, false, 0.48);
        left_pitch_value = convert_sbus2servo(sbus_throttle, false, 0.48, 0, 1387);
        right_roll_value = convert_sbus2servo(sbus_roll, false, 0.48);
        right_pitch_value = convert_sbus2servo(sbus_pitch, true, 0.48);
      }

      left_roll.writeMicroseconds(left_roll_value);
      left_pitch.writeMicroseconds(left_pitch_value);
      right_roll.writeMicroseconds(right_roll_value);
      right_pitch.writeMicroseconds(right_pitch_value);
      Serial.print("throttle: ");
      Serial.println(sbus_throttle);
      Serial.print("roll: ");
      Serial.println(sbus_roll);
      Serial.print("pitch: ");
      Serial.println(sbus_pitch);
      Serial.print("yaw: ");
      Serial.println(sbus_yaw);
    }
  }
  else if (operating_mode == "ROS") {
    //    nh.spinOnce();
    delay(1);
  }
}
