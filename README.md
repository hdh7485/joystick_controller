# joystick_controller
조이스틱 컨트롤러 로봇 코드입니다.

## 세 가지 모드
Joystick Controller는 세 가지의 모드가 있습니다. 각 모드들은 Joystick Controller 오른쪽 하단에 장착되어 있는 토글 스위치로 선택할 수 있습니다. 조이스틱의 모드는 다음 링크를 참고하시기 바랍니다. https://www.getfpv.com/learn/fpv-essentials/choosing-right-transmitter-mode/
1. 장착된 조이스틱의 모드 (1, 2)
2. 작동모드 (SBUS, ROS)
3. SBUS 모드 사용시 사용자의 조이스틱 모드(1, 2)

## ROS Mode

### Subscribed message:
- topic type: `/sensor_msgs/Joy`
- topic name: `/joy`

### How to rosrun
Control the joystick controller using ros joy node and a PS4 joystick. PS4 joystick should be connected to your ROS PC.
```
# terminal 1:
$ roscore
```
```
# terminal 2:
$ rosrun rosserial_python serial_node.py /dev/ttyACM0
```
Before run joy_node, `joy_node` package should be installed. If not, you can install using following command:`sudo apt install ros_melodic_joy`
```
#terminal 3:
$ rosrun joy_node joy
```
