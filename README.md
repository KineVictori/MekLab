# lab1mek

Nyttige funksjoner:

Start med å bygge
colcon build --packages-select joint_simulator
For Kine: ...select pid_controller

Deretter sette opp
source install/setup.bash

Sørge for å bruke rett ID så vi ikke prøver å kommunisere med andre
export ROS_DOMAIN_ID=17

Så kjøre applikasjonen
ros2 run joint_simulator RunJointSim
For Kine: ...run pid_controller RunPID

Så for å hente params/topics
ros2 param list
ros2 topic list

Get param

For å endre verdier
run pid_controller runPID --ros-args -p kp:=10.0 -p kd:=1.0

ros2 param set /pid_controller_node kp 0.002

Sende request i service / endre verdi
ros2 service call /set_reference pid_controller_msgs/srv/SetReference "{value: 1.5}"
