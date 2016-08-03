

default :

include ../kluster/make-utils.inc

force :


# Run these on the robot
show-battery-state:
	rostopic echo -n 1 /robot_state/charger

reset-breakers:
	rosservice call /arm_breaker false && rosservice call /arm_breaker true
	rosservice call /base_breaker false && rosservice call /base_breaker true
	rosservice call /gripper_breaker false && rosservice call /gripper_breaker true
