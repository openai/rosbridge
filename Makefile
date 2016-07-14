

default :

include ../kluster/make-utils.inc

force :


# Run these on the robot
show-battery-state:
	rostopic echo -n 1 /robot_state/charger
