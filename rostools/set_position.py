#!/usr/bin/python
"""
    A proxy to allow a Gym program to connect to a Fetch robot.

    This program connects to the robot using ROS and both observes and controls the robot.

    It runs continuously, accepting connections over zmq from a GymProxyClient, controlling
    the robot for the duration of a session, and then accepting more connections.

    It needs to have low latency both to the robot and the gym program. It works to run it on the robot,
    but it may also work to run it on a nearby computer.

    The Fetch robot is decribed at http://docs.fetchrobotics.com/robot_hardware.html

"""
import math, random, time, logging, re, base64, argparse, collections, sys, os, pdb, threading
import numpy as np
import gym
from gym.spaces import Box, Tuple
import gym.envs.proxy.server as server
import rospy
import actionlib
from rospy.numpy_msg import numpy_msg
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from sensor_msgs.msg import LaserScan, JointState, Image
from fetch_driver_msgs.msg import GripperState #as GripperState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from control_msgs.msg import PointHeadAction, PointHeadGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal


logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def main():
    arm_move_group = MoveGroupInterface("arm", "base_link", plan_only = True)
    arm_trajectory_client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    arm_trajectory_client.wait_for_server()

    argi = 0
    while argi < len(sys.argv):
        arg = sys.argv[argi]
        argi += 1
        if arg == 'arm':
            print('Contacting move_group for arm...', file=sys.stderr)
            arm_move_group = MoveGroupInterface("arm", "base_link", plan_only = True)
            arm_trajectory_client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
            arm_trajectory_client.wait_for_server()

            joints = [
                'shoulder_pan_joint',
                'shoulder_lift_joint',
                'upperarm_roll_joint',
                'elbow_flex_joint',
                'forearm_roll_joint',
                'wrist_flex_joint',
                'wrist_roll_joint',
            ]
            pose = [float(a) for a in sys.argv[argi : argi+len(joints)]]
            argi += len(joints)
            result = self.arm_move_group.moveToJointPosition(joints, pose, plan_only=True)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                follow_goal = FollowJointTrajectoryGoal()
                follow_goal.trajectory = result.planned_trajectory.joint_trajectory
                print('sending trajectory to arm...', file=sys.stderr)
                self.arm_trajectory_client.send_goal(follow_goal)
                result = self.arm_trajectory_client.wait_for_result()
                print('arm followed trajectory %s' % result, file=sys.stderr)
            else:
                logger.warn('moveToJointPosition returned %s', result)
                return
        else:
            print('Unknown joint group %s' % arg, file=sys.stderr)
            return


if __name__ == '__main__':
    logger.info('Creating FetchRobotGymEnv node. argv=%s', sys.argv)
    rospy.init_node('set_position') # After this, logging goes to ~/.ros/log/set_position.log
    main()
