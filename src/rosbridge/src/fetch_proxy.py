#!/usr/local/bin/python
import math, random, time, logging, re, base64, argparse, collections, sys, os
import numpy as np
import gym
from gym.spaces import Box, Tuple
import gym.envs.proxy.server as server
import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


move_group=None
planning_scene=None
joint_names=None

def setup_node():
    global move_group, planning_scene, joint_names
    logger.info('Setting up FetchRobotGymEnv ROS node...')
    rospy.init_node('FetchRobotGymEnv')
    move_group = MoveGroupInterface('arm_with_torso', 'base_link')

    planning_scene = PlanningSceneInterface('base_link')
    planning_scene.removeCollisionObject('my_front_ground')
    planning_scene.removeCollisionObject('my_back_ground')
    planning_scene.removeCollisionObject('my_right_ground')
    planning_scene.removeCollisionObject('my_left_ground')
    planning_scene.addCube('my_front_ground', 2, 1.1, 0.0, -1.0)
    planning_scene.addCube('my_back_ground', 2, -1.2, 0.0, -1.0)
    planning_scene.addCube('my_left_ground', 2, 0.0, 1.2, -1.0)
    planning_scene.addCube('my_right_ground', 2, 0.0, -1.2, -1.0)

    joint_names = [
        'torso_lift_joint', 'shoulder_pan_joint',
        'shoulder_lift_joint', 'upperarm_roll_joint',
        'elbow_flex_joint', 'forearm_roll_joint',
        'wrist_flex_joint', 'wrist_roll_joint']

    logger.info('FetchRobotGymEnv ROS node running')

class FetchRobotEnv:
    def __init__(self):
        self.setup_node()
        self.observation_space = Box(-1, 1, [joint_names.length])
        self.action_space = Box(-1, 1, [joint_names.length])
        self.reward_range = [-1,+1]

    def reset(self):
        move_group.get_move_action().cancel_all_goals()
        obs = np.array([0.0])
        return obs

    def step(self, action):
        move_group.moveToJointPosition(joint_names, action, wait=False)
        move_group.get_move_action().wait_for_result()
        result = move_group.get_move_action().get_result()
        logger.info('moveit result %s', result)

        obs = np.array(result)
        reward = 0.0
        done = False
        info = {}
        return obs, reward, done, info

    def render(self, mode='human', close=False):
        return np.zeros([24,32,3])

setup_node()
server.register(id='FetchRobot-v0', cls=FetchRobotEnv)
server.serve_forever(port=9000)
