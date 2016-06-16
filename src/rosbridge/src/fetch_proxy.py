#!/usr/local/bin/python
import math, random, time, logging, re, base64, argparse, collections, sys, os
import numpy as np
import gym
from gym.spaces import Box, Tuple
import gym.envs.proxy.server as server
import rospy
import pdb
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


move_group=None
planning_scene=None
joint_names=None

def setup_node():
    if 1:
        global move_group, planning_scene, joint_names
        logger.info('Connected to ROS as FetchRobotGymEnv')
        if 1:
            move_group = MoveGroupInterface('arm_with_torso', 'base_link')

        if 0:
            logger.info('Creating PlanningSceneInterface...')
            planning_scene = PlanningSceneInterface('base_link')
            planning_scene.removeCollisionObject('my_front_ground')
            planning_scene.removeCollisionObject('my_back_ground')
            planning_scene.removeCollisionObject('my_right_ground')
            planning_scene.removeCollisionObject('my_left_ground')
            planning_scene.addCube('my_front_ground', 2, 1.1, 0.0, -1.0)
            planning_scene.addCube('my_back_ground', 2, -1.2, 0.0, -1.0)
            planning_scene.addCube('my_left_ground', 2, 0.0, 1.2, -1.0)
            planning_scene.addCube('my_right_ground', 2, 0.0, -1.2, -1.0)
            logger.info('PlanningSceneInterface ok')

    joint_names = [
        'torso_lift_joint', 'shoulder_pan_joint',
        'shoulder_lift_joint', 'upperarm_roll_joint',
        'elbow_flex_joint', 'forearm_roll_joint',
        'wrist_flex_joint', 'wrist_roll_joint']

    logger.info('FetchRobotGymEnv ROS node running')

class FetchRobotGymEnv:
    def __init__(self):
        self.observation_space = Box(-1, 1, [len(joint_names)])
        self.action_space = Box(-1, 1, [len(joint_names)])
        self.reward_range = [-1,+1]

    def reset(self):
        if 0:
            move_group.get_move_action().cancel_all_goals()
        obs = np.array([0 for x in joint_names])
        return obs

    def step(self, action):
        logger.info('moveit action %s', action)
        move_group.moveToJointPosition(joint_names, action, wait=False)
        move_group.get_move_action().wait_for_result()
        result = move_group.get_move_action().get_result()
        logger.info('moveit result %s', result)
        pdb.set_trace()
        obs = np.array(result.trajectory_start.position) # FIXME

        reward = 0.0
        done = False
        info = {}
        return obs, reward, done, info

    def render(self, mode='human', close=False):
        return np.zeros([240,320,3], dtype='uint8')

    def close(self):
        pass

def make_env(name):
    if name == 'FetchRobot-v0':
        return FetchRobotGymEnv()
    else:
        raise Exception('Unknown env name %s' % name)

if __name__ == '__main__':
    logger.info('Creating FetchRobotGymEnv node. argv=%s', sys.argv)
    rospy.init_node('FetchRobotGymEnv') # After this, logging goes to ~/.ros/log/FetchRobotGymEnv.log
    rospy.myargv(argv=sys.argv)
    setup_node()
    zmqs = server.GymProxyZmqServer('tcp://0.0.0.0:6911', make_env)
    zmqs.run_main()
