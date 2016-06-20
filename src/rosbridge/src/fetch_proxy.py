#!/usr/local/bin/python
import math, random, time, logging, re, base64, argparse, collections, sys, os, pdb, threading
import numpy as np
import gym
from gym.spaces import Box, Tuple
import gym.envs.proxy.server as server
import rospy
from rospy.numpy_msg import numpy_msg
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from sensor_msgs.msg import LaserScan, JointState, Image
from fetch_driver_msgs.msg import GripperState #as GripperState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

fetch_api = None

class FetchRobotApi:
    def __init__(self, do_movement=False):
        self.do_movement = do_movement
        self.use_rgb_camera = use_rgb_camera

        logger.info('Subscribing...')
        self.subs = []
        self.subs.append(rospy.Subscriber('/base_scan', LaserScan, self.base_scan_cb))
        self.subs.append(rospy.Subscriber('/joint_states', JointState, self.joint_states_cb))
        self.subs.append(rospy.Subscriber('/joint_states', JointState, self.joint_states_cb))
        self.subs.append(rospy.Subscriber('/head_camera/depth/image', numpy_msg(Image), self.head_camera_depth_image_cb))
        self.subs.append(rospy.Subscriber('/head_camera/rgb/image_raw', numpy_msg(Image), self.head_camera_rgb_image_raw_cb))
        logger.info('Subscribed')

        self.move_group = None
        self.planning_scene = None
        if self.do_movement:
            self.move_group = MoveGroupInterface('arm_with_torso', 'base_link')
            if 1:
                logger.info('Creating PlanningSceneInterface...')
                self.planning_scene = PlanningSceneInterface('base_link')
                self.planning_scene.removeCollisionObject('my_front_ground')
                self.planning_scene.removeCollisionObject('my_back_ground')
                self.planning_scene.removeCollisionObject('my_right_ground')
                self.planning_scene.removeCollisionObject('my_left_ground')
                self.planning_scene.addCube('my_front_ground', 2, 1.1, 0.0, -1.0)
                self.planning_scene.addCube('my_back_ground', 2, -1.2, 0.0, -1.0)
                self.planning_scene.addCube('my_left_ground', 2, 0.0, 1.2, -1.0)
                self.planning_scene.addCube('my_right_ground', 2, 0.0, -1.2, -1.0)

        self.joint_names = [
            'torso_lift_joint',
            'bellows_joint',
            'head_pan_joint', 'head_tilt_joint',
            'shoulder_pan_joint', 'shoulder_lift_joint',
            'upperarm_roll_joint',
            'elbow_flex_joint',
            'forearm_roll_joint',
            'wrist_flex_joint', 'wrist_roll_joint',
            'l_gripper_finger_joint', 'r_gripper_finger_joint']
        self.joint_name_map = dict([(name, index) for index, name in enumerate(self.joint_names)])
        self.cur_joint_pos = np.zeros([len(self.joint_names)])
        self.cur_joint_vel = np.zeros([len(self.joint_names)])
        self.cur_joint_effort = np.zeros([len(self.joint_names)])

        self.cur_base_scan = np.zeros([662])
        self.cur_head_camera_depth_image = np.zeros([480,640], dtype=np.float32)
        self.cur_head_camera_rgb_image = np.zeros([480,640,3], dtype=np.uint8)

        logger.warn('FetchRobotGymEnv ROS node running')

    def base_scan_cb(self, data):
        # fmin replaces nans with 15
        self.cur_base_scan = np.fmin(np.array(data.ranges), 15.0)

    def head_camera_depth_image_cb(self, data):
        shape = [data.height, data.width]
        dtype = np.dtype(np.float32)
        npdata = np.fromstring(data.data, dtype=dtype).reshape(shape)
        npdata.strides = (data.step, dtype.itemsize)
        self.cur_head_camera_depth_image = np.fmin(npdata, 5.0)

    def head_camera_rgb_image_raw_cb(self, data):
        shape = [data.height, data.width, 3]
        dtype = np.dtype(np.uint8)
        npdata = np.fromstring(data.data, dtype=dtype).reshape(shape)
        npdata.strides = (data.step, dtype.itemsize*3, 1)
        self.cur_head_camera_rgb_image = npdata

    def joint_states_cb(self, data):
        for i in range(len(data.name)):
            name = data.name[i]
            jni = self.joint_name_map.get(name, -1)
            if jni >= 0:
                self.cur_joint_pos[jni] = data.position[i]
                self.cur_joint_vel[jni] = data.velocity[i]
                self.cur_joint_effort[jni] = data.effort[i]

class FetchRobotGymEnv:
    def __init__(self, api, obs_joints=True, obs_lidar=False, obs_head_depth=True, obs_head_rgb=False, act_torques=True):
        self.api = api
        self.obs_joints = obs_joints
        self.obs_lidar = obs_lidar
        self.obs_head_depth = obs_head_depth
        self.obs_head_rgb = obs_head_rgb
        self.act_torques = act_torques

        obsparts = []
        if self.obs_joints:
            obsparts.append(Box(-4, +4, self.api.cur_joint_pos.shape))
        if self.obs_lidar:
            obsparts.append(Box(0, +16.0, self.api.cur_base_scan.shape))
        if self.obs_head_depth:
            obsparts.append(Box(0, 5.0, self.api.cur_head_camera_depth_image.shape))
        if self.obs_head_rgb:
            obsparts.append(Box(0, 255, self.api.cur_head_camera_rgb_image.shape))
        self.observation_space = Tuple(obsparts)

        actparts = []
        if self.act_torques:
            actparts.append(Box(-4.0, +4.0, [len(self.api.joint_names)]))
        self.action_space = Tuple(actparts)

        self.reward_range = [-1, +1]
        self.tickrate = rospy.Rate(10)

    def _get_obs(self):
        obsparts = []
        if self.obs_joints:
            obsparts.append(self.api.cur_joint_pos)
        if self.obs_lidar:
            obsparts.append(self.api.cur_base_scan)
        if self.obs_head_depth:
            obsparts.append(self.api.cur_head_camera_depth_image)
        if self.obs_head_rgb:
            obsparts.append(self.api.cur_head_camera_rgb_image)
        return tuple(obsparts)

    def reset(self):
        if self.api.move_group is not None:
            targ = np.zeros([len(self.api.joint_names)])
            self.api.move_group.moveToJointPosition(self.api.joint_names, targ, wait=True)
        return self._get_obs()

    def step(self, action):
        # Wait here for 100mS since the last time
        t0=time.time()
        self.tickrate.sleep()
        t1=time.time()
        logger.info('sleep %s %s', t1-t0, t1)
        if self.api.move_group is not None:
            self.api.move_group.moveToJointPosition(self.api.joint_names, action, wait=False)

        obs = self._get_obs()
        reward = 0.0
        done = False
        info = {}
        return obs, reward, done, info

    def render(self, mode='human', close=False):
        return self.api.cur_head_camera_rgb_image

    def close(self):
        pass



def make_env(name):
    if name == 'FetchRobot-v0':
        return FetchRobotGymEnv(fetch_api)
    elif name == 'FetchRobotRGB-v0':
        return FetchRobotGymEnv(fetch_api, obs_head_depth=False, obs_head_rgb=True)
    else:
        raise Exception('Unknown env name %s' % name)

if __name__ == '__main__':
    logger.info('Creating FetchRobotGymEnv node. argv=%s', sys.argv)
    rospy.init_node('FetchRobotGymEnv') # After this, logging goes to ~/.ros/log/FetchRobotGymEnv.log
    logger.info('Starting API')
    fetch_api = FetchRobotApi()
    if 1:
        logger.info('Starting Zmq')
        zmqs = server.GymProxyZmqServer('tcp://0.0.0.0:6911', make_env)
        logger.info('Creating Zmq thread')
        zmqt = threading.Thread(target=zmqs.run_main)
        zmqt.daemon = True
        logger.info('Starting Zmq thread')
        zmqt.start()
    logger.info('Spinning...')
    rospy.spin()
