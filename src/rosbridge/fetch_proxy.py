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

fetch_api = None

class FetchRobotApi:
    def __init__(self):

        # See http://docs.fetchrobotics.com/robot_hardware.html#naming-conventions
        self.joint_names = [
            'torso_lift_joint',
            #'bellows_joint',
            'head_pan_joint',
            'head_tilt_joint',
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'upperarm_roll_joint',
            'elbow_flex_joint',
            'forearm_roll_joint',
            'wrist_flex_joint',
            'wrist_roll_joint',
            'l_gripper_finger_joint',
            #'r_gripper_finger_joint',
            ]
        self.joint_name_map = dict([(name, index) for index, name in enumerate(self.joint_names)])
        self.cur_joint_pos = np.zeros([len(self.joint_names)])
        self.cur_joint_vel = np.zeros([len(self.joint_names)])
        self.cur_joint_effort = np.zeros([len(self.joint_names)])

        self.cur_base_scan = np.zeros([662])
        self.cur_head_camera_depth_image = np.zeros([480,640], dtype=np.float32)
        self.cur_head_camera_rgb_image = np.zeros([480,640,3], dtype=np.uint8)

        logger.info('Subscribing...')
        self.subs = []
        self.subs.append(rospy.Subscriber('/base_scan', LaserScan, self.base_scan_cb))
        self.subs.append(rospy.Subscriber('/joint_states', JointState, self.joint_states_cb))
        self.subs.append(rospy.Subscriber('/joint_states', JointState, self.joint_states_cb))
        self.subs.append(rospy.Subscriber('/head_camera/depth/image', numpy_msg(Image), self.head_camera_depth_image_cb))
        self.subs.append(rospy.Subscriber('/head_camera/rgb/image_raw', numpy_msg(Image), self.head_camera_rgb_image_raw_cb))
        logger.info('Subscribed')

        self.arm_effort_pub = rospy.Publisher('/arm_controller/weightless_torque/command', JointTrajectory, queue_size=2)
        #self.head_goal_pub = rospy.Publisher('/head_controller/point_head/goal', PointHeadActionGoal, queue_size=2)
        #self.gripper_ctl_pub = rospy.Publisher('/gripper_controller/command', GripperCommand, queue_size=2)

        self.head_point_client = actionlib.SimpleActionClient('head_controller/point_head', PointHeadAction)

        self.arm_move_group = MoveGroupInterface("arm", "base_link", plan_only = True)
        self.arm_trajectory_client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.arm_trajectory_client.wait_for_server()


        if 0:
            logger.info('Creating MoveGroupInterface...')
            self.move_group = MoveGroupInterface('arm_with_torso', 'base_link', plan_only = True)
            logger.info('Created MoveGroupInterface')
            if 0:
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


        logger.warn('FetchRobotGymEnv ROS node running')

        self.head_point_client.wait_for_server()
        logger.warn('FetchRobotGymEnv ROS node connected')

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
        """
        Handle a joint_states message. Messages can have subsets of the joints, hopefully non-overlapping.
        """
        for i in range(len(data.name)):
            name = data.name[i]
            jni = self.joint_name_map.get(name, -1)
            if jni >= 0:
                self.cur_joint_pos[jni] = data.position[i]
                self.cur_joint_vel[jni] = data.velocity[i]
                self.cur_joint_effort[jni] = data.effort[i]

    def move_to_start(self):

        # Look down
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = '/base_link'
        goal.target.point.x = 1.5
        goal.target.point.y = 0.0
        goal.target.point.z = -0.2
        goal.min_duration = rospy.Duration(0.5)
        logger.info('Point head to %s...', goal);
        self.head_point_client.send_goal(goal)
        logger.info('Point head sent')

        joints = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'upperarm_roll_joint',
            'elbow_flex_joint',
            'forearm_roll_joint',
            'wrist_flex_joint',
            'wrist_roll_joint',
        ]
        pose = [0.0, +0.8, 0.0, -0.8, 0.0, 0.0, 0.0]
        result = self.arm_move_group.moveToJointPosition(joints, pose, plan_only=True)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            if 0: logger.info('Got trajectory %s', result.planned_trajectory)
            follow_goal = FollowJointTrajectoryGoal()
            follow_goal.trajectory = result.planned_trajectory.joint_trajectory
            logger.info('sending trajectory to arm...')
            self.arm_trajectory_client.send_goal(follow_goal)
            result = self.arm_trajectory_client.wait_for_result()
            logger.info('arm followed trajectory %s', result)
        else:
            logger.warn('moveToJointPosition returned %s', result)
            return

        result = self.head_point_client.wait_for_result()
        logger.info('head followed trajectory %s', result)

        logger.info('sending empty arm goal')
        empty_goal = FollowJointTrajectoryGoal()
        self.arm_trajectory_client.send_goal(empty_goal)


        logger.info('Point head done')


    def set_weightless_torque(self, action):
        arm_joints = [
            ('shoulder_pan_joint', 1.57, 33.82),
            ('shoulder_lift_joint', 1.57, 131.76),
            ('upperarm_roll_joint', 1.57, 76.94),
            ('elbow_flex_joint', 1.57, 66.18),
            ('forearm_roll_joint', 1.57, 29.35),
            ('wrist_flex_joint', 2.26, 25.70),
            ('wrist_roll_joint', 2.26, 7.36),
        ]
        arm_efforts = [min(1.0, max(-1.0, action[self.joint_name_map.get(name)])) * torque_scale * 0.25 for name, vel_scale, torque_scale in arm_joints]
        arm_joint_names = [name for name, vel_scale, torque_scale in arm_joints]
        if 1:
            arm_joint_names.append('gravity_compensation')
            arm_efforts.append(1.0)
        arm_msg = JointTrajectory(joint_names=arm_joint_names, points=[JointTrajectoryPoint(effort = arm_efforts)])
        self.arm_effort_pub.publish(arm_msg)

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
        self.api.move_to_start()
        return self._get_obs()

    def step(self, action):
        # Wait here for 100mS since the last time
        t0=time.time()
        self.tickrate.sleep()
        t1=time.time()
        logger.info('sleep %s %s', t1-t0, t1)

        self.api.set_weightless_torque(action[0])

        obs = self._get_obs()
        reward = 0.0
        done = False
        info = {}
        return obs, reward, done, info

    def render(self, mode='human', close=False):
        # WRITEME: output this image to an avi encoder, return a compact URL reference
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
    fetch_api.move_to_start()
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
