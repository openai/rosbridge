**Status:** Archive (code is provided as-is, no updates expected)

# rosbridge
**Warning: abandoned in Oct 2016, when we changed to a different system. It might be a good starting point for using a Fetch robot from Gym, but nobody at OpenAI is maintaining it**

A service implementing a bridge from Gym to ROS robots. Currently supports the [Fetch Research Robot](http://docs.fetchrobotics.com/)

 - Runs as a [ROS](http://www.ros.org) node (ie, start it with `roslaunch rosbridge fetch_proxy.launch`.

 - Listens on a [ZMQ](http://api.zeromq.org) socket, which an [OpenAI Gym](http://gym.openai.com) `ProxyClient` connects to.

 - Converts the action space to ROS commands, and the ROS sensors to an observation space.

## Fetch Research robot Environments
 * See [Docs](http://docs.fetchrobotics.com/), especially the [API](http://docs.fetchrobotics.com/api_overview.html)

### FetchRobot-v0: Action and observation space
 * Action is a tuple with
  - an 8x1 vector of joint torques, in range [-1 .. +1] corresponding to the following joints:
    - shoulder_pan_joint
    - shoulder_lift_joint
    - upperarm_roll_joint
    - elbow_flex_joint
    - forearm_roll_joint
    - wrist_flex_joint
    - wrist_roll_joint
    - l_gripper_finger_joint
 * Observation is a tuple of
  - An 8x1 vector of joint angles in range [-4 .. +4]
  - An 8x1 vector of joint velocities in range [-4 .. +4]
  - A 480x640 array of floats representing distance in meters

### FetchRobotRGB-v0: Action and observation space
 * Action: (same as FetchRobot-v0)
 * Observation is a tuple of
  - An 8x1 vector of joint angles in range [-4 .. +4] representing radians
  - An 8x1 vector of joint velocities in range [-4 .. +4] representing radians/sec
  - A 480x640x3 array of uint8s representing RGB pixel values


### Installation on a Fetch:
See https://github.com/openai/fetch-config for installation scripts
