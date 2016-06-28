# rosbridge
Bridge from Gym to ROS robots.

Runs as a ROS node (ie, start it with `roslaunch rosbridge fetch_proxy.launch`

Listens on a ZMQ socket and converts the action space to ROS commands, and the ROS sensors to an observation space
suitable for OpenAI Gym.

### Challenges:
 * How to capture and replay rviz output? Can render return some sort of key into the log which can be rendered later on demand rather than in real-time?

## Fetch high level
 * See [Docs](http://docs.fetchrobotics.com/), especially the [API](http://docs.fetchrobotics.com/api_overview.html)

### Action and observation space
 * Observation is a tuple of
   - joint angles [Nj x 1]
   - kinematic coordinates [Nb x 12]
   - measured joint torques [Nj x 1]
   - lidar scan [Ns x 1]
   - camera [Nh x Nv]
 * Action is a vector of
   - joint torques [Nj x 1]
   - forward velocity [1]
   - rotational velocity [1]
   - (other action spaces might be added)


### Ros Topics
  * Subscribe to `head_camera/depth_registered/points`, of type   [sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)
    - Note: `head_camera/depth/*` not available in simulator.
  * Subscribe to `base_scan` of type [sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)
  * Subscribe to `imu` of type [sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)

  * Write to `arm_with_torso_controller/follow_joint_trajectory` of type [control_msgs/FollowJointTrajectory](http://docs.ros.org/api/control_msgs/html/action/FollowJointTrajectory.html)
    - Or maybe a higher level interface through Moveit.
  * Write to `cmd_vel` of type [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
  * Write to `head_controller/point_head` of type [control_msgs/PointHead](http://docs.ros.org/api/control_msgs/html/action/PointHead.html)
  * Write to `gripper_controller/gripper_action` of type [control_msgs/GripperCommand](http://docs.ros.org/api/control_msgs/html/action/GripperCommand.html)
    - turn a single grip strength number into position and effort, where <=0 means open and >0 means closed with corresponding effort

## Fetch low level
  * Update position targets as fast as possible?


# Managing Fetch
 * rviz crashes without a real monitor hooked up, unless you give it some fake data. Seems like a bug in Ogre. Use `rostools/setup-display.sh` to set it up



Building the enhanced `fetchrobotics/robot_controllers`
```
cd $HOME
mkdir -p catkin_ws/src
cd catkin_ws/src
ln -s /home/tlb/fetchrobotics/fetch_robots/fetch_bringup .
ln -s /home/tlb/fetchrobotics/robot_controllers .
wstool init
cd ..
. devel/setup.bash
catkin_make
catkin_make install
sudo service robot stop && sleep 5 && sudo service robot start
sudo less /var/log/upstart/robot.log
```

If the circuit breaker blows due to excess current draw, you can reset with
```
rosservice call /arm_breaker true  # to check
rosservice call /arm_breaker false && rosservice call /arm_breaker true  # to reset
```
