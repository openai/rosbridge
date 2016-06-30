# rosbridge
A service implementing a bridge from Gym to ROS robots. Currently supports the [Fetch Research Robot](http://docs.fetchrobotics.com/)

 - Runs as a [ROS](http://www.ros.org) node (ie, start it with `roslaunch rosbridge fetch_proxy.launch`.

 - Listens on a [ZMQ](http://api.zeromq.org) socket, which an [OpenAI Gym](http://gym.openai.com) `ProxyClient` connects to.

 - Converts the action space to ROS commands, and the ROS sensors to an observation space.

## Fetch Research robot Environments
 * See [Docs](http://docs.fetchrobotics.com/), especially the [API](http://docs.fetchrobotics.com/api_overview.html)

### FetchRobot-v0: Action and observation space
 * Observation is a tuple of
   - joint angles [Nj x 1]
   - kinematic coordinates [Nb x 12]
   - measured joint torques [Nj x 1]
   - camera [Nh x Nv] floats
 * Action is a vector of
   - joint torques [Nj x 1]
   - (other action spaces might be added)

### FetchRobotRGB-v0: Action and observation space
* Observation is a tuple of
  - joint angles [Nj x 1]
  - kinematic coordinates [Nb x 12]
  - measured joint torques [Nj x 1]
  - camera [Nh x Nv x 3] uint8s
* Action is a vector of
  - joint torques [Nj x 1]
  - (other action spaces might be added)


### Installation on a Fetch:
Building the enhanced `fetchrobotics/robot_controllers`
```sh
cd $HOME
mkdir -p catkin_ws/src
cd catkin_ws/src
ln -s ~/openai/fetch_robots/fetch_bringup .
ln -s ~/openai/robot_controllers .
wstool init
cd ..
. devel/setup.bash
catkin_make
catkin_make install
```

Configuration: edit /etc/ros/indigo/robot.launch to look more like [fetch.launch from the torque-control branch of fetch_robots](https://github.com/openai/fetch_robots/blob/torque-control/fetch_bringup/launch/fetch.launch)

Edit /etc/init.robot.conf to look like:
```sh
env ROS_LOG_DIR=/var/log/ros

start on roscore_is_up
stop on roscore_is_down

respawn

script
    #exec su ros -c ". /opt/ros/indigo/setup.bash && roslaunch /etc/ros/indigo/robot.launch"
    exec su ros -c ". /home/tlb/catkin_ws/devel/setup.bash && roslaunch /etc/ros/indigo/robot.launch"
end script
```

Restart:
```sh
sudo service robot stop && sleep 5 && sudo service robot start
sudo less /var/log/upstart/robot.log # see if everything worked
```

If the circuit breaker blows due to excess current draw, you can reset with
```sh
rosservice call /arm_breaker true  # to check
rosservice call /arm_breaker false && rosservice call /arm_breaker true  # to reset
```
