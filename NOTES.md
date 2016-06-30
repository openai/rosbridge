
# Fetch


## ROS Topics
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
