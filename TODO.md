# Rosbridge TODO


### WIP
  * get catkin package files back from container
   - or run catkin on laptop. Prolly better within container
  * Figure out where log files will go.
  * Move ProxyServer and ProxyClient to a gym/utils class

### Architecture issues
  * Integrate into gym or separate package? Plugin? See how pachi does it.
  * Probably a separate long-lived process that I connect to with a websocket. One at a time (others get rejected)
  * Need two interfaces: high- and low-level. High uses (control_msgs/FollowJointTrajectory). Not clear how to get low.
  * Show sketch of operation around #robotics.
    - also for Dex4
  * How to do logging & replay? Leave up to agent?
  * How to allocate robot among users?
    - Lock server. (Also for dex4, darwin, etc. Need mask to select particular robots. And UI to reserve).
  *

### Functionality issues
  * What does reset do?
  *


### Tidying
  * Rearrange Dockerfile installs
  * Since we'll want to upgrade ros frequently, split out X11 and such deps before it
  * How do I pin versions with apt-get?
  * Use node.js 6?
  *
