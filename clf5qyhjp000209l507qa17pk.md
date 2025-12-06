---
title: "Updating the Robot Description"
datePublished: Sun Mar 05 2023 18:46:09 GMT+0000 (Coordinated Universal Time)
cuid: clf5qyhjp000209l507qa17pk
slug: updating-the-robot-description
cover: https://cdn.hashnode.com/res/hashnode/image/upload/v1678645312248/df0aeb28-113c-4811-8281-668d6514e1cb.jpeg

---

This week, I was supposed to be implementing the [Parameter Server on micro-ROS](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/parameters/), but I decided to put that aside for a while and work on updating the [robot description](https://github.com/adityakamath/akros/tree/main/akros_description) instead. I made [some hardware changes](https://adityakamath.github.io/2022-08-05-akros2-firmware-part-3/) since moving to ROS 2 (new RPi 4 case, changed LIDAR position) a while ago, but never got the chance to update the robot description files. I divided this into four steps:

1. Update the [akros\_description package](https://github.com/adityakamath/akros/tree/main/akros_description), which is still using [ROS 1 Noetic](http://wiki.ros.org/noetic)
    
2. Update the [URDF](https://github.com/adityakamath/akros/blob/main/akros_description/urdf/akros.urdf.xacro) and [meshes](https://github.com/adityakamath/akros_3d_assets/tree/akros) to reflect the hardware changes
    
3. Update the [mesh publisher](https://github.com/adityakamath/akros/blob/main/akros_description/scripts/mesh_publisher.py), which publishes URDF mesh elements as a [MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html) for remote visualization, using [Foxglove Studio](https://foxglove.dev/studio) for example
    
4. Move the [ROS 1 launch file](https://github.com/adityakamath/akros/blob/main/akros_description/launch/akros_description.launch) to a python launch file for ROS 2, and include it in [akros2\_bringup\_launch.py](https://github.com/adityakamath/akros2_bringup/blob/humble/launch/akros2_bringup_launch.py)
    

I started by creating a new [akros2\_description](https://github.com/adityakamath/akros2_description/tree/humble) package in [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) and setting up the [updated URDF](https://github.com/adityakamath/akros2_description/blob/humble/urdf/akros2.urdf). I already had the URDF from before the hardware changes, so I only had to [update the meshes](https://github.com/adityakamath/akros_3d_assets/tree/akros2) and change the positions of the links in the URDF file.

I did have some trouble with the [realsense\_ros library for ROS 2](https://github.com/IntelRealSense/realsense-ros), while setting up the [T265 tracking camera](https://www.intelrealsense.com/tracking-camera-t265/). Earlier versions (with ROS 1) allowed me to do the following things:

* Provide wheel odometry to the T265 (using a calibration JSON file) and get the fused `odom_frame->base_link` transform.
    
* OR disable the odometry transform (instead publish odometry messages), and use an [Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) (EKF, using [robot\_localization](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html#)) on ROS 1 to fuse the T265 and wheel odometry messages. This EKF node then provided the `odom_frame->base_link` transform.
    

However, both these features (previously set as launch arguments from the launch file / CLI) are now deprecated in the [latest ROS 2 release](https://github.com/IntelRealSense/realsense-ros/releases/tag/4.51.1). Intel doesn't plan on supporting it either, especially since the hardware has been discontinued. As a result, I now have the following in ROS 2:

* T265 provides the `odom_frame->base_link` transform directly using [visual-inertial odometry (VIO)](https://www.intelrealsense.com/visual-inertial-tracking-case-study/) and does not include wheel odometry.
    
* Wheel odometry is provided by the [micro-ROS node](https://github.com/adityakamath/akros2_firmware), and is linked to `enc_odom_frame` (as the parent link), with `base_footprint` as the default child link.
    

Going forward, I still need to figure out how I can fuse them both - either I update realsense\_ros by myself to include wheel odometry using a launch argument, or I run an EKF node but I'm unsure how the transform tree will look now that I cannot disable the odometry transform from the T265.

## Mesh Publisher

For now, everything worked as expected, so I decided to work on the [mesh publisher](https://github.com/adityakamath/akros2_description/blob/humble/akros2_description/mesh_publisher.py). When working remotely with ROS 2 running on the robot and Foxglove Studio running on my Windows laptop (without ROS 2 installed), I can visualize all topics and transforms, but not URDFs. So, I created a [MarkerArray publisher](https://github.com/adityakamath/akros/blob/main/akros_description/scripts/mesh_publisher.py), that publishes the meshes of the robot's components as an array of markers, with each mesh linked to its respective URDF link. But this was done a while ago and was still using ROS 1 Noetic. However, migrating to ROS 2 was straightforward.

When I first ran the mesh publisher, I connected a monitor to my RPi to visualize it on RViz2. The marker array transforms looked fine (although it seems RViz2 does not like transparent markers), as seen in the image below.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1678637615866/d7f55754-333e-4182-8bc8-5230e3bcdb1f.jpeg align="center")

But the meshes were all rotated when I tried visualizing it on Foxglove, as seen in the image below.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1678639858009/3465557e-edad-47e3-8c00-3c65c20b5556.jpeg align="center")

I struggled with this for a while, before realizing that meshes in the 3D panel default to the y-up orientation, while [ROS (and my design) uses the z-up orientation](https://www.ros.org/reps/rep-0105.html).

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1678639871436/008891d3-ffc9-44c1-9272-2eb9b715e8ca.jpeg align="center")

After changing the default orientation in the 3D panel settings, and a quick restart, the problem was solved.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1678639885006/c00a28af-6ba6-4bb0-aa7f-2e17ad05c2d5.jpeg align="center")

## Joint State Publisher

While testing my implementation, I ran two other nodes alongside the mesh publisher:

* [robot\_state\_publisher](https://github.com/ros/robot_state_publisher) to view the actual robot description on RViz2. This node also subscribes to the combined [JointState message](https://docs.ros2.org/galactic/api/sensor_msgs/msg/JointState.html), which moves the corresponding joints in the URDF according to the provided velocity and position.
    
* [joint\_state\_publisher](https://github.com/ros/joint_state_publisher) to subscribe to individual joint states (motor states in this case - position, velocity) and publish the combined JointState message to which the robot\_state\_publisher subscribes.
    

Originally, in ROS 1, I was using the mesh publisher node instead of the joint\_state\_publisher to publish the JointState message. The node subscribed to the measured speeds of each motor from the [ROSSerial firmware](https://github.com/adityakamath/arduino_sketchbook_ros/tree/main/akros_holo_drive) and calculated the speed in rad/s and the position in radians for each joint. This was then published as a combined JointState message to robot\_state\_publisher.

In ROS 2, I decided to do this directly from the [micro-ROS firmware](https://github.com/adityakamath/akros2_firmware). So, instead of publishing the measured speeds of each motor, the micro-ROS node would now publish a combined JointState message with the measured speeds and positions of each motor. I made the necessary changes to the micro-ROS firmware, which can be seen [here](https://github.com/adityakamath/akros2_firmware/blob/akros2_humble/akros2_firmware.ino).

I was originally also publishing the required speeds for each motor, so I also changed that to publish another JointState message but with the required speeds and positions of each joint. While it is not used anywhere, it is useful for me as a debugging and controller tuning tool.

## Launch File

Finally, I set up the launch file. I created a launch argument to switch between the micro-ROS node and the joint\_state\_publisher to publish the combined joint states. While I did start by checking out [rosetta\_launch](https://github.com/MetroRobots/rosetta_launch) (an amazing resource about writing ROS 2 launch files), I eventually ended up just asking [ChatGPT](https://chat.openai.com/) to write it for me. Since ROS and its community-developed resources are open-source and mostly freely available on GitHub, ChatGPT seems to be trained really well to write simple ROS (1, 2, and micro-ROS) code. Once the [akros2\_description launch file](https://github.com/adityakamath/akros2_description/blob/humble/launch/akros2_description_launch.py) was ready, I then updated the [akros2\_bringup launch file](https://github.com/adityakamath/akros2_bringup/blob/humble/launch/akros2_bringup_launch.py) to include the description launch file and set the correct launch arguments. Once ready, I could launch everything (sensors, motor control using micro-ROS, visualization nodes, and the robot description including the mesh publisher) using a single launch file. Here is the resulting visualization on Foxglove Studio:

%[https://www.youtube.com/watch?v=W2UfhBy7NNU] 

### Update:

I've been using this for nearly 2 weeks now, but the visualization hasn't always been reliable. I could only reproduce the visualization from the video around 30% of the time I tried the launch file. I saw two main issues:

1. Sometimes the transforms didn't load correctly. The 3D panel on Foxglove Studio looked like the below image. The top panel, [LD06](https://github.com/linorobot/ldlidar) and the T265 were shown as separate transform trees and not connected to the rest of the robot. This mainly (90% of the time) happened in two situations:
    
    * Foxglove Studio was kept running, and the launch file was terminated and re-launched after some time (usually a few minutes)
        
    * Foxglove Studio was closed and re-opened in a few minutes while the [bringup launch file](https://github.com/adityakamath/akros2_bringup/blob/humble/launch/akros2_bringup_launch.py) was kept running.
        
    
    ![](https://cdn.hashnode.com/res/hashnode/image/upload/v1679083806337/7e68721d-4306-4c37-9a58-33faaffbeda4.jpeg align="center")
    
    In other instances, the wheels (defined as continuous joints) were not displayed at all if the JointState message wasn't published by the micro-ROS node.
    
2. While the joint states were published correctly, the visualization wasn't always as smooth as in the above video. Sometimes the visualization was jerky, sometimes the wheels didn't move at all.
    

For background, I recently moved from [ROSBridge](https://foxglove.dev/docs/studio/connection/ros2#rosbridge) to [Foxglove Bridge](https://foxglove.dev/docs/studio/connection/ros2#foxglove-websocket) to connect ROS 2 data from the robot to Foxglove Studio. I decided to try using [rosbridge\_server](https://github.com/RobotWebTools/rosbridge_suite/tree/ros2/rosbridge_server) instead (I hadn't tried it in ROS 2 so far, only in ROS 1 Noetic) and see if it fares any better. It did, to some extent.

The transform tree issue from above was solved, both when Foxglove Studio was closed/re-opened and also when the launch file was terminated/re-launched. The wheels still needed JointState message to be published, but unlike foxglove\_bridge, the meshes were updated as soon as the micro-ROS node was enabled. I still need to update the mesh publisher to publish one JointState message for the wheels when it is launched so that the meshes are initialized.

The second issue with the visualization is not completely solved but there's some improvement. The wheels now move correctly according to the joint states, however, there is significant and visible latency now, up to nearly 0.5 seconds at times. There's also a little bit of latency with the laser scan, which is quite visible when turning. Results can be seen in the video below (all the transforms are correctly published, I had disabled them when recording this example):

%[https://www.youtube.com/watch?v=ioM_6P7OYRg] 

For now, I have decided to go with rosbridge\_server as my means of connecting live data to Foxglove Studio. But I hope these issues are fixed in later updates of [foxglove-bridge](https://github.com/foxglove/ros-foxglove-bridge). They've done a great job in reducing latency because when it works, it works really well and with minimal latency.

However, my priority for visualizing the robot is a correct and reliable transform tree, any latency is fine as long as data is not being dropped. I guess that was the problem with the first issue with foxglove\_bridge: I'm using it to publish a lot of data - meshes, transforms, laser scans and sometimes camera streams, and possibly a few messages were being dropped to provide such low latency.

On the plus side:

%[https://twitter.com/kamathsblog/status/1637159680337117185] 

### Update (October 2023)

Seems like my issues with Foxglove Bridge have been resolved. I no longer have the issue with the transform tree, and it seems to work even over a VPN connection. Reverting back to Foxglove Bridge for now, although I have still kept the option to switch to ROSBridge if I want...

## Next Up

Next up I want to stick to my earlier plan and set up the [Parameter Server on micro-ROS](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/parameters/). Like I mentioned last time, I also want to explore [tracing for ROS 2](https://github.com/ros2/ros2_tracing) and [micro-ROS](https://micro.ros.org/docs/tutorials/advanced/tracing/). As for [the book for February](https://www.goodreads.com/book/show/40876575-utopia-for-realists), I was able to finish it (although a few days late). For March, I chose [this book](https://www.goodreads.com/book/show/36062756-science-ish?ref=nav_sb_ss_1_9). And speaking of books, I also received [this one](https://www.amazon.com/Concise-Introduction-Robot-Programming-ROS2/dp/1032264659) by [Francisco Martin Rico](https://twitter.com/Fmrico) during the week:

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1678646675451/72372d3e-6cbb-45d0-ba09-57cedd25efff.jpeg align="center")

I skimmed through the chapters, and it seems to have a lot of relevant information despite being so thin (as the title suggests, it is definitely concise). It looks like an amazing resource, that I will be referring to throughout the year...