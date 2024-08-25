---
title: "Visualizing Robots in Unity"
datePublished: Wed Apr 26 2023 19:08:54 GMT+0000 (Coordinated Universal Time)
cuid: clh3scj29000b09l3bcehg2ah
slug: visualizing-robots-in-unity
cover: https://cdn.hashnode.com/res/hashnode/image/upload/v1682720555965/14d2e89c-a75e-4a72-8148-d7ffee71367d.jpeg
tags: unity, robotics, unity3d, ros, ros2

---

Over the last few weekends, I've been trying to set up a visualization for AKROS2 on Unity. I have had some successes, and I think I have reached a state where I have correct transforms, and can visualize most of the topics quite correctly. In this post, I will talk about how I did all of it in Unity 2022.2.13 (please be warned that it might not work in previous versions of Unity, it does not work in the 2020 and 2021 versions for example).

%[https://twitter.com/kamathsblog/status/1642663040532217857] 

### Setting Up

First, setting up the [Unity environment](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/README.md). There are a few packages that need to be installed on Unity:

1. `TCP Connector`: This package enables the connection to a remote robot running ROS 2 (which is on the same Wi-Fi network as the computer running Unity). This can be installed on Unity using the steps provided [here](https://github.com/Unity-Technologies/ROS-TCP-Connector).
    
2. `URDF Importer`: This package enables the import of a URDF file into Unity. The importer also corrects the frames of reference from the Z-up orientation in ROS 2 to the Y-up orientation in Unity. This package can be installed using the steps provided [here](https://github.com/Unity-Technologies/URDF-Importer).
    
3. `Unity Robotics Visualization`: This package enables the visualization of ROS / ROS 2 topics in Unity. It can be installed using the steps provided [here](https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/main/com.unity.robotics.visualizations/Documentation~/README.md).
    

Finally, the last package [`ROS_TCP_Endpoint`](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) needs to be cloned in a ROS workspace and built like any other ROS package. Make sure the correct branch is cloned and built, as there are different ROS and ROS 2 branches. This package establishes the connection with the TCP Connector package on Unity.

With these packages installed, a new project can be created for the AKROS2 visualization.

### Importing the URDF

After setting up and creating a new project, it is time to import the URDF. First, the URDF and the associated meshes need to be added to the project in the `Assets` directory. This can be easily added by simply cloning the [`akros_3d_assets`](https://github.com/adityakamath/akros_3d_assets/tree/akros2_urdf) repository into the assets. Make sure to clone the [`akros2_urdf`](https://github.com/adityakamath/akros_3d_assets/tree/akros2_urdf) branch, since that's the only one that has a URDF. This URDF was generated using the Xacro files in the [`akros2_description`](https://github.com/adityakamath/akros2_description/tree/humble/urdf) package.

Next, the URDF can be imported by navigating to it from the Project window and selecting the `Import Robot from Selected URDF File` option upon right-clicking on the URDF file (`akros2` in the following image).

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1682717966623/e31f3b82-229b-4f58-b521-bc29dd9e6783.jpeg align="center")

Please use the setting shown in the following image when importing the URDF:

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1682718107557/cdab86ff-5275-4204-8e52-cfe1523929f6.jpeg align="center")

A few things can be observed - in our Xacro and corresponding URDF files, the `t265_pose_frame` is the root frame, instead of `base_link` (this was necessary to make the T265 camera work properly), and the origin of the world seems to be connected to the root frame. So, a few things need to be changed.

First, an [articulation body component](https://docs.unity3d.com/Manual/class-ArticulationBody.html) is added to the `t265_pose_frame`. This is done because of the [Unity Nav2 SLAM example](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example), where the root frame of the robot is also an articulated body. It seems to work without issues for now.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1682718267609/8515c00f-74f3-4d76-911a-b6bc7d8df6a9.jpeg align="center")

Next, `base_link` needs to be set as the root frame. This can be done by unselecting the `Is Base Link` option for the `t265_pose_frame` and selecting the same option for the `base_link`.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1682718397861/c30a4e99-85fb-4961-a1d5-d85bb073294f.jpeg align="center")

Finally, it can be seen that the `t265_pose_frame` is still situated at the origin, so the tree needs to be translated such that the origin is situated at the `base_link` frame.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1682718957658/145c7801-f470-4480-a403-a1765f50e99e.jpeg align="center")

In this case, it can be done by setting the `t265_pose_frame` translation to y:+0.115, z: 0.094.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1682718982305/afdb902b-7645-4ecc-99b2-61f649a04bae.jpeg align="center")

Next, before continuing make sure that gravity is enabled for each frame and articulation body. Other settings are configured automatically by the URDF importer.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1682718542674/118b9c18-c15b-4c54-861f-6c9f6448528d.jpeg align="center")

This brings us to the next issue - in the current state, with gravity enabled and the play button pressed, the robot drops down (and does not stop). This is because there is no plane underneath the robot. So, the final thing to do is add a plane and configure the transform so that it is below the robot.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1682719093878/8e7bf9df-4a14-49bb-bfba-1acce6a3cac2.jpeg align="center")

In my case, the robot is levitating a few millimeters above the plane, which is okay as the robot drops onto the plane when the play button is pressed.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1682788409518/8544a5a7-c846-4036-ae6c-da5b065ad1a0.jpeg align="center")

### Visualizing topics in Unity

First, to set up a visualization of topics, the [`DefaultVisualizationSuite`](https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/main/com.unity.robotics.visualizations/Documentation~/README.md#using-the-inspector) needs to be added as an object in our scene. This can be done by navigating to the [`Unity Robotics Visualization`](https://github.com/Unity-Technologies/ROS-TCP-Connector/tree/main/com.unity.robotics.visualizations) directory under Packages in the Project window. From there, the `DefaultVisualizationSuite` prefab can be dragged into the scene.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1682719194332/856446d9-3da6-4bd0-8b3a-e0a91f88265e.jpeg align="center")

This package includes a list of topics and message types that can be visualized in Unity. These visualizations can be configured by selecting the specific message type under the `DefaultVisualizationSuite` prefab and configuring options like topic name, and frame ID to name a few.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1682719295559/d176ea98-4c83-4420-a5ec-dd4f5be1f88a.jpeg align="center")

I first configured the following topics:

* [`geometry_msgs/Twist`](https://docs.ros2.org/latest/api/geometry_msgs/msg/TwistStamped.html) : Configured the script with the topic name and frame ID
    
    ![](https://cdn.hashnode.com/res/hashnode/image/upload/v1682719675210/ef97b553-7eea-4563-b910-f247bc6ff0bd.jpeg align="center")
    
* [`nav_msgs/Odometry`](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html) : Configured the script with wheel odometry and tf topic names. This is only the wheel encoder odometry published by the micro-ROS node. The actual `odom->base_link` transform is provided by the T265 cam
    
    ![](https://cdn.hashnode.com/res/hashnode/image/upload/v1682719743189/bd8f736f-3563-4747-885f-d4e98f9f96ea.jpeg align="center")
    
* [`sensor_msgs/Imu`](https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html) : Configured the script with IMU and tf topic names.
    
    ![](https://cdn.hashnode.com/res/hashnode/image/upload/v1682719831922/138641d9-745f-4725-afbb-da81cbd31ad3.jpeg align="center")
    
* [`sensor_msgs/LaserScan`](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html) : Configured the script with the laser scan and tf topic names. The visualization script does not allow `Inf` or `NaN` values, which my laser scan topic does publish, so I had to filter the laser scan topic using [laser scan chain filter](http://wiki.ros.org/laser_filters) plugins: [`LaserScanRangeFilter`](http://wiki.ros.org/laser_filters#LaserScanRangeFilter) which limits the range of the scan between thresholds and anything outside the range is set to `NaN`, and then the [`InterpolationFilter`](http://wiki.ros.org/laser_filters#InterpolationFilter) that sets any (invalid) `NaN` values to a valid interpolation of neighboring valid values. The visualization script is configured with the filtered laser scan topic name.
    
    ![](https://cdn.hashnode.com/res/hashnode/image/upload/v1682719905052/948699fb-718e-4d5f-ace8-1b7bcfe80ea7.jpeg align="center")
    

Finally, it was time to run the visualization. I started off by launching the robot, and in a separate terminal, launching the [TCP endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) script:

```bash
$ ros2 launch ros_tcp_endpoint endpoint.py
```

On Unity, I went to `Robotics > ROS Settings` and configured the IP address of the robot and the port number of the TCP endpoint.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1682719983704/12bf95ab-9a3f-4ef9-b45c-8adf09b13863.jpeg align="center")

Next, I connected to the TCP endpoint by pressing the play button. [A Heads-up Display (HUD)](https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/main/com.unity.robotics.visualizations/Documentation~/README.md#the-hud) is shown in the Game view, from where you can choose the topics and transforms to show:

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1682791174847/80c5db85-9be9-44a8-91fe-73e2f9126af1.jpeg align="center")

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1682791197026/672313e3-1725-4b08-950d-51eac67cf8c2.jpeg align="center")

A better visualization can be seen in the Scene view, where you can change camera angles and zoom in or out.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1682795859207/edfdb818-87d8-43bb-9c34-a39ff9331872.jpeg align="center")

### Challenges

While most of the visualization seems to work, I observed a few issues:

1. **Fixed frames**: The `base_link` frame is fixed to the plane and I can observe `odom_frame` (T265 odometry frame) and `enc_odom_frame` (wheel encoder odometry frame) moving around when I move the robot. For now, I am using `odom_frame` as the main odometry frame, and I would like it to be fixed to the plane so that I can see the robot moving.
    
2. **Twist rotation**: When the robot is rotated in a particular direction, the twist visualization shows the opposite direction of rotation. When translating the robot, the visualization is correct. I am not sure what is happening here.
    
3. **JointState messages**: I also tried visualizing the joint states (position and velocity) for each motor. However, the motors rotate around a different axis altogether, have a look:
    

%[https://twitter.com/kamathsblog/status/1642663082278150144] 

### Next Up:

I will get some help from a colleague at work (who knows Unity) to at least try and fix the issues there. Hopefully, this can be a starting point for a similar project at work. I meanwhile want to move on to using [Gazebo](https://gazebosim.org/home) instead. I want to try and set it up on WSL2 on my laptop. I want to set up [ros2\_control on Gazebo](https://github.com/ros-controls/gazebo_ros2_control), so this should help in the efforts.

I was also able to install ROS 2 and micro-ROS on my Raspberry Pi Zero 2 W. I have also ordered [Pimoroni's](https://shop.pimoroni.com/) new [Inventor Hat Mini](https://shop.pimoroni.com/products/inventor-hat-mini), and I plan on using it to experiment with [ros2\_control](https://control.ros.org/master/index.html) and its [differential drive controller](https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html). Once I am comfortable working with ros2\_control, my next goal would be to write a mecanum drive controller and hardware interfaces for the AKROS2 robot.

I also spent some time during the long weekend playing with the [Wio Terminal and trying to get it working with micro-ROS over WiFi](https://qiita.com/MAEHARA_Keisuke/items/bf08aa46db5e17007c77). I tried all kinds of different things over a few days but nothing worked - I was unable to connect to my access point. However, I was able to get it working with micro-ROS over serial.

%[https://twitter.com/kamathsblog/status/1650172006669733890]