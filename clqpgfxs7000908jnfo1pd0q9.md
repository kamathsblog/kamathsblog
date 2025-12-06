---
title: "2023: A year in review"
datePublished: Sun Dec 31 2023 23:00:00 GMT+0000 (Coordinated Universal Time)
cuid: clqpgfxs7000908jnfo1pd0q9
slug: last-update-of-2023
cover: https://cdn.hashnode.com/res/hashnode/image/upload/v1702559058546/16f7958a-f579-46a7-81ab-dcedcb2d46cd.png

---

In my last post of 2023, I'll try to summarize what I've been up to these last few months. Firstly, I've been playing around with some more sensors - a [VL53L7CX ToF Imager](https://www.st.com/en/imaging-and-photonics-solutions/vl53l7cx.html), a [Raspberry Pi Sense HAT](https://projects.raspberrypi.org/en/projects/getting-started-with-the-sense-hat), and a new controller - an [8BitDo SN30 Pro](https://www.8bitdo.com/sn30-pro-g-classic-or-sn30-pro-sn/) gamepad. Besides this, I also spent some time upgrading my AKROS2 robots - both the mecanum and omni-wheeled variants. I've covered a lot of different things in this post, so I've ordered this post in the same order as I did these different things. However, feel free to use the table of contents and read it in any order you want. Let's start with the sensors and the controller first...

## Sensors

### VL53L7CX ToF Imager

The VL53L7CX is an upgraded variant of the VL53L5CX Time-of-Flight (ToF) Imager from [ST Microelectronics](https://www.st.com/content/st_com/en.html). The new version has the same capabilities, providing 4x4 or 8x8 range values as output, but with a 90-degree FOV compared to the VL53L5CX which has a FOV of 65 degrees. I used this [breakout board from Pololu](https://www.pololu.com/product/3418/resources):

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1702562183394/3b274b19-538b-4341-b1c3-f81a976f9391.jpeg align="center")

The best part is that both sensors are compatible with the same driver software, so implementing it was easy. I simply renamed my [ROS 2](https://docs.ros.org/en/humble/index.html#) and [micro-ROS](https://micro.ros.org/) driver software to [tof\_imager\_ros](https://github.com/adityakamath/tof_imager_ros) and [tof\_imager\_micro\_ros](https://github.com/adityakamath/tof_imager_micro_ros), so there is no confusion with the naming.

I forgot to capture a video of this new sensor at work, so here's an old video using the [VL53L5CX breakout board from Sparkfun](https://www.sparkfun.com/products/18642) with micro-ROS. In terms of performance, both sensors behave identically, both with ROS 2 (directly connected to the RPi GPIO pins) and with micro-ROS (connected to the [Teensy Micromod](https://www.sparkfun.com/products/16402), which is connected to the RPi via USB):

%[https://www.youtube.com/watch?v=DetBMuRi-Do] 

### Raspberry Pi Sense HAT

Next up, I needed an IMU for a prototype, and the only one I had was already wired to my AKROS2 mecanum robot. So, I picked up the [Raspberry Pi Sense HAT](https://www.raspberrypi.com/documentation/accessories/sense-hat.html) v1 that I had purchased a while ago. The Sense HAT is a [Raspberry Pi HAT](https://www.okdo.com/blog/your-guide-to-hats-and-phats/), and a part of the [Astro Pi project](https://astro-pi.org/). It provides several sensors, a joystick, and an LED matrix, handy for displaying the sensor measurements. The v1 variant has a 9-axis IMU, Magnetometer, Pressure Sensor, Humidity Sensor, and a 5-button Joystick. These days, there's a [newer v2 variant](https://thepihut.com/products/raspberry-pi-sense-hat-astro-pi), with the addition of a color sensor (everything else remains the same). The Sense HAT v1 is not sold anymore (at least I couldn't find any online).

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1702632935531/dd3ae14f-ab59-4c63-a4e1-52d0b1ea9121.jpeg align="center")

I started with the [python-sense-hat](https://github.com/astro-pi/python-sense-hat/tree/master) library by Astro Pi, which provides access to all the different elements of the board. Using this library, I first created the [sensehat\_publisher](https://github.com/adityakamath/sensehat_ros/blob/main/sensehat_ros/sensehat_publisher.py) node, which publishes the following:

* `/imu`: IMU (accelerometer + gyroscope + magnetometer) readings, [converted from NED to ENU](https://github.com/mavlink/mavros/issues/49) if needed - [sensor\_msgs/msg/Imu](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html) message type
    
* `/mag`: Magnetometer readings in Teslas, [converted from NED to ENU](https://github.com/mavlink/mavros/issues/49) if needed - [sensor\_msgs/msg/MagneticField](https://docs.ros2.org/foxy/api/sensor_msgs/msg/MagneticField.html) message type
    
* `/pressure`: Pressure sensor readings converted to Pascals - [sensor\_msgs/msg/FluidPressure](https://docs.ros2.org/foxy/api/sensor_msgs/msg/FluidPressure.html) message type
    
* `/humidity`: Humidity sensor readings converted from percentage to the range \[0.0, 1.0\] - [sensor\_msgs/msg/RelativeHumidity](https://docs.ros2.org/foxy/api/sensor_msgs/msg/RelativeHumidity.html) message type
    
* `/temp_p`: Temperature readings from the pressure sensor in degrees Celcius - [sensor\_msgs/msg/Temperature](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Temperature.html) message type
    
* `/temp_h`: Temperature readings from the humidity sensor in degrees Celcius - [sensor\_msgs/msg/Temperature](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Temperature.html) message type
    
* `/joy`: 5-button joystick readings as an array of buttons - [sensor\_msgs/msg/Joy](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Joy.html) message type
    
* `/color`: Color sensor readings in the form of RGBA, each in the range \[0, 256\] - [std\_msgs/msg/ColorRGBA](https://docs.ros2.org/foxy/api/std_msgs/msg/ColorRGBA.html) message type
    

This node is implemented as a [lifecycle node](https://github.com/ros2/demos/blob/humble/lifecycle/README.rst), and its states are displayed on the LED display. Other than this, the LED display is not used for anything else. Eventually, I plan on implementing a [sensehat\_display\_handler](https://github.com/adityakamath/sensehat_ros/blob/main/sensehat_ros/sensehat_display_handler.py) node, which can visualize sensor and joystick measurements on the LED display.

I also created [sensehat\_node](https://github.com/adityakamath/sensehat_ros/blob/main/sensehat_ros/sensehat_node.py) that launches sensehat\_publisher in a [single-threaded executor](https://docs.ros2.org/latest/api/rclpy/api/execution_and_callbacks.html). Once the sensehat\_display\_handler is complete, I will also update the executor to launch both sensehat\_publisher and sensehat\_display\_handler nodes. All of these nodes, [config files](https://github.com/adityakamath/sensehat_ros/tree/main/config), a [launch file](https://github.com/adityakamath/sensehat_ros/blob/main/launch/sensehat_launch.py), and additional details about the implementation can be found within the [sensehat\_ros package](https://github.com/adityakamath/sensehat_ros) on GitHub.

## 8BitDo SN30 Pro

Next, I got an [8BitDo SN30 Pro](https://www.8bitdo.com/sn30-pro-g-classic-or-sn30-pro-sn/) controller. It is a compact little Bluetooth (or wired using USB-C) controller that is comfortable to use and can fit into my pocket. It can connect to a [Nintendo Switch](https://www.nintendo.com/us/switch/), which I was considering buying a few months ago. Now, I'm going for a [Steam Deck](https://store.steampowered.com/steamdeck), more details later.

It has the same number of inputs as the Stadia controller (standard layout with 4 special buttons) (see more info in my [earlier post about controllers](https://adityakamath.hashnode.dev/teleop-with-game-controllers)). However, there are a few differences:

1. The L2/R2 buttons are digital, even though they show up as axes in some of the modes.
    
2. The A/B and X/Y buttons are flipped.
    
3. One of the special buttons (bottom left corner) is considered a turbo enable/disable button in some of the modes. This allows some buttons to be set as turbo buttons.
    
4. Just like the Stadia controller, there are a few mystery buttons that appear in some modes
    

The controller can connect to the RPi using three of the four available modes, each with different features and mappings. More details can be found in the [mapping document](https://github.com/adityakamath/akros2_teleop/blob/humble/config/sn30pro_mapping.md) in [akros2\_teleop](https://github.com/adityakamath/akros2_teleop), where the mapping during each mode is shown.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1703782963647/d317f880-167d-42f1-bdda-507c7f4eb93f.jpeg align="center")

With the controller started in the [Android mode](https://manual.8bitdo.com/sn30pro/sn30pro_bluetooth_android.html) (my preferred option for the RPi), I then connected it to the RPi in the same way as the PS4 or the Stadia controller and ran the same node to test the joystick output. Using this I verified the mapping and set the mode and twist configs. I also tried the X-input mode, which does not always work for some reason. I did not investigate further. This mode also has haptic feedback enabled, but I also need to find a way to control the rumble and the LEDs. For now, the Android mode does everything I need and is definitely a good start.

# Updating AKROS2 Mecanum

Other than playing around with controllers and sensors, I also made some upgrades to the AKROS2 Mecanum robot. I hadn't worked with this robot for quite some time, so I first started by updating the operating system and packages. Next, I tried to test everything by launching the drivers for the [LD06 LIDAR](https://www.inno-maker.com/product/lidar-ld06/), the [T265 camera](https://dev.intelrealsense.com/docs/depth-and-tracking-cameras-alignment), the joystick controllers for teleoperation, and the [micro-ROS Agent](https://github.com/micro-ROS/micro-ROS-Agent) for the microcontroller client. There were some issues, which I then fixed, as I will explain in the following sections. After all the fixes and upgrades, this is what the AKROS2 Mecanum robot looks like:

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1702559108355/26ecc1d4-dbe1-467c-9932-f975b8800530.png align="center")

## Removing the T265 camera

During the bring-up, the [RealSense ROS](https://github.com/IntelRealSense/realsense-ros) package just did not work with the T265. I realized that the latest update to the package had dropped support for the T265, so I then tried to revert to an [older version](https://github.com/IntelRealSense/realsense-ros/releases/tag/4.51.1). This did not work either. I went through the [GitHub issues](https://github.com/IntelRealSense/realsense-ros/issues), and several different forums but did not find a solution. I even tried replacing the USB cable, but nothing worked. Eventually, I decided to retire the T265 and use it somewhere else (It still works on Windows).

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1702557017433/bb6474c0-1682-41f9-8889-51e500396e86.jpeg align="center")

It is a shame that Intel stopped supporting the T265. This should have been their flagship [RealSense](https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html) product, as there is no direct alternative for it on the market (As far as I know, this is the only Visual SLAM device out there). As for their other depth camera products, most roboticists are already moving on to other alternatives like [Luxonis](https://www.luxonis.com/) and [StereoLabs](https://www.stereolabs.com/) products.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1702557029847/10f32993-4a42-469d-87f2-b8b4da6c4b7f.jpeg align="center")

## Updating the micro-ROS firmware

Next up, I took a look into the [micro-ROS](https://micro.ros.org/) firmware to see what I could update. I started by removing the namespace from the firmware node, mainly because I realized that I [couldn't remap micro-ROS topics from the micro-ROS Agent on the host side](https://github.com/micro-ROS/micro-ROS-Agent/issues/210#issuecomment-1836970983). Also, this makes things much cleaner.

I also realized that although my IMU was already providing measurements in the ENU (East-North-Up) convention (according to [ROS REP-103](https://www.ros.org/reps/rep-0103.html)), this might change if I used a different IMU. So, I added a transform from [NED (North-East-Down) to ENU (swap x/y, negate z)](https://github.com/mavlink/mavros/issues/49) if needed. This transform is only performed when a boolean parameter is set. I also added this parameter to the parameter server, which now has 5 parameters - the PID gains, the max RPM scale, and the NED to ENU boolean parameter.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1703137880909/523dabf2-af42-43b1-8595-918b985c66d7.png align="center")

Finally, I updated the micro-ROS libraries and tried to flash the updated firmware to my [Teensy 4.1 breakout board](https://www.tindie.com/products/cburgess129/arduino-teensy41-teensy-41-expansion-board/). It did not work, the compilation failed with multiple errors. I first updated the [Arduino IDE 2](https://docs.arduino.cc/software/ide-v2) and [Teensy libraries](https://www.pjrc.com/arduino-ide-2-0-0-teensy-support/) to the latest version. I then deleted the [micro\_ros\_arduino](https://github.com/micro-ROS/micro_ros_arduino) pre-built library, re-installed it, added the [akros2\_msgs](https://github.com/adityakamath/akros2_msgs) message types, and tried to build the library again. This again failed with some errors. Eventually, after some digging, I realized that it worked in Linux so I tried [building the library in WSL2 - and this worked as expected](https://github.com/micro-ROS/micro_ros_arduino/issues/1490#issuecomment-1839365729). I copied the newly built library to Windows 11 and was able to flash the firmware using the latest Arduino IDE 2. I hope this issue is fixed in the future, it was a bit annoying to switch between Windows and WSL2. For people who are using the Arduino IDE on Linux, this should not be an issue.

Once the firmware was flashed, I tried it out and it worked perfectly. I then had the idea of adding an integer parameter to change the brightness of the status LEDs. I was able to flash the firmware after adding this parameter, but when I tried to run it, the firmware failed to create the parameter server and entered the error loop (flashes the LED twice repeatedly). I tried changing different configuration options in the micro\_ros\_arduino library but it still did not work. I have a suspicion that the parameter server is limited to only 5 parameters somewhere in the micro\_ros\_arduino library, but I haven't found it yet. I've raised an [issue on GitHub](https://github.com/micro-ROS/micro_ros_arduino/issues/1596) and hoping for a response soon (fingers crossed).

## Fusing IMU and Odometry

Now that I had the micro-ROS node working, I wanted to improve the odometry estimate by fusing it with IMU measurements using an [Extended Kalman Filter](https://medium.com/@serrano_223/extended-kalman-filters-for-dummies-4168c68e2117). Naturally, I decided to use the [robot\_localization](https://github.com/cra-ros-pkg/robot_localization) package, which has been my go-to EKF/UKF implementation since ROS 1. Before I could do this, I wanted to improve my IMU measurements, so I put it through a [Madgwick filter](https://github.com/CCNYRoboticsLab/imu_tools/tree/humble/imu_filter_madgwick).

The [IMU tools](https://github.com/CCNYRoboticsLab/imu_tools/tree/humble) package that I used also provides an implementation of the [Complementary Filter](https://github.com/CCNYRoboticsLab/imu_tools/tree/humble/imu_complementary_filter). They are both commonly used implementations to compute the orientation using angular velocities, accelerations, and optionally magnetometer readings (my [MPU9250](https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/) does not have a working magnetometer) from an IMU. While I chose the Madgwick filter, I haven't gone through the difference in performance between the two filters, so I do not have an answer to which one is better.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1703678621629/e4717562-1fab-41d8-bcb4-7c8106bd325c.png align="center")

I then created the config file for robot\_localization using [Automatic Addison's incredible blog](https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/) as a reference and used the [default covariance values](https://github.com/cra-ros-pkg/robot_localization/blob/ros2/params/ekf.yaml) for fusing the odometry and IMU measurements. I still need to find some time and [tune the covariances and the different options](https://docs.ros.org/en/melodic/api/robot_localization/html/configuring_robot_localization.html) that robot\_localization provides. For now, the generated filtered odometry values and the `odom->base_link` transform are sufficient. I did notice something though, turns out the robot\_localization package does not work if the odometry frame is called anything other than `odom`. This is a bit strange since in ROS 1 I have been able to rename the odometry frame. If anyone knows why this is happening in ROS 2, please help me out by answering my [Robotics Stack Exchange question here](https://robotics.stackexchange.com/questions/105659/renaming-odom-frame-on-robot-localization).

If you want to know more about Kalman Filters, here's one of my favorite resources on the topic: [How a Kalman Filter works, in pictures](https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/). However, this does not cover the Extended Kalman Filter. For a more detailed understanding, here's an amazing [video](https://www.youtube.com/watch?v=sVnXG-xUhzU&list=PLgG0XDQqJckmfolmM8y0GFS8l3x_r2p_S) + [blog](https://medium.com/@mathiasmantelli/kalman-filter-series-introduction-6d2e2b28d4cf) series by [Sharad Maheshwari](https://twitter.com/msharad19) and [Mathias Mantelli](https://twitter.com/MathiasMantelli):

%[https://www.youtube.com/watch?v=sVnXG-xUhzU&list=PLgG0XDQqJckmfolmM8y0GFS8l3x_r2p_S] 

## Adding a Camera

In the past, I used the T265 camera for two reasons - to provide odometry and IMU information and to stream video streams when needed. I'm now using a different IMU and fusing it with the wheel odometry has worked fine so far. But since removing the T265, I cannot stream video anymore. So, I bought [this tiny USB web camera](https://www.amazon.nl/-/en/dp/B09DSWRT7F?psc=1&ref=ppx_yo2ov_dt_b_product_details) from Amazon, that has a 5MP image sensor (with a 60-degree FOV), a microphone, and an LED for low-light conditions.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1702557053786/df6470d2-2946-42b3-99ee-778622f72761.jpeg align="center")

I then made a small mount for the camera, so that the assembly could fit onto the same mounting point as the T265. This can be seen in the CAD model below:

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1702557064465/0f5d0b99-2f10-493d-bd11-d41168267574.jpeg align="center")

Finally, I updated the [URDF](https://github.com/adityakamath/akros_3d_assets/tree/akros2_urdf/akros2_mecanum), which can be seen in the [Foxglove Studio](https://foxglove.dev/studio) visualization here:

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1702557085956/e3441672-9388-4a56-9521-9b1e6b60832a.jpeg align="center")

Next, I managed to use [v4l2\_camera node](https://gitlab.com/boldhearts/ros2_v4l2_camera) to publish the camera output as [ROS 2 Image messages](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html). Additionally, I also used the [image\_transport\_plugins](https://github.com/ros-perception/image_transport_plugins) to automatically republish the Image messages as compressed images. I did have an issue when launching the v4l2\_camera node as a [composable node](https://docs.ros.org/en/galactic/How-To-Guides/Launching-composable-nodes.html), where the [compressed image transport plugin](https://github.com/ros-perception/image_transport_plugins/tree/rolling/compressed_image_transport) did not automatically publish a compressed image stream. I even [asked for a solution on the Robotics Stack Exchange](https://robotics.stackexchange.com/questions/105825/camera-node-does-not-publish-compressed-image-when-launched-in-a-composable-cont), but no luck so far. For now, I am launching v4l2\_camera as a normal node, which works as expected.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1702557092649/3afaa70f-d515-482c-be82-4f8dab54618d.jpeg align="center")

There are currently a few issues that I still need to address: firstly, the addition of the camera meta-data which will be published as a [CameraInfo message](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html). Next, I want to be able to stream audio from the camera's on-board microphone. Finally, I want to control the LED using a [ROS 2 service](https://docs.ros.org/en/eloquent/Tutorials/Services/Understanding-ROS2-Services.html). This is still a work in progress and will carry forward to 2024.

For now, I can visualize the images remotely from Foxglove Studio. However, it only works without any lag when both my Foxglove Studio device and the robot are on the same access point. When connected to different access points (while still connected to the same [Tailscale network](https://adityakamath.hashnode.dev/ros-2-and-vpns)), there is some visible latency in the camera stream, even when displaying the compressed image stream. From some research, it seems [Cyclone DDS](https://www.zettascale.tech/product/cyclone/) should improve things significantly (I am currently using [Fast DDS](https://www.eprosima.com/index.php/products-all/eprosima-fast-dds)). Even with Cyclone DDS, some latency can still be expected. Another improvement to this will be using [WebRTC](https://webrtc.org/) instead.

%[https://www.youtube.com/watch?v=osiSNGhKwQo] 

The above video from [Polyhobbyist](https://www.youtube.com/@polyhobbyist) gives a brief introduction to [RobotWebTools](https://robotwebtools.github.io/) and the [webrtc\_ros](https://wiki.ros.org/webrtc_ros) package. I first tried cloning and building the original [webrtc\_ros](https://github.com/RobotWebTools/webrtc_ros) from [RobotWebTools](https://github.com/RobotWebTools), but this did not work - I ended up with some build issues. I did not investigate this - but maybe this was an issue with ROS 2 Humble. Next, I cloned [Polyhobbyist's fork of webrtc\_ros](https://github.com/polyhobbyist/webrtc_ros) and tried building it. It took a while (about 1.5 hours), but this managed to build successfully. I had to leave on holiday and did not get a chance to try it out, but I will be working on it at the beginning of 2024. From what I've read, this alongside Cyclone DDS should solve any latency issues I have with video streaming.

## Remote Teleoperation

Next, I also implemented remote teleoperation over the [Tailscale VPN](https://tailscale.com/). I wanted to publish joystick commands over the internet, rather than having the joystick controller connected to the robot over Bluetooth. The main reason behind this was that the Bluetooth driver stopped working on the AKROS2 mecanum robot's RPi. I still don't know what the reason behind this is, but for now, I dare not try to reinstall the OS. One of my goals for 2024 is to be proficient with [Docker](https://www.docker.com/), so maybe I'll try again once I reach this goal.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1703678572098/6a3e1614-d7a1-4980-ac6d-5f92dca3ddf8.png align="center")

Meanwhile, for remote teleoperation, my system is as follows: I have the joystick connected to a spare RPi which is also on the same Tailscale VPN as the robot. I could have connected it to [WSL2](https://learn.microsoft.com/en-us/windows/wsl/install) on my laptop, but even after [attaching the USB ports to WSL2](https://learn.microsoft.com/en-us/windows/wsl/connect-usb), it does not show up as /dev/tty\* devices, which is needed for the joystick ROS nodes. On the spare RPi, I run the [joystick\_drivers](https://github.com/adityakamath/joystick_drivers) and the [akros2\_teleop](https://github.com/adityakamath/akros2_teleop) packages, which publish the joystick status as [Joy messages](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Joy.html), and use the Joy messages to publish [Twist](https://docs.ros2.org/galactic/api/geometry_msgs/msg/Twist.html) and [Mode](https://github.com/adityakamath/akros2_msgs/blob/master/msg/Mode.msg) messages. Since all devices are connected to the Tailscale VPN as [Discovery Server super-clients](https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html), all these published topics are visible on the robot computer. The latency is minimal, almost negligible.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1703678586096/dc61269e-58b1-4b2e-bc9d-7cc378bdad66.png align="center")

I have two goals for this feature - firstly, I want to upgrade to a Steam Deck device, where I can run ROS 2, the remote teleoperation nodes, and the visualization nodes on the same device. So, the Steam Deck will replace the RPi, the Joystick, and my laptop which I normally use for visualization. Secondly, I want to move to using Cyclone DDS as I mentioned earlier. From what I've read, Cyclone DDS should help in improving latency in the video streams. However, for teleoperation, I doubt there will be a significant improvement as the latency is already minimal.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1703678597914/02730d51-3112-4467-b431-30d74f5dd16d.png align="center")

# Updating AKROS2 Omni

Next, I moved on to the AKROS2 Omni-wheeled robot. In my [last update](https://adityakamath.hashnode.dev/driving-serial-servo-motors), I had finished the base assembly with the 3 [Feetech STS3215](https://www.feetechrc.com/2020-05-13_56655.html) (or the [Waveshare ST3215](https://www.waveshare.com/st3215-servo.htm)) motors, the motor controller, battery, LED strips, and a [Wio Terminal](https://wiki.seeedstudio.com/Wio-Terminal-Getting-Started/). I had also experimented with driving the motors using a Raspberry Pi and tried running Micro-ROS on the Wio Terminal. Now, I decided to make some more updates to the robot.

First, I added a [USB/Ethernet HAT](https://www.waveshare.com/eth-usb-hub-hat.htm) to the RPi0. This lets me connect an [LD06 LIDAR](https://www.inno-maker.com/product/lidar-ld06/), the [FE-URT-1 motor controller](https://www.feetechrc.com/FE-URT1-C001.html) as well as the Wio Terminal using USB (I couldn't get micro-ROS working with UDP over WiFi, but Serial works perfectly). It also leaves the Ethernet port free for any future use.

Next, I added the LD06 LIDAR and a [BNO085 IMU from Adafruit](https://www.adafruit.com/product/4754) and designed a plate to hold them. I left some holes so that I can add spacers and another layer on top, for any additional payload and an [I2C display](https://www.amazon.nl/-/en/dp/B08HDGNXWB?psc=1&ref=ppx_yo2ov_dt_b_product_details) to show the RPi0's IP address and other diagnostic information. But this is for the next update, for now, here's the 3D model of the robot, with the RPi0 (+ the USB/Ethernet HAT), the LD06, the IMU (which is between the RPi0 and the LD06) and the base plate.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1703399298416/9c7383d3-51b6-459e-9e6d-3932081b0468.png align="center")

I am currently waiting for the [laser-cut base plate](https://snijlab.nl/) and the IMU to arrive in the first week of January, so the final assembly will be slightly delayed.

# Everything else

Since this is a recap of the year, I also worked on these things as well (with linked articles):

* I started the year by upgrading to [ROS 2 Humble and micro-ROS Humble](https://kamathsblog.com/upgrading-ros-2-micro-ros-versions) and experimenting with [teleop using different game controllers](https://kamathsblog.com/teleop-with-game-controllers)
    
* I then [migrated the mecanum robot's description file to ROS 2 Noetic to ROS 2 Humble](https://kamathsblog.com/updating-the-robot-description)
    
* Next, I fixed some issues I was having with [micro-ROS Galactic, especially the Parameter Server](https://kamathsblog.com/micro-ros-parameter-server)
    
* In April, I decided to build a [digital twin of the mecanum robot in Unity](https://kamathsblog.com/visualizing-robots-in-unity).
    
* I got bored with the mecanum robot, so I decided to play around with some sensors ([optical flow](https://kamathsblog.com/odometry-using-optical-flow), ToF array) and motors ([serial bus servo motors](https://kamathsblog.com/driving-serial-servo-motors))
    
* Finally, I dug into [VPNs for use with Fast DDS and ROS 2](https://kamathsblog.com/ros-2-and-vpns)
    

# Plans for 2024

As the year comes to an end, there are a few things that I want to work on in 2024. I already wanted to spend some time with [ros2\_control](https://control.ros.org/master/index.html) and [Docker](https://www.docker.com/) this year but did not get much time due to other commitments, so they are at the top of my list. For practicing ros2\_control, I also plan on building a small differential drive robot, just to test out the existing [differential drive controller](https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html). Once this is done, I will then work on my own three omni-wheeled controller for the AKROS2 Omni robot.

Another thing that will carry forward to 2024 is the [webrtc\_ros](https://github.com/polyhobbyist/webrtc_ros) implementation. As I explained earlier, I now have a new camera on board the AKROS2 Mecanum robot, and I need webrtc\_ros for streaming the images from it. Additionally, I also want to move from [Fast DDS](https://www.eprosima.com/index.php/products-all/eprosima-fast-dds) to [Cyclone DDS](https://www.zettascale.tech/product/cyclone/) as it seems to reduce latency over distributed systems, especially for camera images.

Besides ros2\_control, Docker, webrtc\_ros and Cyclone DDS, I also plan to work with (or at least explore) the following things:

* [Micro-ROS](https://micro.ros.org/): I've mostly worked with [micro\_ros\_arduino](https://github.com/micro-ROS/micro_ros_arduino) and the [Arduino IDE 2](https://docs.arduino.cc/software/ide-v2), now I want to [develop using the ESP32 family of microcontrollers](https://github.com/micro-ROS/micro_ros_espidf_component), especially the new [ESP32-S3](https://www.espressif.com/en/products/socs/esp32-s3). I've also purchased a couple of [XIAO ESP32-S3 boards](https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html), which I want to try. I also want to play around with the Raspberry Pi Pico and the RP2040 microcontroller.
    
    %[https://twitter.com/kamathsblog/status/1728843894811775407] 
    
* Teleop with [Steam Deck](https://store.steampowered.com/steamdeck): As a holiday present for myself, I ordered a Steam Deck 64GB a few days ago. I've seen it running ROS and controlling robots on multiple occasions and want to give it a try. As a bonus, I can also play games with it. PS: Since the launch of the new [Steam Deck OLED](https://www.steamdeck.com/en/oled), the price for the non-OLED variants have dropped significantly, especially the 64GB version that does not have an NVMe SSD. So, it is a perfect time to get one!
    
    ![](https://cdn.hashnode.com/res/hashnode/image/upload/v1703315404600/c13edfc9-c003-4872-96eb-cc7665f19c69.png align="center")
    
* [Gazebo](https://gazebosim.org/home): Time to finally get started with Gazebo. The last time I used it was for my master thesis between 2016 and 2017 and I haven't touched it since. I want to implement Gazebo simulations for at least one of my robots.
    
* [Nav2](https://navigation.ros.org/): Same thing goes for Nav2. Once the control and simulation packages are done, time to implement the navigation stack for both Mecanum and Omni robots.
    
* Fleet Management: For fleet management, I want to explore existing solutions like [Open-RMF](https://openrmf.readthedocs.io/en/latest/) but I would also like to take a shot at designing a solution myself. I think I'll eventually end up building on top of an existing open-source solution. I've already started reading [this multi-robot notebook by OSRF](https://osrf.github.io/ros2multirobotbook/) for this. I'm putting this on the 'nice to have' list as I don't think I'll end up with time to work on a robot fleet.
    
    ![](https://osrf.github.io/ros2multirobotbook/images/free_fleet_block_diagram.png align="center")
    
* [Zenoh](https://zenoh.io/): [Zenoh](https://github.com/ros2/rmw_zenoh) was chosen earlier this year as an [alternate RMW alongside other DDS RMWs for future releases of ROS 2](https://discourse.ros.org/t/ros-2-alternative-middleware-report/33771/1). This should solve quite a lot of the issues users face while working with DDS. For now, Zenoh can be tried out with [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) by using the [Zenoh-DDS bridge](https://zenoh.io/blog/2021-04-28-ros2-integration/), which relies on Cyclone DDS. Since I'm already moving to Cyclone DDS, seems like a perfect opportunity to try Zenoh out as well.
    
* [ROSCon](https://roscon.ros.org/2023/): Finally, [ROSCon 2024](https://www.odenserobotics.dk/roscon-comes-to-odense-next-year/) which will be held in [Odense, Denmark](https://www.visitdenmark.com/denmark/destinations/fyn/odense). Perfect location for me, since it is about 3 hours away from Eindhoven (flight + train), and no need for any visa as well! I will definitely be attending. I also want to try to attend other local ROSCons in Europe, as long as they are in English. I hope they have a [ROSCon India](https://rosconindia.in/) next year as well, which I also want to attend (I had to cancel my plans last minute this year).
    

![](https://www.visitodense.com/sites/visitodense.com/files/styles/article_teaser/public/2023-09/hca-julemarked2022%40daniel-jensen.jpg?h=d375c041&itok=q-BusMQC align="center")

That said, thanks for reading so far. I hope you have a fun time during the holidays, and wishing you all an excellent 2024!