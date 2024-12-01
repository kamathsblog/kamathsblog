---
title: "Upgrading ROS 2, micro-ROS versions"
datePublished: Sun Feb 12 2023 16:23:37 GMT+0000 (Coordinated Universal Time)
cuid: cleblm9qf00000al2a7xucf4u
slug: upgrading-ros-2-micro-ros-versions
cover: https://cdn.hashnode.com/res/hashnode/image/upload/v1676823136093/bc242cd8-b415-40eb-bddc-ab2fd7ceff29.jpeg
tags: migration, robotics, controllers, ros

---

Recently, I migrated the software stack of my robot from [ROS 2 Galactic](https://docs.ros.org/en/galactic/index.html) to [ROS 2 Humble](https://docs.ros.org/en/humble/index.html). I thought it was going to be straightforward - add my Galactic nodes to a new Humble workspace, compile and it will work right away. But I faced a few issues getting my nodes to compile, and in this article, I will detail these issues and how I fixed them.

## Context

I did not opt for the standard ROS 2 Humble installation on Ubuntu 22.04. Instead (after being inspired by [this ROSCon talk)](https://vimeo.com/showcase/9954564/video/767139709), I decided to use [this Raspberry Pi image](https://github.com/ros-realtime/ros-realtime-rpi4-image/releases/tag/22.04.1_v5.15.39-rt42-raspi_ros2_humble) with Humble installed on a real-time Ubuntu kernel. I followed the steps in the GitHub repository (the image does not include the desktop environment, which I also had to install separately), and used [this amazing blog post](https://robofoundry.medium.com/notes-on-upgrading-to-ubuntu-22-04-and-ros2-humble-8149804abc91) by [RoboFoundry](https://twitter.com/robofoundry), which helped with initial issues like compile time on the Raspberry Pi.

First I used the following command (from RoboFoundry's blog) to minimize the CPU processing by building each package sequentially. I promptly created an alias for this for future use.

```bash
MAKEFLAGS="-j1 -l1" colcon build --symlink-install --executor sequential
```

## Fixing warnings and errors

Next, I used the command to build my workspace which I hadn't changed since Galactic. Most were built successfully, but with loads of warnings. Some failed completely. I started with the warnings and went through them one by one. The first warning can be seen below:

```bash
SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
```

This turned out to be a known issue, especially with the latest version of [setuptools](https://pypi.org/project/setuptools/), and the fix involved downgrading setuptools to version 58.2.0, the last version to work with ROS 2 without any warnings. The solution is detailed in [this ROS Answers post](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/).

**Update:** There's an alternative solution to this, where downgrading setuptools is not needed. [This warning can instead be suppressed](https://robotics.stackexchange.com/a/24349) by updating the `PYTHONWARNINGS`environment variable, which can be done by adding the following line to `~/.bashrc`:

```bash
PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install"; export PYTHONWARNINGS
```

Moving on to the next warning:

```plaintext
WARNING:colcon.colcon_ros.task.ament_python.build:Package 'foobar' doesn't explicitly install a marker in the package index (colcon-ros currently does it implicitly but that fallback will be removed in the future)
```

Once again, thanks to [ROS Answers](https://answers.ros.org/questions/), I manually created a marker file for the packages where it was missing, and the warning was resolved. The detailed answer can be found [here](https://answers.ros.org/question/367328/ament_python-package-doesnt-explicitly-install-a-marker-in-the-package-index/).

Next, having resolved the warnings, I moved on to the build errors. It took me a while, but I eventually found [this GitHub issue](https://github.com/ros2/rosidl_python/issues/141), which led me to the [documentation of ament\_cmake\_python](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Python-Documentation.html). One of my Galactic packages ([ds4\_driver](https://github.com/naoki-mizuno/ds4_driver), for use with the PS4 controller) was failing to compile because it was calling [rosidl\_generate\_interfaces](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html) and [ament\_python\_install\_package](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Python-Documentation.html) in the same CMake project, which apparently does not work with ROS 2 Humble.

The only solution here was to separate the message generation from the node functionality into a separate package. I fixed the issue over the weekend and created a [pull request](https://github.com/naoki-mizuno/ds4_driver/pull/33) on the ds4\_driver project, which has since been reviewed and merged.

With this, everything was building and I was able to run the launch files. So, it was time to test the functionality.

## Testing

I haven't set up any unit testing for my nodes yet, but ever since I realized that ChatGPT can write unit tests for any given ROS code, I've been meaning to set up testing in my workspace. But for now, it was time to manually test all the functionality. This includes the following components, which can be turned on or off from launch file arguments.

* [akros2\_bringup](https://github.com/adityakamath/akros2_bringup):
    
    * Launches the [LD06 Lidar](https://www.inno-maker.com/product/lidar-ld06/) using the [linorobot/ldlidar](https://github.com/linorobot/ldlidar) package
        
    * Launches the [T265 Tracking Camera](https://www.intelrealsense.com/tracking-camera-t265/) with the latest release of [realsense-ros](https://github.com/IntelRealSense/realsense-ros/releases/tag/4.51.1) (currently did not work on Galactic)
        
    * Launches [akros2\_teleop](https://github.com/adityakamath/akros2_teleop) launch file
        
        * Launches the ds4\_driver node and reads inputs from the PS4 controller
            
        * Launches nodes to multiplex between autonomous and teleoperation nodes and for emergency stop signals
            
        * Launches the [micro-ROS agent](https://github.com/micro-ROS/micro_ros_setup) which connects to the [Teensy 4.1 platform](https://www.tindie.com/products/cburgess129/arduino-teensy41-teensy-41-expansion-board/) for the low-level motion control
            
    * Launches visualization tools:
        
        * [Foxglove Studio](https://foxglove.dev/studio) connection using [rosbridge\_server](https://foxglove.dev/docs/studio/connection/ros2#rosbridge)
            
        * [ROSBoard](https://github.com/dheera/rosboard)
            

### LD06 and Visualization

First, I had just learned about [Foxglove's own WebSocket](https://foxglove.dev/docs/studio/connection/ros2#foxglove-websocket) protocol as a more robust alternative to rosbridge\_server, so I decided to make the necessary changes to use it. Then, for my first test, I launched the LD06 Lidar, along with ROSBoard and Foxglove visualization using Foxglove's WebSocket. Everything worked smoothly, and I was able to visualize results on Foxglove Studio on Windows and on the ROSBoard portal on my browser. Small success!

### Micro-ROS

Like ROS 2, [micro-ROS](https://micro.ros.org/) also has had a lot of changes between Galactic and Humble. The micro-ROS code which was already on the Teensy platform used Galactic libraries, and when I tried it with the micro-ROS agent in Humble, it did not work at all. So, I had to install the Humble libraries on my laptop and compile it, but then I was unable to compile the Arduino code.

[I found out](https://github.com/micro-ROS/micro_ros_arduino/issues/1285) that now, the supported method for running micro-ROS on Teensy with Arduino used the [Arduino 2.0 IDE](https://docs.arduino.cc/software/ide-v2/tutorials/getting-started/ide-v2-downloading-and-installing) and [a different process to set up Teensyduino](https://www.pjrc.com/arduino-ide-2-0-0-teensy-support/). Once I set everything up, I was able to use the new Humble libraries. However, I had to comment out the [Parameter Server](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/parameters/). The parameter server is initialized differently in micro-ROS Humble as compared to previous ROS 2 versions.

I made the necessary changes according to the documentation, but I wasn't able to get it working. I was able to upload the code, but the microcontroller always went into the error loop, triggering the onboard LED to flash. After temporarily commenting out the parameter server, the remaining parts worked perfectly. So, when the akros2\_teleop launch file is executed, the ds4\_driver reads joystick inputs and converts joystick inputs to twist messages, which the Teensy subscribes to in order to drive the motors.

### DS4 Driver troubles

When I first launched the updated ds4\_driver, it did not work entirely as expected. I was able to read the joystick and button inputs and convert them to [twist](https://docs.ros2.org/galactic/api/geometry_msgs/msg/Twist.html) and [mode](https://github.com/adityakamath/akros2_msgs) messages. On the other hand, I was unable to send feedback to the controller to set LED colors or rumble. This was a bit of a bummer, but at that time, I moved on with trying out the micro-ROS node.

Once the micro-ROS node and the rest of the drive functionality were working, some issues started cropping up with the PS4 controller. First, the ds4\_twist node stopped publishing twist messages a few minutes after launching. The main ds4\_driver node was operational as it was still publishing status messages, but the conversion to twist messages did not work. This, along with the inability to send feedback messages to the PS4 controller was a dealbreaker. For the time being, I decided to use the [teleop\_twist\_keyboard](https://index.ros.org/r/teleop_twist_keyboard/) to send twist messages and [raised an issue](https://github.com/naoki-mizuno/ds4_driver/issues/34) on the ds4\_driver repository. I have a sneaky suspicion that these issues are because of the RT kernel, but I am waiting for someone else to try it and verify this assumption.

Meanwhile, here's the robot driving around during one of the few tries where the PS4 controller driver actually worked:

%[https://www.youtube.com/watch?v=B-Hqz9Os7Kw] 

### Success with the T265

Finally, I tried the T265 node. I made a fresh install of the [realsense\_ros drivers](https://github.com/IntelRealSense/realsense-ros/releases/tag/4.51.1) to run the device, and to my surprise, everything worked on the first attempt! This never happened in Galactic, where my RPi ended up crashing every time I launched the T265 node. On ROS 1, I also had weird issues with the T265 node not publishing any data after some time of operation. I had only one issue on Humble (just like with previous versions) - the T265 is not recognized when the robot/RPi is turned on for the first time. The device needs to be physically unplugged and re-connected for it to be detected. But this is a known issue that Intel has stated it won't be spending time fixing, especially since the T265 is now discontinued. There was nothing I could do here, so I moved on.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1676823180639/1a43ce5a-34d8-4c9b-b817-4a66ca188283.jpeg align="center")

## Next up

There are two things I want to accomplish next:

1. Fixing the ds4\_driver issue: The first step would be to figure out why it is failing and see if my suspicion about the RT kernel is true. I'll do that by setting up Humble on a new RPi without the RT kernel and trying the driver there. If nothing works, I have some fallback options - the [joy](https://github.com/adityakamath/joystick_drivers/tree/ros2/joy) node and [teleop\_twist\_joy](https://index.ros.org/p/teleop_twist_joy/github-ros2-teleop_twist_joy/) for instance. Using these packages, I would also be able to use other controllers like the [Stadia controller](https://stadia.google.com/controller/) which I recently updated to Bluetooth mode and had already tested with the RPi on the robot. The Stadia controller is also a fallback option if I find out that something is wrong with the PS4 controller hardware itself.
    
    ![](https://cdn.hashnode.com/res/hashnode/image/upload/v1676823340988/aaf080e9-da92-49af-9b7c-d4fd7ea00ca9.jpeg align="center")
    
2. Fixing the parameter server issue: I only glanced through the documentation and tried to fix it as fast as I could and I clearly missed something. I don't always use the parameter server, I use it only during tuning and calibration to set the PID constants at run-time. But having realized that I hadn't committed the latest PID values to [my GitHub repo](https://github.com/adityakamath/akros2_firmware), I need to tune the motor controllers again, so the parameter server is crucial.
    

Meanwhile, there's a second robot I'm working on. So far I've only managed to run some LED sequences and an example of [Uncanny Eyes](https://github.com/adafruit/Uncanny_Eyes) on the [Wio Terminal](https://wiki.seeedstudio.com/Wio-Terminal-Getting-Started/).

%[https://www.youtube.com/watch?v=tuPxhawaxes] 

%[https://www.youtube.com/watch?v=kOFKQO0oR54] 

%[https://www.youtube.com/watch?v=c1lq1oFbpxc] 

If you've been following my progress on [Twitter](https://twitter.com/kamathsblog), I first planned on using the [Arduino Nano RP2040 Connect](https://docs.arduino.cc/hardware/nano-rp2040-connect), but I have now switched to the Wio Terminal. It has got an IMU and a microphone just like the RP2040 Connect, but also a screen, buttons, and a speaker. I'm also using the [Wio Terminal Battery Chassis (650 mAh)](https://wiki.seeedstudio.com/Wio-Terminal-Chassis-Battery\(650mAh\)/) which also extends the number of Grove connectors and works perfectly with the Grove-based LED strips. Bonus: the Wio Terminal can also run micro-ROS!

%[https://twitter.com/K66941741/status/1604849092483776512] 

That's it for this weekend. Small and steady steps.