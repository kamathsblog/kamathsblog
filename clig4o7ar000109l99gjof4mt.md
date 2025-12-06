---
title: "Odometry using Optical Flow"
datePublished: Sat Jun 03 2023 15:07:14 GMT+0000 (Coordinated Universal Time)
cuid: clig4o7ar000109l99gjof4mt
slug: odometry-using-optical-flow
cover: https://cdn.hashnode.com/res/hashnode/image/upload/v1685800724354/89aebdd0-006f-4a47-bd14-43d77658e122.jpeg

---

A few weeks ago, I got my hands on the [PAA5100 Near (15-35mm) Optical Flow sensor from Pimoroni](https://shop.pimoroni.com/products/paa5100je-optical-tracking-spi-breakout?variant=39315330170963). With such a short range, it seemed perfect for one of my mobile robots, so I decided to write a [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) node for it. I decided to start off using the [pmw3901-python library](https://github.com/pimoroni/pmw3901-python/tree/master) provided by Pimoroni. This library provides classes for both the [PMW3901](https://shop.pimoroni.com/products/pmw3901-optical-flow-sensor-breakout?variant=27869870358611) (long-range) and PAA5100 (short-range) sensors. To things simple, I decided to write the node in Python. I also used this as an opportunity to play around with some ROS 2 concepts in Python - like [Lifecycle nodes](https://design.ros2.org/articles/node_lifecycle.html) and [Executors](https://docs.ros.org/en/humble/Concepts/About-Executors.html), so this implementation is definitely over-engineered.

%[https://twitter.com/kamathsblog/status/1655490934186954752] 

An optical flow sensor acts like the sensor in a digital mouse, by measuring the movement of objects using the motion of pixels in a sequence of images. By tracking these changes, the sensor can calculate the relative motion of the surface in two dimensions. These relative displacement values are represented in pixels, which need to be converted into meters as will be explained later. The PMW3901 sensor is a long-range sensor (80mm-infinity), which makes it suitable for quadrotors and other flying robots. The PAA5100 has a short range (15-35mm), making it perfect for ground vehicles as the sensor can be placed closer to the ground.

### Lifecycle Node

The first step was to write a [minimal publisher](https://github.com/ros2/examples/tree/rolling/rclpy/topics/minimal_publisher) - I started off by creating a simple optical flow class, and within it creating a node, a publisher, and a timer. I then added a timer callback from where I read the sensor data and publish an empty odometry message. I then created a main function that creates an instance of this class and spins the node.

Next, I updated the node to be a [lifecycle (managed) node](https://design.ros2.org/articles/node_lifecycle.html), using a minimal example [here](https://github.com/ros2/demos/tree/rolling/lifecycle_py). A lifecycle node uses a state machine to provide control of the state to the user. More information about the lifecycle node can be found in the design documents [here](https://design.ros2.org/articles/node_lifecycle.html). The state machine can be seen below:

![](https://design.ros2.org/img/node_lifecycle/life_cycle_sm.png align="center")

To implement the lifecycle node, I first updated the class so that it inherits from (lifecycle) Node class and then added the following five functions:

* **on\_configure**: this function runs when `onConfigure` is called. Here, I configure the sensor and create my odometry publisher, transform broadcaster, and timer. If successful, the state of the node transitions from `unconfigured` to `inactive`.
    
* **on\_activate**: this function runs when `onActivate` is called. In my implementation, I only use this to log a message stating the sensor is activated. It then transitions the state of the node from `inactive` to `active`.
    
* **on\_deactivate**: this function runs when `onDeactivate` is called. Like on\_activate, this function logs a message stating the sensor is deactivated and transitions the state from `active` to `inactive`
    
* **on\_cleanup**: this function runs when `onCleanup` is called. This function cancels the timer, and destroys the publisher and the transform broadcaster. Then, it logs a message and transitions the state from `inactive` to `unconfigured`.
    
* **on\_shutdown**: this function runs when `onShutdown` is called. This function, does the same thing as on\_cleanup, except the state transitions from `unconfigured`, `active` or `inactive` to `finalized`.
    

I then updated the timer callback, which first checks if the state is `active`, and if yes, it reads sensor data as delta x and delta y values represented in pixels. These pixel values are first converted to relative distances per time step, using which the absolute distance and the current velocity are calculated. These values are populated in an [Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html) message, which is published, and also used to compute the transform between the odometry frame and the base frame. This transform is then broadcasted using the [transform broadcaster](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html).

### Pixel values -&gt; Distances

It was quite challenging to find the equation to convert from pixel values to distances in meters, especially because of all the guarded information by the manufacturer - more about this later. Eventually, I found two resources - [this](https://ardupilot.org/copter/docs/common-mouse-based-optical-flow-sensor-adns3080.html) archived document about the [ADNS3080](https://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/s2009/ncr6_wjw27/ncr6_wjw27/docs/adns_3080.pdf) sensor, and the measurement equations provided in [this](https://lup.lub.lu.se/luur/download?func=downloadFile&recordOId=8905295&fileOId=8905299) master thesis. I also realized that the frame of the PAA5100 sensor seems to be rotated 90 degrees clockwise as compared to the default orientation of PMW3901 (according to [documentation,](https://docs.px4.io/main/en/sensor/pmw3901.html#mounting-orientation) not verified), so I had to make the necessary changes. The final equations I ended up using are as follows:

For PAA5100:

$$dist_x = -1 * (\frac{delta_y * h}{res * scaler}) * 2 * tan(\frac{fov}{2})$$

$$dist_y = (\frac{delta_x * h}{res * scaler}) * 2 * tan(\frac{fov}{2})$$

For PMW3901 (untested):

$$dist_x = (\frac{delta_x * h}{res * scaler}) * 2 * tan(\frac{fov}{2})$$

$$dist_y = (\frac{delta_y * h}{res * scaler}) * 2 * tan(\frac{fov}{2})$$

Where `h` is the height from the ground at which the sensor is placed, `res` is the resolution of the sensor in pixels, `fov` is the field of view of the sensor (defined as 42 degrees for both PMW3901 and PAA5100 sensors) and `scaler` is the [scaling factor](https://github.com/adityakamath/pmw3901_ros#improvements-to-do). `delta_x` and `delta_y` are the raw sensor measurements and `dist_x` and `dist_y` are the resulting delta displacements in the x and y axes.

Another issue that I observed was the resolution value. The library uses a [raw data length of 1225](https://github.com/pimoroni/pmw3901-python/blob/master/library/pmw3901/init.py#L344) for both PMW3901 and PAA5100, which is 35x35, i.e `res` is 35. However, most [online resources](https://github.com/bitcraze/crazyflie-firmware/blob/6308ff47ff4d4691f9b7f6f991564244c76d7910/src/modules/src/estimator_kalman.c#LL1040C15-L1040C15) about the PMW3901 define the resolution as 30x30. This needs to be verified practically with a PMW3901 sensor, which I do not have.

The measured displacements in meters are accumulated every time period to get the absolute distance traveled in the x and y directions, and the displacements are divided by the timer period to calculate the velocity at that particular instance. These values are populated in the [Pose](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Pose.html) ([position](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Point.html)) and [Twist](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html) ([linear velocity](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Vector3.html)) fields in the Odometry message.

%[https://gist.github.com/adityakamath/b88d36209b5ebf08d484bc9898362812] 

Finally, I updated the lifecycle node to accept constant values like height and resolution using the [parameter server](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html). The publisher node can be run using the following command. This command runs the **optical\_flow\_publisher** executable that executes the `main()` function in the publisher node.

```bash
$ ros2 run pmw3901_ros optical_flow_publisher
```

The publisher node implementation can be found [here](https://github.com/adityakamath/pmw3901_ros/blob/humble/pmw3901_ros/optical_flow_publisher.py).

### Improvements / To Do

There are still some improvements that can be done. Firstly, the publisher assumes that the sensor is placed at a fixed height parallel to the ground (within the range of the chosen sensor variant), and currently does not account for any rotations about any axis. For now, any rotation about the x and y axes would confuse the sensor resulting in invalid measurements. However, these rotations (roll and pitch) can be accounted for using the explanation in the [ADNS3080 documentation](https://ardupilot.org/copter/docs/common-mouse-based-optical-flow-sensor-adns3080.html) and the [master thesis](https://lup.lub.lu.se/luur/download?func=downloadFile&recordOId=8905295&fileOId=8905299) referenced in the [Bitcraze PMW3901 driver](https://github.com/bitcraze/Bitcraze_PMW3901/tree/master). This still needs to be implemented.

Next, the scaling factor - for a single pixel move, the sensor returns a value of more than 1. This is the scaling factor or simply *scaler*. This scaler seems to be a proprietary value and cannot be found anywhere online, and most references mention an NDA with the sensor manufacturer. This can be calculated empirically by moving the sensor to a known distance and comparing it with the distance measured by the sensor (at a fixed height). It is also unknown if the scaler will change for different sensor heights, but this can only be tested empirically once a sensor mount is designed.

%[https://twitter.com/kamathsblog/status/1659259171106177024] 

Both PMW3901 and PAA5100 breakout boards have identical footprints and identical positions of the sensor module on the PCB. However, on both these boards, the sensor package is not centered on the PCB. The sensor aperture itself is not centered on the sensor package. This is quite annoying since these offsets must be accounted for when designing the mount for the sensor. Fortunately, there is an accurate [mechanical drawing provided by Pimoroni](https://cdn.shopify.com/s/files/1/0174/1800/files/39b9173de8970896f2eaa114ef5738eb993a06cc.png?v=1621246728).

Finally, the entire implementation has been tested with the PAA5100 sensor, and not the PMW3901. There are slight differences in the documentation available online, resulting in different resolution values, and different default orientation for PMW3901 as compared to the PAA5100. So, I'm not entirely sure if the implementation will work correctly with the PMW3901 sensor. This needs to be tested practically with a physical sensor.

### Executor

With the lifecycle publisher up and running, I decided to implement an executor node. An [Executor](https://docs.ros.org/en/humble/Concepts/About-Executors.html) implements one or more threads of a system to invoke callbacks, and provides control over the execution management, especially with multiple nodes. Some nice examples (for microcontrollers) using C++ can be seen in the [execution management documentation for micro-ROS](https://micro.ros.org/docs/concepts/client_library/execution_management/). In my case, since there's only one node, this does not add much value. I only implemented it to see how it is done using the [ROS2 Client Library for Python (rclpy)](https://github.com/ros2/rclpy).

I found [this simple example](https://github.com/ros2/examples/blob/rolling/rclpy/executors/examples_rclpy_executors/composed.py) and used it as a starting point to write my own executor node. This node implements its own `main()` function, which overrides the one in the optical flow publisher. I've kept the `main()` in the publisher node just so that I can launch it on its own without using an executor, if it is not required, it can be removed.

The executor node can be run using the following command. This command runs the **optical\_flow\_node** executable that executes the `main()` function in the executor node, overriding the `main()` in the publisher node.

```bash
$ ros2 run pmw3901_ros optical_flow_node
```

So, now there are two ways to run the node. I am not sure if there is an advantage to using one over the other, especially in this use case. I also haven't run any tests to check. However, it has been a nice learning experience.

The executor node implementation can be found [here](https://github.com/adityakamath/pmw3901_ros/blob/humble/pmw3901_ros/optical_flow_node.py).

### Launch file

So far, I have been running the executor node, which creates an instance of the optical flow publisher in the `unconfigured` state. I was then configuring and activating the node using the following commands on the terminal:

```bash
$ ros2 lifecycle set /optical_flow configure
$ ros2 lifecycle set /optical_flow activate
```

I wanted to automate the process so that I run the executor node first, then configure the lifecycle node, and finally activate the lifecycle node when the `inactive` state is reached. This was implemented using a [ROS 2 Python launch file](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html) which can be found [here](https://github.com/adityakamath/pmw3901_ros/blob/humble/launch/optical_flow_launch.py).

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1685799617528/d845c74b-861b-4c3a-86eb-d0df1facd7f8.jpeg align="center")

With the launch file working, the package is almost fully implemented. I still haven't implemented any unit tests as I have no experience with [testing in Python](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html), but I'll get to it when I get the time. For now, I've sufficiently tested the implementation in various conditions, and it works quite well in a controlled scenario (sensor at a fixed height, parallel to the ground, no rotations).

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1685799514823/6507680b-abfa-423d-8103-72218c9a2c55.gif align="center")

The **optical\_flow\_ros** package can be found [here](https://github.com/adityakamath/optical_flow_ros).

### ToF Imager

I'm working on a project with the [VL53L5CX ToF Imager](https://www.sparkfun.com/products/18642) sensor directly connected to a Raspberry Pi, so I decided to implement a ROS 2 node for it. This is a Time-of-Flight sensor that produces 8x8 or 4x4 range measurements, which can be converted into a pointcloud. So far, I've been using these sensors with a microcontroller in the middle - I connect the sensor to the microcontroller using I2C, populate the pointcloud on the microcontroller and then stream that to the Raspberry Pi using [micro-ROS](https://micro.ros.org/). Two implementations can be seen in the **tof\_imager\_micro\_ros** package [here](https://github.com/adityakamath/tof_imager_micro_ros).

%[https://twitter.com/kamathsblog/status/1646854548709859329] 

However, in this case, I want it directly connected to the Raspberry Pi's I2C port, so I decided to replicate the optical flow lifecycle node implementation for the VL53L5CX. Similar to the optical flow publisher, the ToF imager publisher first accepts parameters from the parameter server, and uses it configure the VL53L5CX sensor, converts raw sensor measurements into a pointcloud, which is published as [Pointcloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) message. The executor then initializes the lifecycle node and runs it. Finally, the launch file launches the executor node, configures it, and automatically sets the state to `active` once configured.

The **tof\_imager\_ros** package can be found [here](https://github.com/adityakamath/tof_imager_ros).

### Next up

Next up, I want to set up a small differential drive robot. I've ordered two [100:1 micro metal geared motors](https://shop.pimoroni.com/products/micro-metal-gearmotor-extended-back-shaft?variant=39421592043603) from Pimoroni, as well as two of their [encoder shields](https://shop.pimoroni.com/products/micro-metal-motor-encoder?variant=39888423321683). I will use them with the [Inventor HAT](https://shop.pimoroni.com/products/inventor-hat-mini?variant=40588023464019) that sits on top of a [Raspberry Pi Zero 2 W](https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/). This robot will use [ros2\_control](https://control.ros.org/humble/index.html) to control the differential drive and calculate odometry using the wheel encoders. This odometry will be fused with measurements from the PAA5100 optical flow sensor to get better accuracy. Finally, a camera will be added to the Raspberry Pi.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1685804761200/3e744106-d3fc-454d-8337-0acba6fb28fa.jpeg align="center")

With the PAA5100 node done, I will next focus my efforts on getting a minimal ros2\_control implementation with the [differential drive controller](https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html). Next, I will come back to the PAA5100 to implement unit tests in Python. I'm still waiting for these motors and encoders to be delivered, I cannot wait to get my hands on them to start implementing the control system. More on this in the next update.

As for my other goal of reading a book every month, I took a break from normal books and picked up a few graphic novels. In April, I read [Batman: The Killing Joke](https://www.goodreads.com/book/show/96358.Batman) and in May, I finished the [graphic novel version of 1984](https://www.goodreads.com/book/show/48930315-1984?ref=nav_sb_ss_1_12), both really good books with some amazing art work. Probably for June, I will pick up another graphic novel but I haven't made up my mind just yet.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1685798991426/668bfbf0-f961-48a6-82b7-48f7a017aec1.jpeg align="center")