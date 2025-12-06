---
title: "Micro-ROS Parameter Server"
datePublished: Thu Apr 06 2023 22:47:11 GMT+0000 (Coordinated Universal Time)
cuid: clg9zxf7o000009ky66xzejxp
slug: micro-ros-parameter-server
cover: https://cdn.hashnode.com/res/hashnode/image/upload/v1681080345841/cb9eb2ab-2268-4293-940d-75e2e2f6fed0.jpeg

---

Fixing the [micro-ROS parameter server](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/parameters/) on my robot has been on my mind for a long time, but I got quite busy working on other things like setting up a [visualization in Unity](https://github.com/Unity-Technologies/Unity-Robotics-Hub) (I still have some issues to fix here, and I will hopefully write a post about this next week). I now find myself with some extra time, so let's fix the parameter server once and for all.

For context, when I first tried compiling the micro-ROS firmware written for Galactic, with micro-ROS Humble, I got a lot of errors, especially with the parameter server. Then I realized that the parameter server is defined and initialized differently in micro-ROS Humble as compared to Galactic, and I ended up commenting out all the parameter server bits just to make the rest of the code work.

### Setting up the Parameter Server

First I started by including the necessary library, and [defining the parameter server](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/parameters/#initialization):

```c
#include <rclc_parameter/rclc_parameter.h>

rclc_parameter_server_t parameter_server;
```

Next, I updated the [`createEntities()`](https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino) function, where I created and [initialized the parameter server](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/parameters/#initialization-options-1) with options:

```c
// create parameter server
rclc_parameter_options_t param_options = {
      .notify_changed_over_dds = true,
      .max_params = 4,
      .allow_undeclared_parameters = false,
      .low_mem_mode = true};
  
RCCHECK(rclc_parameter_server_init_with_option(
        &param_server,
        &node,
        &param_options));
```

So far, the implementation has been identical to the micro-ROS Galactic firmware, except for the param\_options configuration. I decided to set the `allow_undeclared_parameters` option to false, as I had no intention for any more parameters to be declared externally. I also set the `low_mem_mode` option to true. The low memory mode significantly reduces the memory used by the parameter server, and constrains the functionality such that only one parameter request (get, set, etc) can be handled at one time.

In my case, I plan on having 4 parameters - `kp`, `ki`, `kd` and `scale`, the first three representing the PID gains, and the last parameter representing the max RPM scale of the motors. I only plan on using these parameters for testing and tuning, and not during normal operation. So, I was quite fine with the reduced functionality because of the low memory mode.

Next, I [updated the number of handles in the executor initialization function](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/parameters/#memory-requirements) and added the parameter server to the executor. For Galactic, the number of handles required by the parameter server is defined by `RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER` which needs to be added to the number of handles required by the rest of the code. For Humble, however, the name of the variable has been changed to `RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES`:

```c
RCCHECK(rclc_executor_init(&executor, &support.context, RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES+3, &allocator));

RCCHECK(rclc_executor_add_parameter_server(
          &executor,
          &param_server,
          paramCallback));
```

Next, I [added the parameters to the server](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/parameters/#add-a-parameter-1), and set them to their default values:

```c
// Add parameters to the server
RCCHECK(rclc_add_parameter(&param_server, kp_name, RCLC_PARAMETER_DOUBLE));
RCCHECK(rclc_add_parameter(&param_server, ki_name, RCLC_PARAMETER_DOUBLE));
RCCHECK(rclc_add_parameter(&param_server, kd_name, RCLC_PARAMETER_DOUBLE));
RCCHECK(rclc_add_parameter(&param_server, rpm_ratio_name, RCLC_PARAMETER_DOUBLE));

// Set parameter default values
RCCHECK(rclc_parameter_set_double(&param_server, kp_name, K_P));
RCCHECK(rclc_parameter_set_double(&param_server, ki_name, K_I));
RCCHECK(rclc_parameter_set_double(&param_server, kd_name, K_D));
RCCHECK(rclc_parameter_set_double(&param_server, rpm_ratio_name, MAX_RPM_RATIO));
```

Next, in the [`destroyEntities()`](https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino) function, I added the function call to [destroy the parameter server](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/parameters/#cleaning-up), which de-allocates any memory created on the microcontroller. It is important to note that since the parameter server is added to the executor, the executor should be destroyed first. So, the following line needs to be added after the `rclc_executor_fini()` function is called.

```cpp
rc += rclc_parameter_server_fini(&param_server, &node);
```

Finally, once the parameter server was correctly set up, I moved on to the callback function `paramCallback` which is defined earlier when adding the parameter server to the executor. This function is called every time a parameter in the server changes value.

This is where I struggled a bit. In Galactic, when a parameter was changed, I used the getter to get all four parameter values and set the RPM and the PID constants accordingly. However, this did not work in Humble for some reason (I did not investigate further), so I decided to follow [the method described in the Parameter Server tutorial](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/parameters/#callback-1). The callback function in Humble has three parameters unlike Galactic (which only has one parameter - the changed value):

* `old_param`: Old value of the parameter (`NULL` for new parameter request, but in this case, undeclared parameters are not allowed, also the parameter values are already pre-defined).
    
* `new_param`: New value of the parameter, (`NULL` for the parameter removal requests, however I've not tried removing a parameter yet).
    
* `context`: User context, configured on `rclc_executor_add_parameter_server_with_context` (In my case, no user context is configured)
    

```c
bool paramCallback(const Parameter * old_param, const Parameter * new_param, void * context)
{
  if (old_param != NULL && new_param != NULL) 
  {
    if(strcmp(new_param->name.data, rpm_ratio_name) == 0){ rpm_ratio = new_param->value.double_value; }
    else if(strcmp(new_param->name.data, kp_name) == 0){ kp = new_param->value.double_value; }
    else if(strcmp(new_param->name.data, ki_name) == 0){ ki = new_param->value.double_value; }
    else if(strcmp(new_param->name.data, kd_name) == 0){ kd = new_param->value.double_value; }

    kinematics.setMaxRPM(rpm_ratio);
    motor1_pid.updateConstants(kp, ki, kd);
    motor2_pid.updateConstants(kp, ki, kd);
    motor3_pid.updateConstants(kp, ki, kd);
    motor4_pid.updateConstants(kp, ki, kd);

    return true
  }
  else return false;
}
```

In the above code, I first make sure that the old parameter and the new parameter are not `NULL`. If this is OK, I check what parameter has changed and accordingly set the global variable (which has been pre-defined). Then I set the RPM and PID constants.

If either or both of the old and new parameters are `NULL`, I return `false`. I have no intention of supporting the addition of new parameters or the deletion of an existing parameter. By setting the `allow_undeclared_parameters` value to false earlier, I believe I cannot add any new parameters. I am not sure if I can delete a parameter though, or if I can disable this feature. For now, if I try, then I return `false`.

### Testing the implementation

Finally, I was able to build the firmware on the [Arduino IDE](https://docs.arduino.cc/software/ide-v2/tutorials/getting-started/ide-v2-downloading-and-installing), and upload it to the [Teensy](https://www.pjrc.com/store/teensy41.html). Trying it out the first time, and everything worked as expected. Once the [micro-ROS agent](https://github.com/micro-ROS/micro-ROS-Agent) was running OK, I was able to [use the command line](https://docs.ros.org/en/foxy/How-To-Guides/Using-ros2-param.html) to list the parameters defined in the micro-ROS node:

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1680985739336/eacfa8fd-63c0-4898-afcc-37b53976cb07.jpeg align="center")

Next, I was able to successfully set the parameter values and see the results by driving the robot around:

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1680985782672/29205ca7-54cd-4f08-a013-1a47693b7178.jpeg align="center")

I also realized that I cannot declare new parameters from the command line in ROS 2, I can only load parameters from a YAML file. I was however able to try to delete a parameter, and this request was successfully rejected by the server:

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1680985869594/331951a2-9406-4fe7-852f-9cc082ef93a1.jpeg align="center")

With this, I was good to go. I do want to try it without the low memory mode, but I will leave that for another project, where I don't have so many publishers and subscribers.

### Visualization update

On a side note, I have some updates when it comes to visualization, especially with Foxglove Studio. Earlier, I was publishing a [MarkerArray message](https://docs.ros2.org/foxy/api/visualization_msgs/msg/MarkerArray.html) from the robot, so that I can visualize the robot's meshes on Foxglove Studio. I thought this was the only way of visualizing the URDF on a remote Foxglove Studio without having a local ROS 2 installation. Turns out I was wrong.

%[https://twitter.com/kamathsblog/status/1642665954306818048] 

I found out that the meshes defined in a URDF do not have to be stored locally. Instead, an HTTPS URL can be used to define the mesh location. This updated URDF file can be stored locally on the computer running Foxglove Studio, and it should be able to [load the meshes directly from the internet](https://foxglove.dev/docs/studio/panels/3d#add-urdf). This also works with RViz, but in both cases, the computer running RViz or Foxglove should be connected to the internet.

Another method is by adding a [custom URDF layer](https://foxglove.dev/docs/studio/panels/3d#custom-layers) in Foxglove's 3D panel. I can simply store my URDF and corresponding meshes in a GitHub repository and provide a URL to the URDF to Foxglove Studio. This also works quite well. I decided to use this option (for no particular reason). As a result, I can remove the mesh publisher node completely. I also don't need to publish the meshes from the robot anymore, as Foxglove Studio can simply load it from the internet (and this happens on the remote computer, not on the robot computer).

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1680986920294/c803bd15-d67e-4287-8b8a-288509654912.jpeg align="center")

### Next up

I have some interesting things coming up in my next post. I got [started with Unity with ROS 2 Humble](https://blog.unity.com/engine-platform/advance-your-robot-autonomy-with-ros-2-and-unity) and I have already made some progress - I am now able to load and visualize the URDF in Unity, and also connect to the robot and visualize some topics.

%[https://twitter.com/kamathsblog/status/1642663040532217857] 

%[https://twitter.com/kamathsblog/status/1642663082278150144] 

I have some issues with the transforms, and I still haven't tried publishing messages from Unity. But before getting to that, I want to try out the [Nav2 SLAM tutorial](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example) which uses Unity for simulation and visualization. This tutorial should give me some idea about how to figure out the issue with the transforms. I'll talk more about this in my next post.

Other than this, some fun new toys arrived, which I have already started playing with. First, I received a package from Pimoroni a couple of weeks ago. It came with a [Cosmic Unicorn](https://shop.pimoroni.com/products/cosmic-unicorn) LED matrix with an [RPi Pico W](https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html) onboard, a [Teensy MicroMod](https://shop.pimoroni.com/products/sparkfun-micromod-teensy-processor), and a [MicroMod carrier board](https://shop.pimoroni.com/products/sparkfun-micromod-atp-carrier-board).

%[https://twitter.com/kamathsblog/status/1639327806457774099] 

The MicroMods are for [this open-source project](https://github.com/rosmo-robot/smartcar_shield) that I'm collaborating with. It is a robot kit that will use the Teensy MicroMod with micro-ROS support. I intend to try out some micro-ROS examples on the Teensy MicroMod before I receive a prototype board to play with. As for the Cosmic Unicorn, I've programmed it to display a lava lamp pattern and hung it up as a display.

%[https://twitter.com/kamathsblog/status/1640028947957133313] 

The second thing I received is something that I've been searching for for the last two years - a [Raspberry Pi Zero 2 W](https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/). I've found Raspberry Pi Zeros but never found a 2 W in stock. Luckily, thanks to [RPilocator](https://rpilocator.com/?instock), I was able to find it in stock at [a webshop in Belgium](https://shop.mchobby.be/en/) and placed an order immediately. And this week, it arrived:

%[https://twitter.com/kamathsblog/status/1645177582256635906] 

I've already been able to install [Ubuntu 22.04](https://ubuntu.com/download/raspberry-pi) and [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) on it and set it up for remote development ([with a JupyterLab server](https://linuxhint.com/install-jupyter-lab-raspberry-pi/)). Since it is not a RPi 4, I decided to not install a desktop environment or the ROS 2 GUI tools. Speaking of RPi 4s, I also set up a second RPi 4 with the [ROS 2 Humble + RT Kernel image](https://github.com/ros-realtime/ros-realtime-rpi4-image/releases/tag/22.04.1_v5.15.39-rt42-raspi_ros2_humble) and set it up just like the RPi 4 on the robot. The only difference is that this is the 4GB RAM version, the robot has the 8GB one. I plan on using it on [the 3 wheeled robot](https://twitter.com/kamathsblog/status/1611469922890141712) I talked about last time (I do plan on working on it, and I also see a few public holidays and long weekends coming up where I can)

I also finished my book for March. Interesting read, but not very memorable. I got a little bored reading it, which is why it took more than a month.

%[https://twitter.com/kamathsblog/status/1643405575785439233]