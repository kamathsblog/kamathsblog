---
title: "Retiring the AKROS Platform"
datePublished: Wed Sep 24 2025 11:53:45 GMT+0000 (Coordinated Universal Time)
cuid: cmfxxecez000002kv9ok2eaq8
slug: retiring-the-akros-platform
cover: https://cdn.hashnode.com/res/hashnode/image/upload/v1758714789164/47ab65c8-6044-4fa3-9bf2-6b154faa83e0.jpeg

---

This story starts during the Covid-19 pandemic. I found myself working on a new assignment the day the lockdowns started in the Netherlands. Since we were new to the project and had to get familiar with the tech stack, our team decided to work from the office, at least for the first few weeks. During this time, I realized three things:

1. When you work with software, especially large codebases, and legacy tech, you end up with a lot of spare time. Code needs to be built, tests need to run, and while this happens, you wait.
    
2. Working in an empty office means you no longer need to book time on the office 3D printer or laser cutter; you can just walk in and start using these tools.
    
3. Working in front of a screen for 8 hours a day is boring. And unhealthy too, both mentally and physically.
    

This was the perfect opportunity for me to sharpen my robotics skills and learn new things. My first project was to learn to use the laser cutter and upgrade my [Donkey Car](https://docs.donkeycar.com/) that I had built a few months earlier. The next step was to upgrade the Donkey Car software stack to ROS 1. Finally, I decided to [blog](https://docs.donkeycar.com/) about all of this, mainly for my own reference rather than something for public use. This pastime, based on my passion, soon became a hobby. I moved from a Raspberry Pi to a Jetson Nano, moved from a RC car to a differential drive [Jetbot](https://github.com/NVIDIA-AI-IOT/jetbot). I played around with sensors, cameras, and LEDs. Eventually, I purchased a cheap mecanum wheel-based holonomic robot kit.

This is where the story of AKROS starts. AKROS was a working title at first, a combination of my initials and ROS. However, I admit that I am not so creative when it comes to naming things, so the name stuck. For the next 3 years (2021 - 2023), this was my primary platform. I built on top of the cheap kit, adding a Raspberry Pi, a LiDAR, then an OAK-D (which I purchased from Luxonis’ Kickstarter campaign), and eventually a [Realsense T265](https://discourse.openrobotics.org/t/realsense-t265-hands-on-review/11023) SLAM camera (unfortunately, months before they discontinued it). I experimented with the [ROS Navigation Stack](https://wiki.ros.org/navigation) and [Behavior trees](https://www.behaviortree.dev/), comparing different SLAM methods and implementing navigation behaviors, most of which are now standardized in [Nav2](https://docs.nav2.org/) (by the amazing community, not by me).

I eventually moved to ROS 2, which gave me the opportunity to experience even more things - starting with migrating from ROS 1 to ROS 2, implementing [micro-ROS](https://micro.ros.org/) (during which I got to experience some cool new sensors like the VL53L5CX ToF imager and the PMW3901 optical flow sensor), and integrating ROS 2 with [Unity 3D](https://github.com/Unity-Technologies/Unity-Robotics-Hub). I also progressed on the CAD and rapid prototyping front, going through multiple hardware upgrades (including the addition of wireless charging) and eventually getting my first 3D printer so that I could print at home instead of going to the office every time. I was, of course, blogging through this entire process.

In 2024, I had the opportunity to lead a robotics project at work, where I was able to use all of these new skills to build a similar robot for internal R&D purposes. I worked on other projects involving different robots, so I had to stop working on AKROS. As I built new things, AKROS eventually ended up being an ‘organ’ donor, as I would regularly use components from the robot for the different projects I was working on. Eventually, in 2025, it was finally time to retire the platform.

%[https://twitter.com/kamathsblog/status/1955657773523349993] 

That said, here’s a list of my build logs (at least the ones I’ve been able to blog about). I know its a month later than I promised it, but I’ve been working on something cool that I will write a post about soon. Happy reading!

%[https://adityakamath.github.io/2021-01-10-new-year-new-project/] 

%[https://adityakamath.github.io/2021-03-01-akros-holonomic-update/] 

%[https://adityakamath.github.io/2021-04-03-playing-with-pointclouds/] 

%[https://adityakamath.github.io/2021-04-19-pointcloud-laserscan-filters/] 

%[https://adityakamath.github.io/2021-05-07-camera-calibration-fiducial-slam/] 

%[https://adityakamath.github.io/2021-06-09-visualizing-depthai-hector-slam-outputs/] 

%[https://adityakamath.github.io/2021-08-01-navigation-module-design-update/] 

%[https://adityakamath.github.io/2021-08-07-akros-drive-update/] 

%[https://adityakamath.github.io/2021-08-31-first-steps-with-nav-stack/] 

%[https://adityakamath.github.io/2021-09-05-comparing-slam-methods/] 

%[https://adityakamath.github.io/2021-09-26-navigation-stack-update/] 

%[https://adityakamath.github.io/2021-10-03-first-behavior-tree/] 

%[https://adityakamath.github.io/2021-10-17-holonomic-motion-planning/] 

%[https://adityakamath.github.io/2021-10-31-storing-loading-waypoints/] 

%[https://adityakamath.github.io/2021-12-12-getting-started-with-odometry/] 

%[https://adityakamath.github.io/2021-12-19-closed-loop-motor-control/] 

%[https://adityakamath.github.io/2021-12-26-holiday-upgrades/] 

%[https://adityakamath.github.io/2022-01-02-final-update-2021/] 

%[https://adityakamath.github.io/2022-01-09-review-2021/] 

%[https://adityakamath.github.io/2022-01-23-wireless-charging-update/] 

%[https://adityakamath.github.io/2022-03-22-visualizing-akros-in-foxglove/] 

%[https://adityakamath.github.io/2022-06-01-ros2-setup-on-windows/] 

%[https://adityakamath.github.io/2022-06-06-akros-teensy-update/] 

%[https://adityakamath.github.io/2022-06-19-microros-examples/] 

%[https://adityakamath.github.io/2022-06-26-more-microros-examples/] 

%[https://adityakamath.github.io/2022-07-02-even-more-microros-examples/] 

%[https://adityakamath.github.io/2022-07-10-tof-imager-with-microros/] 

%[https://adityakamath.github.io/2022-07-17-akros2-firmware-part-1/] 

%[https://adityakamath.github.io/2022-07-24-akros2-firmware-part-2/] 

%[https://adityakamath.github.io/2022-08-05-akros2-firmware-part-3/] 

%[https://adityakamath.github.io/2022-09-04-akros2-drive-and-bringup/] 

Around this time, I transitioned from GitHub Pages to Hashnode, a more convenient blogging platform. Unfortunately, I’ve been unable to embed Hashnode links here, so here is a list of the AKROS-related blogs on the new platform:

* [2022: A year in review](https://kamathrobotics.com/2022-a-year-in-review)
    
* [Upgrading ROS 2, micro-ROS versions](https://kamathrobotics.com/upgrading-ros-2-micro-ros-versions)
    
* [Teleop with Game Controllers](https://kamathrobotics.com/teleop-with-game-controllers)
    
* [Updating the Robot Description](https://kamathrobotics.com/updating-the-robot-description)
    
* [Micro-ROS Parameter Server](https://kamathrobotics.com/micro-ros-parameter-server)
    
* [Visualizing Robots in Unity](https://kamathrobotics.com/visualizing-robots-in-unity)
    
* [Steam Deck as a Robot Controller](https://kamathrobotics.com/steam-deck-as-a-robot-controller)