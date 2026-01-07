---
title: "2022: A year in review"
datePublished: Sun Jan 01 2023 04:15:52 GMT+0000 (Coordinated Universal Time)
cuid: clcmv5b3n000008kw739jeuuu
slug: 2022-a-year-in-review
cover: https://cdn.hashnode.com/res/hashnode/image/upload/v1767826829974/64b34f6f-3408-451e-bfff-7ceb32a816f3.png

---

I know it's quite late for this post, but it has certainly been difficult getting my head out of the holiday period. But it is high time I sit down and look back at the rollercoaster ride that was 2022. First, I must get myself a mug of [Bradley's No 29 tea](https://bradleys.nl/product/no-29green-white-tea-ginger-orange/), an amazing ginger and orange green tea, definitely one of my favorite finds of last year.

### Kicking off 2022

I started 2022 by finishing off the software of the AKROS robot, having configured the [ROS Navigation Stack](http://wiki.ros.org/navigation), a few behaviors using [BehaviorTree.cpp 3.x](https://www.behaviortree.dev/docs/3.8/intro), and a closed loop control system that resulted in precise, reliable motion of the robot. For the first few months of 2022, I was ready to make some upgrades - The first being adding a wireless charging module.

%[https://adityakamath.github.io/2022-01-23-wireless-charging-update/] 

The next step was to set up the [URDF](http://wiki.ros.org/urdf) of the AKROS robot with the correct meshes and be able to visualize them on [Foxglove Studio](https://foxglove.dev/). During this period, I also got a sturdy [carrying case with foam inserts](https://www.amazon.nl/gp/product/B07GTBTVRW/ref=ppx_yo_dt_b_asin_title_o03_s00?ie=UTF8&psc=1) so that I could safely carry the robot around for demos, and store it when it wasn't being used.

%[https://adityakamath.github.io/2022-03-22-visualizing-akros-in-foxglove/] 

### First steps to ROS 2 Galactic

Midway through the year, I decided that it was time to upgrade the robot even further - this time focusing on the software - from [ROS 1 Noetic](http://wiki.ros.org/noetic) to [ROS 2 Galactic](https://docs.ros.org/en/galactic/index.html). However, this was not a straightforward upgrade. My first step was to upgrade the low-level control firmware running on an [Arduino Mega](https://store.arduino.cc/products/arduino-mega-2560-rev3). One of the most important features of ROS 2 is [micro-ROS](https://micro.ros.org/) (or ROS 2 for microcontrollers), which allows a microcontroller to communicate directly with ROS 2 messages instead of relying on [ROSSerial](http://wiki.ros.org/rosserial) which converts serial messages to ROS messages on the host side. Unfortunately, micro-ROS is not compatible with the Arduino Mega, so I decided to use a [Teensy 4.1](https://www.pjrc.com/store/teensy41.html) and was luckily able to find a [breakout board](https://www.tindie.com/products/cburgess129/arduino-teensy41-teensy-41-expansion-board/) in the same form factor as the Arduino Mega. I also had to get [new motor drivers](https://www.elektor.nl/plateformes/arduino/cytron-3amp-4-16-v-dc-motor-driver-2-channels?gclid=Cj0KCQiAzeSdBhC4ARIsACj36uGFeSPXe43Jjvq8KE77ZtzBwbzDbF1cusmdiNJBuoTWSj2DAlMzZ6gaApnZEALw_wcB) that run on 3.3v from the Teensy instead of 5v supplied by the Arduino Mega.

%[https://adityakamath.github.io/2022-05-08-akros-final-update/] 

%[https://adityakamath.github.io/2022-06-06-akros-teensy-update/] 

### Experimenting with micro-ROS

With these changes, the hardware of the AKROS2 platform was ready. However, I still hadn't written any micro-ROS code yet, so first I had to learn and experiment. I started by collecting a few microcontroller devices that I had lying around - an [Arduino Portenta H7](https://docs.arduino.cc/hardware/portenta-h7) ([lite connected](https://docs.arduino.cc/hardware/portenta-h7-lite-connected)), an [Arduino Nano RP2040 Connect](https://docs.arduino.cc/hardware/nano-rp2040-connect), a [Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/), and finally, the Teensy 4.1. Over the months of June and July, I played around with [micro-ros-arduino](https://github.com/micro-ROS/micro_ros_arduino) (the micro-ROS libraries for the Arduino IDE), and the [example firmware](https://github.com/micro-ROS/micro_ros_arduino/tree/galactic/examples) with each of the different devices I had collected. I also tried the examples with different communication methods - Serial, UDP over Ethernet, and finally UDP over Wifi (of course, with the devices that supported these methods). These experiments were very useful for learning micro-ROS, especially by having these different microcontroller devices at hand. Due to technical difficulties, I had to stick with the Arduino IDE and was unable to set up a working installation of [micro-ros-platformio](https://github.com/micro-ROS/micro_ros_platformio) to use with VSCode and PlatformIO. I believe this was because I was using Windows 10 (later Windows 11) and the micro-ROS PlatformIO libraries seemed to be specifically for installations on Linux. The Arduino IDE worked, so for the time being, I decided to stick with it.

%[https://adityakamath.github.io/2022-06-19-microros-examples/] 

%[https://adityakamath.github.io/2022-06-26-more-microros-examples/] 

%[https://adityakamath.github.io/2022-07-02-even-more-microros-examples/] 

### Upgrading to AKROS2

During this period, I also made hardware upgrades to the AKROS2 robot - replacing the 3D printed Raspberry Pi cover with [a Flirc case](https://flirc.tv/products/flirc-raspberrypi4-silver?variant=43085036454120), changing the position of the [LD06 Lidar](https://www.inno-maker.com/product/lidar-ld06/), and using a matte black acrylic top panel instead of a translucent one.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1673151240203/cfcfa3ba-6188-49fc-a0ff-6d1375efbe2b.jpeg align="center")

Around this time, I first learned about the [VL53L5CX ToF imaging sensor](https://www.st.com/en/imaging-and-photonics-solutions/vl53l5cx.html), an amazing new time-of-flight sensor that could scan for obstacles over a grid of 8x8 points. Having worked only with single-point ToF sensors before, this blew my mind. So, I purchased one and decided to spend some time on a 'side quest'. My goal was to use this sensor with the Arduino Portenta, read the sensor measurements, convert them to [Pointcloud2 messages](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) and publish these messages to the ROS 2 host. It was a fun project to do, and also made me realize the potential for sensor platforms that could natively publish ROS 2 messages with the addition of a small microcontroller device to it. With the growing interest and adoption of micro-ROS, I really hope to see more such sensors on the market that can speak ROS 2 natively...

%[https://adityakamath.github.io/2022-07-10-tof-imager-with-microros/] 

Once I was done with this side-project, it was finally time to use my newly learned skills and upgrade the firmware of the AKROS2 robot from ROS 1 to ROS 2. I chose ROS 2 Galactic since it worked on the same version of Ubuntu as my earlier installation of ROS 1 Noetic. This allowed me to use [ros1\_bridge](https://github.com/ros2/ros1_bridge) to migrate my software node-by-node instead of having to do everything from scratch on an upgraded Linux distribution. My inspiration for this upgrade was the [Linorobot2](https://github.com/linorobot/linorobot2) package, an open-source project that provides ROS 2 support for different mobile robot configurations such as (2WD/4WD) differential drive and mecanum wheel based systems. I also realized that the Linorobot2 package was organized in a very similar fashion to the AKROS project, so it was definitely a good place to start. The first thing to do was migrate [my ROSSerial implementation](https://github.com/adityakamath/arduino_sketchbook_ros/tree/main/akros_holo_drive) on the Arduino Mega to [a micro-ROS implementation using the new Teensy 4.1](https://github.com/adityakamath/akros2_firmware) and the breakout board I talked about earlier. I also took the time to set up my launch files on the ROS 2 side to bring up all my nodes and drivers, and I ended up with a hybrid system with low-level control and the sensors working with ROS 2 Galactic (and micro-ROS), and the rest of the Navigation Stack working on ROS 1 Noetic.

%[https://adityakamath.github.io/2022-07-17-akros2-firmware-part-1/] 

%[https://adityakamath.github.io/2022-07-24-akros2-firmware-part-2/] 

%[https://adityakamath.github.io/2022-08-05-akros2-firmware-part-3/] 

%[https://adityakamath.github.io/2022-09-04-akros2-drive-and-bringup/] 

### ROSCon 2022, Kyoto

I decided to stop at this point because I had another big thing to prepare for - my first visit to [ROSCon](https://roscon.ros.org/2022/), which meant my first-ever visit to Japan. Japan had been at the top of my travel list for a very long time, so I had a lot to plan out for my 5-day trip to Kyoto. The trip finally happened in October, and it was definitely the top highlight of 2022 for me. I learned a lot during the talks and workshops, saw and operated some incredible new robot platforms, and got a chance to meet and network with some of the top roboticists in the world. I had been very active on [Twitter](https://twitter.com/kamathsblog) for the last few years and had made a lot of friends/connections on the platform, and it was amazing to meet some of them in real life for the first time. Not only did I get a chance to talk robots with them, but these conversations even led to a few drinks and meals after the conference, walks to some landmark sites in Kyoto, and even a souvenir shopping excursion. I ended up with a lot of new friends and loads of memories that I will cherish for a long time. I hope I get a chance to visit more ROSCons (and other robotics conferences) in the future and get a chance to meet these amazing people again. Another trip to Japan will definitely be a bonus. Read more about my ROSCon Kyoto trip [here](https://kamathrobotics.com/roscon-2022-kyoto).

### Finishing off 2022

Finally, after returning from Japan, and suffering from jet lag for a few days, it was time for a trip to India after not being able to visit for a couple of years. I had a nice time visiting my family, traveling with them, and trying out all the food that I had missed over the years. The trip was certainly hectic at times, since I was about to move to a new apartment a day after returning to the Netherlands, and I had to arrange everything while being thousands of kilometers away.

I returned a day before moving out, so I had less than 24 hours to pack everything and clean my old place before returning the keys. Moving to the new apartment was certainly more time-consuming than I had expected. I had only rented fully furnished places before this, and there were so many things I hadn't expected. Now, in 2023, I've still not completely finished setting everything up. While I have my workspace and bedroom done, I still need to fix my curtains and install ceiling lamps. Definitely my top priority for the first few months of 2023.

%[https://twitter.com/kamathsblog/status/1609247194846224385] 

Finally, I must say that 2022 has been an amazing year, perhaps my favorite year since the pandemic - I got to work on some fun projects, learned a lot, met some amazing people, played with some high-tech robots, visited Japan where I tried out some amazing food and [clicked some of my best pictures](https://instagram.com/kamathsblog), visited my family after a long time, and finally, moved to a new apartment. I hope 2023 is just as exciting, although I would like it to be slightly less hectic.

### Plans for 2023

For 2023, I do have some exciting new plans. First, I want to finish setting up my new apartment. It will take some time, so I do have a few projects I want to work on in parallel. In 2022, the ROS 2 Galactic distribution reached [end-of-life (EOL)](http://docs.ros.org/en/galactic/Releases/End-of-Life.html). So, the first robotics project for me in 2023 is to upgrade to [ROS 2 Humble](https://docs.ros.org/en/humble/index.html), which means I need to upgrade to [Ubuntu 22.04](https://releases.ubuntu.com/22.04/). This distribution does not support ROS 1 Noetic, so I cannot use the bridge technique to migrate node-by-node. However, since I have the low-level control and the high-level launch files all configured in ROS 2, setting up [Nav2](https://navigation.ros.org/) would definitely be straightforward. After learning about [BehaviorTree.cpp 4.0](https://www.behaviortree.dev/docs/intro) at ROSCon, I also want to write new behavior trees from scratch rather than upgrading my earlier attempts using the earlier version.

%[https://twitter.com/kamathsblog/status/1608869126658985984] 

The next project is to work on a micro-ROS robot. Last year, I also spent some spare time designing a three-wheeled platform that I got to assemble and wire up in the last few days of 2022. This year, my goal is to develop the software. The platform consists of three [Feetech STS3215 serial servo motors](https://www.waveshare.com/wiki/ST3215_Servo) attached to three [omniwheels](https://www.nexusrobot.com/product/58mm-plastic-omni-wheel-for-lego-nxt-and-servo-motor-14135.html), and is powered by an [NP-F750 battery](https://www.kamera-express.nl/hollyland-np-f750-battery). The system uses an Arduino Nano RP2040 Connect as the brains. For this project, I want to leverage the micro-ROS experiments that I tried out using the same Arduino device last year, and also my initial experiments with the Feetech motors.

%[https://twitter.com/kamathsblog/status/1606710886797803523] 

%[https://twitter.com/kamathsblog/status/1611394534864199683] 

%[https://twitter.com/kamathsblog/status/1611469922890141712] 

Unlike previous years, I don't want to over-estimate things and will limit myself to these two technical projects (alongside fixing up the apartment). Just like last year, I expect to have a few (unplanned) side quests along the way, and I will write about them when the time comes. That said, I will continue blogging about my experiences - the only change being that it will be here on [Hashnode](https://adityakamath.hashnode.dev/), rather than on [GitHub pages](https://adityakamath.github.io/). My old blog is archived for posterity and will remain on GitHub pages since I'm too lazy to migrate (and fix) these posts to Hashnode. I’ve also got a custom domain now: [kamathrobotics.com](https://kamathrobotics.com/)

Finally, after the ROSCon 2022 experience, I have plans to visit another robotics conference this year. [ROSCon 2023](https://10times.com/e1zp-zf3g-9d39) is happening in the USA, and I will not be able to attend. So, I will instead try to visit [ICRA 2023](https://www.icra2023.org/) in London in May. I also intend on visiting another Asian country this year before going on another trip to India. My current plans are for a return trip to Japan, but this might change - I've also heard good things about Vietnam...

That said, here's to an amazing 2023, and see you in the next update!