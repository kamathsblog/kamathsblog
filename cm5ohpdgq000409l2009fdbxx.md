---
title: "2024: A year in review"
datePublished: Tue Dec 31 2024 23:00:00 GMT+0000 (Coordinated Universal Time)
cuid: cm5ohpdgq000409l2009fdbxx
slug: 2024-a-year-in-review
cover: https://cdn.hashnode.com/res/hashnode/image/upload/v1736040990505/96d74763-b209-47b7-ae6b-b54c69009de5.jpeg

---

2024 has been one of the most eventful years of my life, especially since the pandemic when I began my professional career in the Netherlands. However, because it has been so eventful in my professional and personal life, I have been unable to work on as many personal projects as usual. Still, I did manage some fun projects this year and reused parts from earlier projects to build new ones. Here’s a recap of everything I did in 2024.

## Steam Deck as a Robot Controller

I started the year by buying a [Steam Deck](https://store.steampowered.com/steamdeck/) after I noticed they lowered the price by about 150 Euros for the LCD version when they launched the new OLED version. This turned out to be my favorite purchase of the year. Since then, I've set it up with [Distrobox](https://wiki.archlinux.org/title/Distrobox) and [ROS2](https://wiki.archlinux.org/title/Distrobox) (blog linked below) and now use it as a dedicated development machine for my robotics projects and experiments. After buying a MacBook this year and selling my Windows laptop, the Steam Deck has become more of a ROS device than a gaming console. Still, I found some time to enjoy games like [Hogwarts Legacy](https://store.steampowered.com/app/990080/Hogwarts_Legacy/) and [Portal 2](https://store.steampowered.com/app/620/Portal_2/), to name a few.

## Arvatar

For those who don’t know me, I am an engineering consultant by profession. For work, I work on client projects, which since I’m in Eindhoven revolve around semiconductor manufacturing and photolithography systems thanks to ASML. At the beginning of this year, I took a break from a client project at ASML and decided to work on internal research and innovation projects. I started with Arvatar - a robotic system that can be teleoperated / remote-controlled using a VR headset. When I joined, the team was in a weird state - they were going in the right direction with the technology (intending to use ROS, use Unity for VR, had acquired a Meta Quest ) but were focusing mostly on the mobile robot - which ended up as a 6 wheeled lunar/mars rover design with some burnt motors, loosely fitting mechanical parts and the team did not even have any full-time mechanical designers.

So, the first task was to use an existing commercial robot we had in our labs. We chose a 12-year-old, [4-wheeled mecanum platform](https://www.piscinarobots.nl/4wd%20mecanum%20wheel%20mobile%20robotic%20platform/kit%20-%20kr0003) that had been unused for three years since an intern last worked with it. It needed some upgrades, though - the software wasn't fully available, it was in ROS 1 (soon to be EOL, and this was in January), and while the motors were in good shape, the motor drivers weren't working properly. Since we wanted to get this up and running quickly, I decided to replicate my AKROS project on this similar (but slightly bigger) platform. My project is open-source and is already a fork of another open-source project [Linorobot2](https://github.com/linorobot/linorobot2), so licensing wasn’t an issue. So we ended up with the [same motor drivers as AKROS2 alongside a Teensy 4.1 microcontroller](https://adityakamath.github.io/2022-06-06-akros-teensy-update/), modified and reused the [micro-ROS firmware](https://github.com/adityakamath/akros2_firmware), and reused [ROS2 nodes and launch files](https://github.com/adityakamath/akros2_base) on a Raspberry Pi 4. Since we had a laser scanner ([rplidar\_ros](https://github.com/Slamtec/rplidar_ros)), an OAK-D ([depthai-ros](https://github.com/luxonis/depthai-ros)), and a [Sense HAT](https://www.raspberrypi.com/documentation/accessories/sense-hat.html) (LED matrix, IMU and some buttons/sensors) ([sensehat\_ros](https://github.com/adityakamath/sensehat_ros)), we also attached them all and integrated them into our launch files. For testing, we used an XBox controller ([joystick\_drivers](https://github.com/ros-drivers/joystick_drivers), [akros2\_teleop](https://github.com/adityakamath/akros2_teleop)) and planned on replacing it with messages from the VR headset. We also created a URDF and updated the Gazebo simulation the team had worked on before I joined. For visualization and debugging, we used [Foxglove](https://foxglove.dev). For networking, we used [Tailscale](https://tailscale.com/) ([ROS 2 and VPNs](https://hashnode.com/post/clldxzbqv00000al3du13309f)). Since I had already implemented all this for AKROS, getting the Arvatar robot up and running took literally less than 2 weeks of work (+ 2 weeks of knowledge sharing, waiting for budget approvals, and then for the parts to arrive) and we had this within a few months:

%[https://twitter.com/kamathsblog/status/1794014926547104203] 

For the VR, I already had some experience with [visualizing AKROS in Unity](https://hashnode.com/post/clh3scj29000b09l3bcehg2ah), so I could guide the process - first, we visualize the robot's state, including sensor data and camera streams, in Unity. Next, we add a camera in Unity and project that view to the Meta Quest. Then we get data from the hand controllers/hand tracking and publish [Joy messages](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Joy.html) to replace the XBox controller messages - this was a solid plan, but we faced issues connecting to the Meta Quest. By the time we resolved that, our Unity licenses expired, and we had to wait for budget approvals to renew them (Unity licenses are costly, especially for companies earning more than a certain amount per year - like 7,000 Euros per license). However, we managed to get the visualization working, at least as a proof-of-concept (unfortunately, I do not have an image to share here).

Another goal of the project was to remotely pick up and manipulate objects using VR, so we needed a small robot arm on the mobile robot. Our biggest mistake was deciding to design and build it ourselves instead of buying an affordable commercial one. This decision took a lot of time due to various issues as well as the summer holiday period.

%[https://twitter.com/kamathsblog/status/1811883637710655907] 

Around the same time we realized our mistake, we hired an intern to develop the control software for a small-scale robot arm and bought the commercial arm we had originally planned to purchase. Fortunately, this arm uses the same ST3215 servo motors as the one we designed, so the software the intern creates can be used for both arms.

%[https://twitter.com/kamathsblog/status/1838939024045621391] 

Meanwhile, I also began working on another innovation project for the business side of my company, which aimed to use robots for customer interaction in the retail sector. I'm not sure I can discuss what we built in a lot of detail, but I had a lot of fun working with the Pepper robot and a fantastic team of business analysts and UX/UI designers (non-engineers). Eventually, we brought in some excellent engineers who helped with the technical implementation.

%[https://twitter.com/kamathsblog/status/1777648684370403341] 

## Pan-Tilt mechanism with ROS

In spring, I decided to build a small hobby project with [STS3215](https://www.feetechrc.com/2020-05-13_56655.html)/[ST3215](https://www.waveshare.com/st3215-servo.htm) motors. We wanted to use the same motors for the Arvatar arm and I had a few that I wasn’t using. So, I decided to build a little [pan-tilt mechanism and control it using ROS 2](https://github.com/adityakamath/pan_tilt_ros), as a starting point for the Arvatar arm software stack. As you can see in the blog post below, this mechanism was then replicated for use on the Arvatar rover with an [OAK-D lite](https://shop.luxonis.com/products/oak-d-lite-1?srsltid=AfmBOors7DgMtFAim2XYm9W90Eg-IJSbnGTGmibVGnc13IyNJBAgB8gQ) camera so that it can imitate the head movements of the VR headset user and provide a more immersive experience. This was a fun little project, and you can read more about it [on my blog post here](https://kamathrobotics.com/building-a-pan-tilt-mechanism).

## RoboCup 2024 Eindhoven 🇳🇱

[RoboCup 2024](https://2024.robocup.org/) was hosted in Eindhoven this year and was pretty exciting. I was aware of and had worked with [Tech United](https://www.techunited.nl/?page_id=2135&lang=en) (the TU Eindhoven RoboCup team) robots before, but never actually seen them compete. It wasn’t just the mid-sized league that the Tech United football robots participate in, there were various events distributed across three massive sports halls, ranging from wheeled robots to humanoids and robot dogs, playing football, doing industrial / home tasks, or performing search and rescue. There was also a massive exhibition area with even more cool robots. I went on three different days, and had the best time, sometimes a bit overwhelming as can be seen in the threads below. The first day also included a Dutch ROS Users Group Meetup, which was also really interesting.

%[https://twitter.com/kamathsblog/status/1814026660984902137] 

%[https://twitter.com/kamathsblog/status/1815041558489469334] 

## Pixi

I’ve mentioned this before - I got a new Macbook this year, replacing my Windows laptop. This was also my first time using a MacOS device, so it took me a while to get used to. I did not want to [install ROS binaries](https://docs.ros.org/en/crystal/Installation/macOS-Install-Binary.html) (I’ve heard some bad experiences with this) and fully intended to use my Steam Deck and RPi devices as ROS devices which I could SSH into. That is till I heard about [Pixi](https://pixi.sh/latest/) - an open-source package manager using which I was able to set up a ROS 2 project on MacOS within minutes. I will probably write a blog about my experiences with Pixi in 2025.

%[https://twitter.com/kamathsblog/status/1822670808105296270] 

## ROSCon 2024 Odense 🇩🇰

This was a well-deserved holiday for me, and by the end, I was grateful that I treated ROSCon as a vacation rather than a work event. I was able to enjoy the conference and also explore Denmark, savoring the food and culture without worrying about work. Professionally, many people in the ROS community know me from my Twitter posts and blog, so it was helpful to have my handle on the ROSCon badge instead of my company name. I've shared more of my thoughts and highlights in the blog post linked below.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1736376386976/3f51f22e-3420-4bf7-b8e5-d118546f518b.jpeg align="center")

It was such a different experience to [Kyoto in 2022](https://kamathsblog.com/roscon-2022-kyoto) but still just as amazing. Just like Japan, this was my first trip to Denmark. I spent an entire week there, with 4 days in Odense (with 3 days at ROSCon), a day trip to Billund to visit the [Lego House](https://legohouse.com/en-gb/), and then two days in Copenhagen. I had the best time with the best people (both roboticists and non-roboticists) and some delicious food. I’ll probably do the same for ROSCOn 2025 in Singapore and have a holiday around the conference. So excited for it!

## SO-100

In October, I discovered the [SO-100 arm](https://github.com/TheRobotStudio/SO-ARM100) on Twitter. Since I already had the motors and 3D printing filament, I decided to build one. This robot arm is designed as a pair of leader and follower arms. However, to save money on buying six more motors, I chose to build only the follower arm with the gripper. I plan to record movements and then replay them using the same arm.

%[https://twitter.com/RemiCadene/status/1841081581093732723] 

I made a few small changes of my own. I chose to use 7.4V STS3215 motors from Feetech instead of the 12V ST3215 motors from Waveshare, mainly so I could power them with an 8V output from an NP-F750 battery. I designed and 3D-printed a holder to attach the battery to the arm. Then, I designed and printed another holder to mount the entire arm and battery setup onto a tripod. I used the [SCServo\_Linux C++ library](https://github.com/adityakamath/SCServo_Linux/tree/main) to make the arm work. I first tried the [LeRobot scripts](https://github.com/huggingface/lerobot) to operate the arm but couldn’t get them working, especially on my new MacBook at the time. Given that they’ve added [new documentation](https://github.com/huggingface/lerobot/blob/main/examples/10_use_so100.md) to their repository, this is something I want to try in 2025.

%[https://twitter.com/kamathsblog/status/1855619727395053830] 

Meanwhile, a fellow maker who also built an SO-100 arm used my [pan-tilt ROS project](https://github.com/adityakamath/pan_tilt_ros) to control the arm with ROS 2. Fortunately, the creators of the arm provided the [URDFs](https://github.com/TheRobotStudio/SO-ARM100/tree/main/URDF) along with the mesh files, so setting up the visualization seems straightforward. Getting the arm to work with ROS 2 is another one of my goals for 2025.

%[https://twitter.com/JohnVial/status/1855882440327454746] 

## ToF Imager + IMU Mapper

This is another fun little project I made in 2024 over a weekend. I met the team from [PiSugar](https://www.pisugar.com/) at the [Eindhoven Maker Days](https://makerdays.nl/) in June and decided to buy one for my Raspberry Pi Zero 2 W because it seemed like such a convenient power solution. Once it arrived, I wanted to add some sensors, so I chose a [VL53L5CX ToF Imaging sensor](https://www.sparkfun.com/products/18642) and a [BNO055 IMU](https://www.adafruit.com/product/2472) for this project. To connect everything using Qwiic connectors, I added the [Pimoroni Inventor HAT mini](https://shop.pimoroni.com/products/inventor-hat-mini?variant=40588023464019), which provides a Qwiic connector and some useful LEDs for status indicators. If I want to add servo and DC motors later, the Inventor HAT also makes it possible. My goal is to use the ToF Imager and IMU together to implement SLAM and experiment with visualization on [Rerun](https://rerun.io/) - another goal for 2025.

%[https://twitter.com/kamathsblog/status/1845079871908684002] 

## Touch Designer

Every year, one of my favorite events in Eindhoven is [GLOW](https://gloweindhoven.nl/), a light art festival featuring various light installations, projection mapping, and interactive projects along a walking route of about 5 kilometers. Each year, I'm inspired by the projection mapping installations and dream of learning how to do it myself, with the hope of presenting my work at a future GLOW event as an artist. Earlier this year, I decided to begin my journey into projection mapping by installing the free version of [Touch Designer](https://derivative.ca/), a visual programming tool for multimedia content, and purchasing the cheapest projector I could find online. However, I soon got busy with other projects, and then changed laptops, so I set it aside.

%[https://twitter.com/kamathsblog/status/1855336958500757997] 

Then, at GLOW 2024, I had the chance to meet some artists and designers, who along with some of my friends inspired me to take up the hobby again. I found an amazing teacher, [Pao Olea](https://www.youtube.com/@pao_olea/playlists), and began by following her [tutorials on YouTube](https://www.youtube.com/watch?v=WCPv27M_LLE&list=PLCQm2_dRv60W7myryzovHaoxRdvNA-oWJ). Eventually, I joined [her Patreon](https://www.patreon.com/pao_olea) to access more tutorials. My goal is to complete one tutorial per day. The thread below shows the results of the tutorials I followed:

%[https://twitter.com/kamathsblog/status/1864236912480747561] 

During this time, I wanted to attend some in-person workshops and discovered a [creative coding symposium called Iterations](https://creativecodingutrecht.nl/en/archive/iterations-2024), which included a TouchDesigner workshop. I also found another workshop for a tool called [Hydra](https://hydra.ojack.xyz/?sketch_id=eerie_ear_1), a video synth tool that can be programmed and run in a browser. By then, I was quite comfortable with Touch Designer, so I decided to join the Hydra workshop instead. Hosted by the talented artist [Flor de Fuego](https://www.instagram.com/flordefuega/) for a group of about 20 people, it was a fantastic experience to learn something new and meet some amazing people all while indoors on a cold Saturday morning.

%[https://twitter.com/kamathsblog/status/1865806039091757497] 

Returning to the Touch Designer aspect, once I completed all of Pao Olea’s tutorials, I started another set of [tutorials by the Interactive and Immersive HQ on YouTube](https://www.youtube.com/playlist?list=PLpuCjVEMQha9Yf1aMrAls0HlDFq6LG9-X). They have many videos, but I decided to follow their playlist on creating generative art that mimics the styles of famous artists and art pieces. I'm watching these videos daily, learning a lot not only about Touch Designer techniques but also some incredible artists, and becoming more creative along the way.

%[https://twitter.com/kamathsblog/status/1871300204391239765] 

## 2025

As we enter the new year, I already have a few projects in mind. For hardware, I have the two projects I mentioned earlier—the arm and the mapping device. I also have a [Wave Rover by Waveshare](https://www.waveshare.com/wiki/WAVE_ROVER) in case I need a wheeled robot for any experiments. I don't want to invest in more hardware (I already spent a lot on my new laptop); instead, I plan to use what I have and focus on software for 2025.

I want to focus heavily on [Rust](https://www.rust-lang.org/) in the new year, especially in the [robotics context](https://robotics.rs/). I had already planned to experiment with [Rerun](https://rerun.io/) and now I'm considering using Rust for it, as it's the language it was originally written in, instead of using its C++ or Python bindings. The mapping device I mentioned earlier seems like the perfect tool for this.

Another project I want to explore is [Dora (dora-rs)](https://dora-rs.ai/), a Rust-based robotics framework. It appears to be faster than ROS 2 and also offers a [bridge interface to ROS 2](https://github.com/dora-rs/dora/tree/main/libraries/extensions/ros2-bridge), allowing both frameworks to work together. I don't want to completely abandon ROS 2 because I really like the community. While the technology has some drawbacks, things are definitely improving, such as using [Zenoh](https://zenoh.io/) (another Rust-based project) as [an RMW alternative to DDS](https://vimeo.com/showcase/11451831/video/1024971621).

For the SO-100 arm, I planned on getting it working with the [LeRobot framework](https://github.com/huggingface/lerobot) and then connecting it with [HuggingFace](https://huggingface.co/). While learning more about Dora, I also discovered [Dora-LeRobot](https://github.com/dora-rs/dora-lerobot), a complete Dora pipeline for interfacing with LeRobot hardware, and I'm eager to learn more about it.

As always, all of my experiments will be shared in a blog or at least a [Twitter](https://x.com/kamathsblog) post. I'm also on [BlueSky](https://bsky.app/profile/kamath.bsky.social) these days, so you can expect some posts there too. I'm not leaving Twitter for now, as it is still much more entertaining.

Touch Designer will also be very important for me in 2025. I plan to follow tutorials and eventually create something new every day. I want to use the [OAK-D](https://shop.luxonis.com/products/oak-d?srsltid=AfmBOoo-O2yvAj4kFuIYc-VWQSU79OsYssLPOleay6D-Hi1VQAYKo55F) in my work to generate visuals based on color thresholds, depth, and even object detection. Fortunately, [Touch Designer already supports interfacing with an OAK-D camera](https://derivative.ca/UserGuide/OAK-D). Next, I want to find a way to include other sensors and microcontroller devices in my visuals. I know there are methods to connect [Arduino projects to Touch Designer](https://docs.derivative.ca/Arduino), and I'm curious if I can use a Raspberry Pi instead, along with the ToF Imager + IMU mapper, for something similar.

Besides these projects, I am also planning a trip to Singapore in 2025 and intend to include ROSCon as part of the trip. I expect to spend at least a week there, with a couple of days dedicated to ROSCon 2025. With that said, I hope you have an amazing new year, and see you around! :)