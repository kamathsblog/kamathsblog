# Recap: ROSCon 2022 Kyoto

2022 was without a doubt an amazing year, as it included my very first trip to Japan and my very first [ROSCon](https://roscon.ros.org/2022/). ROSCon, for the uninitiated, is the Robot Operating System ([ROS](https://www.ros.org/)) conference, an annual gathering of roboticists, developers, researchers, and hobbyists. ROSCon 2022 was extra special, as the ROS community was able to meet each other after two years of virtual ROSCon events.

![ROSCon 2022, Kyoto](https://cdn.hashnode.com/res/hashnode/image/upload/v1671974944252/9eba4ad4-1387-4fd6-97a5-87f31590736b.jpeg align="center")

As a bonus, the conference was organized in Kyoto, Japan. I've always wanted to visit Japan ever since I was introduced to Japanese food here in the Netherlands. Japan had also just opened up its borders for tourism, so there were hardly any tourists or crowds there (an absolute dream for photographers and enthusiasts like me). In this post, I'll talk about the conference itself, and also link some of my favorite talks (All the talks were recorded, and the recordings can be found on the [Open Robotics Vimeo page](https://vimeo.com/showcase/9954564). The slides from these talks can be found [here](http://download.ros.org/downloads/roscon/2022/)). For my experiences outside of the conference, including photo walks and trips to different places, I've already talked about it on [my Instagram page](https://www.instagram.com/kamathsblog/), so make sure you check that out as well.

## 19th October: Workshops

I started the day with a coffee at perhaps one of the best [Starbucks](https://www.google.com/maps/place/Starbucks+Coffee+-+Kyoto+Karasuma-Rokkaku/@35.00758,135.7586584,18z/data=!3m1!5s0x60010885325b0c1d:0x73b2527a8e8758c0!4m9!1m2!2m1!1sStarbucks!3m5!1s0x60010885331bf779:0x16d8674e3a571648!8m2!3d35.0075801!4d135.7597954!15sCglTdGFyYnVja3MiA4gBAVoLIglzdGFyYnVja3OSAQtjb2ZmZWVfc2hvcOABAA) I have ever been to. I found it accidentally, after walking into a [beautiful temple](https://www.google.com/maps/place/Ch%C5%8Dh%C5%8D-ji+(Rokkaku-d%C5%8D)+Temple/@35.00746,135.7598936,19.67z/data=!3m1!5s0x60010885325b0c1d:0x73b2527a8e8758c0!4m12!1m6!3m5!1s0x60010885331bf779:0x16d8674e3a571648!2sStarbucks+Coffee+-+Kyoto+Karasuma-Rokkaku!8m2!3d35.0075801!4d135.7597954!3m4!1s0x0:0xc55a4b35e90756c8!8m2!3d35.0077132!4d135.7602563) on my way to the subway station. Right next to it was the coffee shop, with massive windows overlooking this temple. After the coffee, I took the subway to the conference venue.

![One of the most beautiful Starbucks in Kyoto](https://cdn.hashnode.com/res/hashnode/image/upload/v1671975006425/ea4969b4-1566-42b8-8aa9-c90294e7d5ff.jpeg align="center")

Day 0 of the conference was reserved for full-day workshops, and participants had a choice between four workshops. But before talking about the workshops, I cannot miss talking about the stunning conference venue - the [Kyoto International Conference Center](https://www.icckyoto.or.jp/en/), a massive brutalist structure in the middle of green hills and woods, which were slowly turning yellow/red at the start of autumn. My favorite part of the venue was certainly the pond, with swans, turtles basking in the sun, and some beautifully colored Koi swimming around. We hung out a lot there, during lunch and breaks...

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1672070792749/c183e2be-2d84-43d7-b416-78ebd50a4861.jpeg align="center")

There were four workshop choices - [ROS 2 Control](https://control.ros.org/master/index.html), [Open-RMF](https://www.open-rmf.org/), A hands-on session with the [Mini Pupper](https://minipupperdocs.readthedocs.io/en/latest/index.html), and finally a day of talks at [ROSConJP](https://roscon.jp/). As a former control engineering student, and with a few control systems related projects in mind, I chose to attend the [ros2\_control](https://github.com/ros-controls/ros2_control) workshop. ros2\_control is an open-source control framework for real-time control of robots using [ROS 2](https://docs.ros.org/en/humble/index.html). It allows users to define their own control architectures and provides the building blocks to do so, including some implementations of commonly used control-theory concepts.

We spent the entire day on a completely hands-on workshop, setting up the URDF for a simple robot, defining the controllers and hardware interfaces, and eventually simulating the project (Thanks to [The Construct's ROS Development Studio](https://www.theconstructsim.com/the-ros-development-studio-by-the-construct/)). Finally, we were shown a real-life demo of ros2\_control running on a robot dog somewhere in Barcelona over a wireless interface set up by [The Construct](https://www.theconstructsim.com/). A very useful workshop and I'm definitely looking forward to using these concepts in the near future. The workshop material is open source and well documented, it can be found [here](https://github.com/ros-controls/roscon2022_workshop).

![ros2_control workshop on Day 0 of ROSCon 2022](https://cdn.hashnode.com/res/hashnode/image/upload/v1671975043734/35e21b07-3460-4d22-9243-7ad9a5e4a19b.jpeg align="center")

## 20th October: ROSCon Day 1

Unlike the previous day of workshops, where only about 250 people participated, Day 1 had at least three times as many people. To accommodate them, the lectures were organized in much larger auditoriums as compared to the previous day. But first, I had to walk through the [exhibition area](https://roscon.ros.org/2022/img/ROSCon2022ExhibitHallFloorPlan.pdf), where robotics companies (most of them [ROSCon sponsors](https://roscon.ros.org/2022/#sponsors)) had set up their booths, and some of them even came with real robots that were just driving/walking about, showing off their skills.

%[https://www.youtube.com/watch?v=LHNH15viKV8] 

![ROSCon 2022 Day 1 morning schedule](https://cdn.hashnode.com/res/hashnode/image/upload/v1671975112856/263b9df5-7dc2-4e7d-9266-d1cc116c5a50.jpeg align="center")

First, I decided to spend some time listening to the [opening remarks](https://vimeo.com/769571056), and some of the plenary talks. One of the interesting talks in this session was by researchers from TU Dresden in Germany, who gave a talk about a ROS-based image-guided navigation system for minimally invasive liver surgery. It was a fascinating talk, as I had never really known of other ROS-based surgical applications. While this seemed like an innovative application, I was still apprehensive about its use on real humans, as ROS does not come with any safety certifications. Fortunately, this is just a research project for now, and from talking with the developers, they certainly had compliances and certifications in mind down the line. I think projects like this will force ROS developers to think about certifying their software for surgical applications, just like what [Space-ROS](https://www.therobotreport.com/open-robotics-developing-space-ros/) is trying to do for space applications. The full talk can be seen below:

%[https://vimeo.com/769636535] 

After sitting through a couple more talks, I headed to the exhibition area to talk to the different robotics companies, and also get some stickers for my bare laptop. It was an amazing experience meeting people I had known from [Twitter](https://twitter.com/kamathsblog) and [ROS Discourse](https://discourse.ros.org/) for the very first time, and seeing some robots that I had only worked on simulation with, in real life.

![Clearpath Robotics Husky with a robot arm](https://cdn.hashnode.com/res/hashnode/image/upload/v1671975141282/96d08756-5cec-4984-8dfe-bfb027bb9f68.jpeg align="center")

![The Hello Robot platform](https://cdn.hashnode.com/res/hashnode/image/upload/v1671975165107/f905cb89-a497-4083-a3a7-93c9357aac74.jpeg align="center")

%[https://youtube.com/shorts/UkKVncMAbMM?feature=share] 

I also learned a lot about the newest robots, tools, and technologies these companies were exhibiting and got a chance to even drive some of the robots on display. Next, it was time for lunch, where we were provided with an amazing bento box (I accidentally picked the vegetarian option, but it was still delicious), which we took outside to eat by the pond.

![Bento box during lunch on Day 1 of ROSCon 2022](https://cdn.hashnode.com/res/hashnode/image/upload/v1671975194146/da1696c2-0273-4843-92b9-3484a69224ac.jpeg align="center")

After lunch, the conference was broken into two tracks - Simulation and Tooling till the break, and Fleet Management and Deployment, till the end of the day. I chose to attend the Simulation track for the first half, and the Deployment track for the next few hours.

The Simulation track was quite interesting, mainly because I had only used [Gazebo](https://gazebosim.org/home) up till now, and always had issues with it. The talks in this track talked about new tools and techniques for robotics simulation with ROS - like [Unreal Engine](https://www.unrealengine.com/en-US/) for example. The talk by [Rapyuta Robotics](https://www.rapyuta-robotics.com/) focused mainly on leveraging existing technology from the gaming industry to build robot simulators - especially the photorealistic rendering capabilities and the accurate physics engines. The talk also included example architectures for a distributed simulator to simulate large numbers of robots and personal experiences of how Rapyuta Robotics uses these technologies for their business. One of my favorite talks in this track. The full talk can be seen below:

%[https://vimeo.com/showcase/9954564/video/767139975] 

The next highlight for me was the talk from [Fraunhofer IPA](https://www.ipa.fraunhofer.de/en.html) about their model-based kinematics editor. The presenter, Harsh, talked about the need for [Model Driven Engineering](https://explainagile.com/agile/model-driven-engineering/) (MDE) in the ROS ecosystem. For someone with an electrical/mechanical engineering background, this resonated with me, as I'm more comfortable working in an MDE environment than writing code. Such tooling will only reduce the barrier to entry for non-software engineers into the world of robotics with ROS. Without it, ROS will only be an afterthought for some companies, where most robotics development still happens using tools like MATLAB/Simulink. There's a lot more information about how the Fraunhofer IPA team used their tooling for kinematics modeling in the video below:

%[https://vimeo.com/showcase/9954564/video/767155036] 

Before heading to a different hall for the Deployment track, I spent some more time in the exhibition area, checking out some more cool robots. First, I went to the [Luxonis](https://www.luxonis.com/) booth to get a look at their upcoming (now a [Kickstarter campaign](https://www.kickstarter.com/projects/opencv/rae-0)) robot called [Rae](https://spectrum.ieee.org/luxonis-rae-robot-kickstarter). A tiny, cute robot with differential drive and depth cameras on both sides. I also got a chance to see it driving around, and I must say, it is quite fast for its compact size.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1672415722844/71eaa74c-e8cd-4901-915b-41618448b6df.jpeg align="center")

%[https://www.youtube.com/shorts/_U4a-otfWu0] 

Next, I headed to the [Husarion](https://husarion.com/) booth, where they had their new Mecanum wheeled robot on display. They let me drive the robot, but I nearly crashed it thanks to their joystick configuration, which was inverted as compared to the configuration I normally use. I also had a nice chat with the team, who I realized were all from Krakow, and [AGH University of Science and Technology](https://www.agh.edu.pl/en/), where I interned for a couple of months back in 2012. Good to know that AGH is still coming up with some amazing robots and robotics companies.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1672415946964/5a1e167e-dc8a-4469-9bc6-9cfbe3c406ac.jpeg align="center")

Finally, we headed to the other hall, which was part of the original building (the morning sessions were in the 'New Hall', a massive space but quite modern and plain). This second venue was much more true to the original style of the building, as can be seen in the photo below:

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1672416164875/c0d0240a-19c3-45cb-bd9c-31cb9df8b3b5.jpeg align="center")

Finally, it was time to start the Deployment track. One of my highlights in this track was the [ATOM Calibration Framework](https://www.sciencedirect.com/science/article/abs/pii/S0957417422012234), developed by researchers at the University of Aveiro. The calibration framework consists of multiple tools and scripts to calibrate a robot with multiple sensors and modalities, and then estimate transforms between these sensors and the robot hardware. A very interesting talk about an innovative tool. The ATOM GitHub repository can be found [here](https://github.com/lardemua/atom), and the entire talk is embedded below:

%[https://vimeo.com/showcase/9954564/video/767139832] 

Next, I wanted to sit through the session on ros2\_control, but then I saw a post from an engineer at [Bitcraze](https://www.bitcraze.io/) on the ROSCon app, who wanted to try out autonomous navigation using [Nav2](https://navigation.ros.org/) and their [Crazyflie 2.1 drone](https://www.bitcraze.io/products/old-products/crazyflie-2-0/) with a [multi-ToF sensor add-on](https://www.bitcraze.io/products/multi-ranger-deck/). Since I had already been to the ros2\_control workshop, I decided to skip the talk and headed to the demo. Kimberly, the engineer who hosted the meetup gave an amazing demo of the tiny Crazyflie drone mapping the environment and then navigating using Nav2. I was amazed at how well the little drone operated, as just a few years ago (during my EngD program), our team of 12 had massive issues in just getting the drone flying. The photos and video below provide a glimpse of this demo. The video shows the drone mapping the environment using a standard [SLAM](http://wiki.ros.org/gmapping) algorithm, then navigating to a user-defined goal, and finally navigating through unmapped spots in the SLAM-generated map.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1672427676650/5c16254d-a11e-4ce7-871e-382a128f8196.jpeg align="center")

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1672427656733/6c6eaedb-2dcd-4413-84ed-aacd05eea90b.jpeg align="center")

%[https://youtube.com/shorts/gRYHsQrEu0Y?feature=share] 

Finally, as the battery of the drone ran out, I headed back inside to listen to a talk about a real-life use case of ros2\_control with [MoveIt 2](https://moveit.picknik.ai/humble/index.html). The talk focused on how [Picknik Robotics](https://picknik.ai/) (the developers of [MoveIt](https://moveit.ros.org/)) helped an optics manufacturing company upgrade their robot with ROS 2 Drivers, and the challenges they faced along the way. It was nice to hear of an actual use case of ros2\_control after just learning how to use it just a day ago. Also nice to hear from a control engineer as most of the other talks seemed to be focused towards software engineers. The full video can be found below:

%[https://vimeo.com/showcase/9954564/video/767140351] 

### **Group Photo**

Finally, at the end of Day 1, all participants were asked to assemble for a group photo. It did take a while as the photographer looked around for the perfect spot to capture 800+ roboticists from all over the world in a single frame. It was funny that there was no drone present in an auditorium full of roboticists. However, we ended up with an amazing photo, as seen below.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1672071015175/d1b8caf3-258e-4c0a-8d1c-cc103874a779.jpeg align="center")

## 21st October: ROSCon Day 2

On Day 2, I decided to skip a few of the morning sessions (I was aware of the contents of many of them, and as mentioned before, the [slides](http://download.ros.org/downloads/roscon/2022/) and [videos](https://vimeo.com/showcase/9954564) were going to be made available later). As I had a limited time in Kyoto, I decided to go on an early morning walk around the landmark historic areas of the city and click some photographs. I first went to [Kiyomizu-Dera](https://www.instagram.com/p/CkSkUm7qW9E/), walked down the beautiful [Ninenzaka](https://www.instagram.com/p/CkVmTNmqrxB/) street, and spent some time around the [Hokan-ji](https://www.instagram.com/p/CkbhfrAKxM_/) temple. Finally, I walked down to the [Gion area and Yasaka shrine](https://www.instagram.com/p/CkdvjqgKTXI/) as the shops were starting to open up, and I went to the [Nishiki market](https://www.instagram.com/p/CkgOE4SKNwr/) for some food and souvenir shopping. Here, I also found another temple hidden between shops and as I went in to explore, I found a dressed-up robot arm that read my fortune:

%[https://youtube.com/shorts/7MchRV0bi68?feature=share] 

I eventually headed back to the Kyoto International Conference Center to grab some lunch by the lake where we were surprised by [Formant's robot dog](https://formant.io/news-and-blog/2022/10/26/events/4-highlights-from-roscon-2022/) once again, who came by to show off some moves:

%[https://youtube.com/shorts/NjtljF9-384?feature=share] 

Finally, after lunch, it was time to attend the track that I had been waiting eagerly for - the Navigation track. The first highlight for me was the talk about migrating from ROS 1 to ROS 2 by [Fetch Robotics](https://fetchrobotics.com/). This wasn't something entirely new for me, but the talk validated my own strategy for moving my robots from ROS 1 to ROS 2 - by migrating node by node and using [ros1\_bridge](https://github.com/ros2/ros1_bridge) so that ROS 1 and ROS 2 nodes worked together. However, this method has a few drawbacks, especially for complex node graphs, so the talk also focused on topic-by-topic migration and its advantages over node-by-node migration. This allows for incremental progress with very low overhead. It was nice to hear from the experiences of professionals in the robotics industry about their experiences and the challenges they faced along the way. I am looking forward to adopting some of these strategies in my own migration process. The full talk is embedded below:

%[https://vimeo.com/showcase/9954564/video/767140113] 

Next, Steve Macenski, the lead developer of Nav2 talked about the use of [Smac planners](https://navigation.ros.org/configuration/packages/configuring-smac-planner.html), a plugin that provides [multiple A\* based path planner implementations](https://github.com/ros-planning/navigation2/tree/main/nav2_smac_planner) for use with Nav2 on ROS 2. Steve talked about the different implementations and how they work, along with how to configure them on Nav2, and examples of these methods being used in the field. I must admit, I look up to Steve and his work, but this was the first time I had heard Steve give a talk, and I was not expecting him to speak so fast. I ended up getting a headache during the talk and did not absorb much, and had to rewatch the recording at a slower speed. The full talk is embedded below, my recommendation is to play it at 0.75x speed...

%[https://vimeo.com/showcase/9954564/video/767157646] 

The final highlight of the Navigation track, and also the talk I was most eager for, was the talk by [Davide Faconti](https://twitter.com/facontidavide) about version 4 of his [BehaviorTree.cpp](https://www.behaviortree.dev/) (BT) project, updates in this version, and the roadmap for the future. This was probably the most fun talk, as Davide included memes and jokes throughout. All this while explaining the most important updates of BT 4.0 - like the ability to add simple scripts, standardized pre and post-conditions, and states. The latter especially put an end to the Behavior Trees vs State Machines debate, by reaching a common ground. For me, these updates fixed most of the pain points of [working with BT 3.x](https://www.behaviortree.dev/migration), and I am looking forward to picking this topic up again in the coming year.

Finally, Davide talked about the updates in the behavior tree editors such as [Groot](https://www.behaviortree.dev/groot/), which has a new version to support BT 4.0, and the new [MoveIt Studio](https://www.behaviortree.dev/moveit_studio/) that also includes a BT editor, which was demoed at the [Picknik Robotics](https://picknik.ai/)' booth in the exhibition area. Both these tools will be commercial products, and I am eagerly waiting for some pricing details, especially for Groot 2.x. I already approached Picknik, but their [MoveIt Studio](https://picknik.ai/studio/) platform is focused towards big robotics companies that can purchase multiple licenses. However, they intend to release a lite version for hobbyists, researchers and individual professionals, sometime in 2023, and I cannot wait for it. Davide's full talk is linked below:

%[https://vimeo.com/showcase/9954564/video/767160437] 

After a little break, where we were provided some delicious Japanese snacks and sweets, I headed to the Hardware track - another set of talks that I was eagerly waiting for. For me, all talks in this track should have been highlights, but I ended up missing a few in order to talk to some of the speakers (mainly because this was my last full day in Kyoto, I was flying back the next evening). Of the talks I attended, the following two were the most impactful.

First, the talk by Avi Brown about [Edge Impulse](https://www.edgeimpulse.com/) with ROS 2 and [micro-ROS](https://micro.ros.org/). He introduced Edge Impulse - which is a platform for developing machine learning models for low-memory devices such as microcontrollers, and gave a few insights into his side projects using this platform, especially with micro-ROS and devices such as the [RPi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/), and the [Arduino Portenta](https://docs.arduino.cc/hardware/portenta-h7). Avi has also written two tutorials on the topic, using Edge Impulse [with ROS 2](https://docs.edgeimpulse.com/experts/machine-learning-prototype-projects/ros2-part1-pubsub-node), and with [micro-ROS](https://docs.edgeimpulse.com/experts/machine-learning-prototype-projects/ros2-part2-microros), a very useful guide for people getting started with the topic.

%[https://vimeo.com/showcase/9954564/video/767140724] 

Finally, the last talk of the track, and also of the conference - a talk about a [Raspberry Pi image with ROS2 Humble and a real-time kernel](https://github.com/ros-realtime/ros-realtime-rpi4-image). [Shuhao Wu](https://shuhaowu.com/), a freelance robotics consultant, talked about his Raspberry Pi image, which had been tuned specifically to run real-time applications with ROS 2, and how he implemented this. He also talked about the need for such an image, the challenges he faced along the way, and took us through some benchmarking results. The crux of this talk was that it is difficult for a newcomer to start with real-time programming, especially with setting up and tuning a custom kernel, and this custom image puts it all together in order to reduce the barrier of entry for newcomers. I also got a chance to flash this image on my RPi in the last few days, and so far it was a smooth experience, but I have yet to test it further by compiling/running some test ROS 2 nodes. The full talk is embedded below:

%[https://vimeo.com/showcase/9954564/video/767139709] 

### **Closing remarks**

ROSCon 2022 ended with [closing remarks](https://vimeo.com/showcase/9954564/video/767164651) by the ROSCon organizers. They told us about the amazing organizing team, gave us stats about the attendees and where they came from, and ended with a few announcements. The first was the open-source release of the PR2 robot's hardware design archive. The [PR2](https://robots.ieee.org/robots/pr2/) is one of the first robots with ROS installed and was sold to labs and universities around the world. Eventually, the software was released as a standalone framework for other robots to use. According to Open Robotics, which has been compiling and documenting all the design resources related to this robot, the archive will be open-sourced early next year.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1671979253323/fef27dce-550f-4749-8f84-8e2725eaa9e3.jpeg align="center")

Next, it was time to announce the location of ROSCon 2023. ROSCon 2020 and 2021 were both canceled due to the pandemic and were supposed to be held in New Orleans in the USA. So, it wasn't surprising when they announced [New Orleans as the venue for ROSCon 2023](https://10times.com/e1zp-zf3g-9d39).

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1671979279755/1c5f589e-84ae-4480-82b9-1767741f5f87.jpeg align="center")

I must be honest, I had mixed feelings about this announcement. Unfortunately for me, the visa process for the USA is extremely complicated compared to other countries and comes with ridiculous wait times even for a visa appointment. This means that I and many others cannot attend ROSCon 2023. It is a bummer, but as an alternative, I am now planning on attending [ICRA 2023](https://www.icra2023.org/) in London and hoping ROSCon 2024 is in a country with easier entry procedures.

## Plans for the future

My first ROSCon was certainly a special experience, as I finally got to meet a lot of people I look up to in the field of robotics, people who have inspired me on my experiences with ROS, and others whom I'd only known via social media interactions. I also learned a lot and came back with a massive list of ideas for future projects. For 2023 especially, I plan on focusing on migrating my robots to [ROS 2 Humble](https://docs.ros.org/en/foxy/Releases/Release-Humble-Hawksbill.html) and then focusing on the following frameworks:

* [ros2\_control](https://control.ros.org/master/index.html)
    
* [Nav2](https://navigation.ros.org/)
    
* [BehaviorTree.cpp 4.0](https://www.behaviortree.dev/)
    
* [micro-ROS](https://micro.ros.org/) (with [Edge Impulse](https://docs.edgeimpulse.com/experts/machine-learning-prototype-projects/ros2-part1-pubsub-node))
    
* [Unity Engine](https://blog.unity.com/manufacturing/advance-your-robot-autonomy-with-ros-2-and-unity)
    

I also want to explore MDE/[MBSE](https://insights.sei.cmu.edu/blog/introduction-model-based-systems-engineering-mbse/) techniques and see how I can apply them while working with robots and ROS. As I said earlier, this significantly reduces the barrier to entry for non-software engineers to get started with ROS, especially researchers who are working with the electrical and mechanical engineering aspects of robotics.

The top highlight of ROSCon 2022, for me, has to be its location, the beautiful city of Kyoto in Japan. Japan has always been at the top of my bucket list and I was so stoked to be there for the very first time. As Japan was just opening its borders to tourists, there were hardly any of them there (which resulted in some amazing photos without any crowds, which can be found [here](https://www.instagram.com/kamathsblog/)). This meant that I could see the real Japan, interact with locals and delve into the culture there. And I cannot forget the food, which were some of the best meals I've had in my life.

Unfortunately, due to the pandemic rules in Japan, my company had strict policies and did not allow me to extend my trip and have a small holiday there. This meant that I did not get a chance to get out of Kyoto, but I plan on (hopefully) changing this in 2023 with another trip to Japan. This time, I want to start from Tokyo, and visit other cities like Yokohama, and places like Mount Fuji.

%[https://twitter.com/tokyovisite/status/1573942392381902851?s=20&t=C6wsD7RRA_l8BOiTh70uVQ] 

To conclude, my very first ROSCon was an absolutely unbelievable experience, mostly thanks to the people (especially the organizing team at [Open Robotics](https://www.openrobotics.org/) for an excellent conference), the location, and the food. I learned a lot, met old friends, and made new ones. I also realized that I still have a lot to learn and grow as a software engineer in the field of Robotics. But there's an entire new year for that. I hope 2023 is just as exciting as 2022 (if not more), and that I get a chance to achieve some of the goals I mentioned above. If all goes according to plan, I hope to see a lot of you (including my new ROSCon friends) at ICRA 2023 in London...