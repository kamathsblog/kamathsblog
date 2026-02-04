---
title: "FOSDEM 2026 - Brussels 🇧🇪"
datePublished: Wed Feb 04 2026 16:29:24 GMT+0000 (Coordinated Universal Time)
cuid: cml88t49k000002jxa4thbfr0
slug: fosdem-2026-brussels
cover: https://cdn.hashnode.com/res/hashnode/image/upload/v1770145197429/f657bc3e-cfcb-4899-aae6-01db39828d99.jpeg

---

This is going to be a quick one, unlike my much more detailed report from [ROSCon 2025](https://kamathrobotics.com/roscon-2025-singapore). I had originally planned for a full weekend in Brussels, but because of some hectic work days before and after the FOSDEM weekend, I decided on a day trip on Saturday and have Sunday for myself, at home.

## What’s FOSDEM?

If FOSDEM sounds unfamiliar, you're not alone. I only discovered it when the Robotics and Simulation devroom debuted in 2025. After hearing rave reviews, I decided to check it out this year. [FOSDEM](https://fosdem.org/2026/) is a free annual event at the [ULB Solbosch](https://www.ulb.be/en/maps-directions/solbosch) campus celebrating all things free and open source. With thousands of talks and several devrooms, each focusing on specific topics, it's a techie's paradise.

I initially planned to stick to the [Robotics and Simulation devroom](https://fosdem.org/2026/schedule/track/robotics-and-simulation/), but it was way more popular than anyone expected. Leaving for lunch, which I did, meant a long wait in line to get back in. With limited time, I opted to explore other FOSDEM areas and wander around Brussels instead of queuing up.

If you know me, I am more interested in hardware than software, so I was naturally attracted to the stands showcasing open-source hardware projects. Here are a few interesting projects I came across:

### [OpenFlexure Microscope](https://openflexure.org/projects/microscope/)

An open-source, 3D printable, Raspberry Pi-powered microscope. Not a hobby/education tool like the ESPressoscope, but real research equipment actually deployed in the medical and research sector in 70+ countries. One of the creators, present at the stand, told me about how they deployed a few in rural Rwanda, where more expensive tooling is not accessible. And looking at the live images from their demo microscopes, it looked really high quality, accurate, and in focus. All from a Raspberry Pi HQ camera, and off-the-shelf stepper motors with PLA parts for positioning. Very cool.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1770144534237/75949484-b2b8-402a-9aca-7fb5de86256d.jpeg align="center")

### Open Micro-Manipulator

At the [FreeCAD](https://www.freecad.org/) and [KiCAD](https://www.kicad.org/) stand, they demonstrated products created using these open-source CAD and PCB design tools, one of them being the [Open Micro-Manipulator](https://github.com/0x23/MicroManipulatorStepper). This is an XYZ-manipulator and motion control platform - made using NEMA stepper motors, 3D printed parts, and commercially available hardware components - achieving sub-micron precision. This was another project that blew my mind. To top that, the entire control system is powered by a Raspberry Pi Pico. I really need to dig into this project when I get the chance. I have the parts to build one, maybe I’ll make a weekend project out of it someday.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1770144543364/05148b33-b3c8-4277-a79d-94d98970046d.jpeg align="center")

### OpenPrinting

They provide open-source drivers for paper printers, like a Klipper for 2D printing. Unfortunately, there was nobody at the stand when I was there; they had probably gone for lunch. But if I ever decide to get a paper printer, I’m definitely getting one that’s compatible with [OpenPrinting](https://openprinting.github.io/).

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1770144558856/53a13cf1-b1ba-4220-939c-e5d95d64c0cf.jpeg align="center")

# Robotics and Simulation Devroom

I reached the venue about 15 minutes before the day started, and was lucky enough to get a choice of seats. I said hi to some people, and after trying out a few spots, found the perfect place to sit. But soon, as more people started coming in, the room was full.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1770144637608/80dd9ab3-da09-4b23-b064-8bebbc499c29.jpeg align="center")

I left to get a coffee and some food, and that’s when I realized how crazy it was outside - there were about 20 people in line, waiting to get in. By the time I returned, the line had gotten even bigger. I decided to first explore other areas of FOSDem and then watch the livestreams for the rest of the talks. I’ve clubbed the interesting ones into two main categories - Robots and Rust. There were a few other memorable ones, which I have another section for below.

## Robots

### HackerBot

First, [Stef Dillo](https://fosdem.org/2026/schedule/speaker/stef_dillo/), the creator of the [Tenacity Rover](https://x.com/rovertenacity), talked about another project of his - [converting a cheap robot vacuum into a ROS-powered robot base](https://fosdem.org/2026/schedule/event/PDWNCJ-map-hacking-a_cheap-robot-vac-with-open-source-sw/), with a hacked mapping solution that converts the vacuum’s internal map into a ROS-compatible costmap. He talked about his challenges, how he fixed them, and shared some general tips and tricks.

%[https://www.youtube.com/watch?v=ykVGcZc0c0A] 

It is a shame he didn’t share the actual make and model of the robot vacuum, but he did mention [HackerBot Industries](https://www.hackerbot.co/), a company that sells complete kits (but also does not mention the original manufacturer). This is something I’ve been wanting for years - a hackable robot vacuum cleaner. There’s the [iRobot Create](https://iroboteducation.github.io/create3_docs/) platform, but it does not include any vacuuming features. I guess I will have to build my own.

### Just1

In a lightning talk, [Nicolas Rodriguez](https://fosdem.org/2026/schedule/speaker/nicolas_rodriguez/) showed (and demoed live) his mecanum-wheeled robot called [Just1](https://fosdem.org/2026/schedule/event/KCPTX7-just1/). It reminded me of my time working on [AKROS](https://kamathrobotics.com/series/akros), but this one was much more affordable. Just1 costs only 250 Euros to build, and this includes a 70 Euro estimate for the [LD19](https://www.waveshare.com/wiki/DTOF_LIDAR_LD19?srsltid=AfmBOooRFtdIZVZMFMJkXW4CuEA6qHCJWrk7tTzcUQXDSZryPYrRWHO4) LiDAR, which these days costs 20-30 Euros on Chinese websites. Perfect solution for students/hobbyists. Combine the Just1 with [Linorobot](https://github.com/linorobot/linorobot2), and you’ve got a powerful robot on your hands, all for about 200 Euros.

%[https://www.youtube.com/watch?v=m_tZ3eg-KsA] 

### iXi

[Botronics](https://www.botronics.be/), a Belgian startup, did an [extensive deep-dive](https://fosdem.org/2026/schedule/event/M38A3V-botronics-robotics-tech-stack/) into the technology stack of [iXi](https://www.ixi.golf/?utm_source=website&utm_campaign=botronics), their autonomous golf-bag carrying robot. I was definitely impressed by their commitment to open-source, and they’re probably the only company I know of that is building a product without re-inventing the wheel. They use off-the-shelf parts, existing ROS 2 packages where possible, and build on top of that, contributing back to the community. And the things they’ve built on top are quite impressive as well. Do check out [their talk](https://mirror.as35701.net/video.fosdem.org/2026/ub2147/M38A3V-botronics-robotics-tech-stack.av1.webm) to learn more.

%[https://www.youtube.com/watch?v=Wpu0oe_uBDM] 

### RustyRover

In the second lightning talk session, engineers from Botronics showcased [RustyRover](https://fosdem.org/2026/schedule/event/3PBHXY-simple_safe_open_building_your_first_ros_2_rover_with_rust_and_pixi/), a DIY differential-drive robot built with Rust. They built it to experiment with Rust and assess new technologies. Botronics seems to have cultivated a really cool company culture, and it is very impressive. And a cool project as well, I’m definitely going to read through this as I slowly get started with Rust.

## Rust

### Copper-rs

[Guillaume Binet](https://fosdem.org/2026/schedule/speaker/guillaume_binet/) talked about his project, [Copper-rs](https://fosdem.org/2026/schedule/event/SK8EGJ-copper-rust-robotics-runtime/), a Rust-based, game-engine-like, deterministic framework for robotics. Very cool project and impressive results, but I am a bit too biased towards ROS, and while I do want to learn Rust, I would rather use it with ROS 2 rather than a completely new framework. But it was good to learn about Copper-rs, I’ll read more when I get the chance.

### rclrs / ros2\_rust

[Esteve Fernandez](https://fosdem.org/2026/schedule/speaker/esteve_fernandez/) talked about [rclrs](https://fosdem.org/2026/schedule/event/J8ZLKG-introducing_rclrs_the_official_ros_2_client_library_for_rust/) - the ROS 2 client library for Rust. I’ll be honest, I was in line at a food truck at the time, and I completely missed this. But I was at [Esteve’s talk at ROSCon](https://vimeo.com/1136376117) last year, so I had an idea of what he spoke about. I’m adding it to this post because this is one of the projects I am curious about, and I plan to try it out as a part of my Rust learning journey

### ROS-Z

This was another project I had heard about at ROSCon, but I believe it was a lightning talk. This was a much longer presentation about it. [ROS-Z](https://fosdem.org/2026/schedule/event/BQ8DVM-ros-z/) is simply ROS 2, but with parts of the ROS 2 stack replaced by pure Rust-based components, and using [Zenoh](https://zenoh.io/) as the middleware. So you could have a Rust-native stack, but still have your C++/Python apps. I wasn’t that interested when I first learned about it at ROSCon, but after using Zenoh, I don’t think I will move back to DDS again. Now it makes sense to move to ROS-Z.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1770144883122/75892a22-bbfc-4c7a-995e-1d11a726c8b7.png align="center")

I am a bit conflicted between working with ROS-Z or ros2\_rust as a potential side-project. But I have some time before I pick a lane. But at least after these talks, I am even more motivated to learn Rust. Now to find the free time…

## Memorable talks

Besides this, there were a few other memorable talks. [Sam Pfeiffer](https://fosdem.org/2026/schedule/speaker/sam_pfeiffer/) and [Roland Meertens](https://fosdem.org/2026/schedule/speaker/roland_meertens/) talked about [getting good-quality robot data](https://fosdem.org/2026/schedule/event/DJK8WL-calibrate-good-times/) from different types of sensors, and shared their personal experiences along with some tips and tricks. There was also a talk about [EasyNav](https://fosdem.org/2026/schedule/event/VD7GN8-easynav/), a lightweight navigation stack with some architectural decisions that make it quite different from [Nav2](https://docs.nav2.org/). Finally, [Davide Faconti](https://x.com/facontidavide?lang=en) talked about the future roadmap of [PlotJuggler](https://fosdem.org/2026/schedule/event/MNT7XJ-plotjuggler_the_log_visualization_tool_loved_by_roboticists/). I watched the live-stream while on the bus back, and laughed so hard that I could have been kicked off the bus. I’m not going to say anything else; watch this talk for yourselves.

%[https://www.youtube.com/watch?v=cy--EC-YKCQ] 

## Summary

Overall, my first FOSDEM was a bit of a hectic experience, and a little overwhelming, but in a positive way. But now I can prepare for next year, and I hope they have a bigger space for the Robotics devroom at FOSDEM 2027. I’m definitely coming back, and given the projects I am working on this year, I have things I would like to present as well.

Speaking of presenting, the talk from Botronics included a surprise at the end - the announcement of ROSCon Belgium in 2026. The spoken language is English, which means I’m definitely attending and submitting a talk when the time comes.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1770219964823/3952d6bc-f763-47e1-a735-a0b09f2d404d.png align="center")

This image is from the Botronics talk. I really like this logo, especially the Moon Rocket from Tintin; I hope they confirm it. I also managed to visit a comic book store in Brussels on my way back - got myself the newest Asterix book, and a couple of Tintin comics in Dutch (I’m getting better at Dutch, so I’m going to give this a shot). Turns out, while it is *Tintin* in both English and French, the Dutch character is called *Kuifje*.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1770221886061/fac5d43c-9f5e-432c-abff-14f3b5cfe98b.jpeg align="center")

# What’s next?

Thanks for FOSDEM, I have even more project ideas now. But this year, I want to focus on only a handful of projects, especially after registering a business - I want to continue doing projects with the LeKiwi robot, I’ve got a long-term product idea I’m working on (more on this soon), and I want to learn Rust. Luckily, the talks were quite relatable to the things I’m working on, and I’m glad. Now, back to working on LeKiwi’s [ros2\_control implementation](https://github.com/adityakamath/sts_hardware_interface)…