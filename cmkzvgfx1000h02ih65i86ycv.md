---
title: "Vibe-Coded Robot Playgrounds"
datePublished: Thu Jan 29 2026 19:53:28 GMT+0000 (Coordinated Universal Time)
cuid: cmkzvgfx1000h02ih65i86ycv
slug: vibe-coded-robot-playgrounds
cover: https://cdn.hashnode.com/res/hashnode/image/upload/v1769717237400/31c79b84-5f75-4d2e-bf96-b6c58d87912b.png

---

A few weeks ago, I stumbled upon [CodePen](https://codepen.io/) while setting up a portfolio page after registering my company [Kamath Robotics](https://kamathrobotics.com). It is one of the embedding options available on [Hashnode](https://hashnode.com/), the platform I use to maintain this website and write my blog posts. As soon as I opened CodePen and saw what it does, I immediately knew I had to use [urdf-loaders](https://github.com/gkjohnson/urdf-loaders), a package that I had bookmarked years ago. Created by GitHub user [gkjohnson (Garrett Johnson)](https://github.com/gkjohnson), it is used for visualizing URDFs on a browser. So, I decided to use CodePen to bring my current robotics project - LeKiwi - to life.

Despite my lack of knowledge or interest in web development and the intricacies of HTML, CSS, or JavaScript (the languages needed for CodePen projects), I vibe-coded my way through using [Claude](https://claude.ai). I added a kinematics section, allowing users to control the robot with a keyboard or a gamepad, instead of controlling each joint like in Garrett’s [examples on his website](https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/).

Turns out Hashnode only allows embeddings in articles, and not individual pages, so it was a setback for my plan for an interactive portfolio page. But I didn’t want my vibe-coded AI slop go to waste, so here’s this post. I also used the LeKiwi project as a template to create visualizations for two other projects, the AKROS robot, and a similar mecanum wheel-based platform called KR0003.

Note that if you want to use the keyboard keys, you must first click on the box to enable keyboard control. Have fun driving these robots around!

## LeKiwi

> GitHub: [https://github.com/adityakamath/lekiwi\_ros2](https://github.com/adityakamath/lekiwi_ros2)

[Open-source](https://github.com/SIGRobotics-UIUC/LeKiwi) DIY omni-wheeled robot base, designed to pair with the [SO-101](https://github.com/TheRobotStudio/SO-ARM100) arm and the [LeRobot framework from HuggingFace](https://huggingface.co/lerobot). My variant is an ongoing project, aiming to evolve into a ROS 2 and [Nav2](https://docs.nav2.org/)\-powered autonomous mobile robot. The plan is to eventually add the SO-101 arm, transforming it into a mobile manipulator.

%[https://codepen.io/adityakamath/full/LEZZzoq] 

## KR0003

> GitHub: [https://github.com/adityakamath/kr0003\_description](https://github.com/adityakamath/kr0003_description)

Mecanum-wheeled robot platform, which seems to be off the market now. We have one at work, and I created a URDF for it as a weekend project and made it available online. It later became part of an internal innovation project at Capgemini Engineering, where we developed additional features, like a simulation package and teleoperation using a VR headset.

%[https://codepen.io/adityakamath/full/myEXPrK] 

## AKROS

> GitHub: [https://github.com/adityakamath/akros2](https://github.com/adityakamath/akros2)

Mecanum-wheeled robot built from scratch by yours truly. I built this during the 2020 lockdowns and kept iterating on it till 2023. This robot was the star of several personal projects, and even inspired the Arvatar project at work, which used the KR003 robot above. The visualization shows the final iteration before it was retired in 2025, as I needed components for other projects.

%[https://codepen.io/adityakamath/full/qENNvxe]