---
title: "ROSCon 2024 - Odense 🇩🇰"
datePublished: Sun Dec 01 2024 13:30:09 GMT+0000 (Coordinated Universal Time)
cuid: cm45n2bkn000709mwhecrdkgv
slug: roscon-2024-odense
cover: https://cdn.hashnode.com/res/hashnode/image/upload/v1730718156781/21d316a8-7a63-49f4-ac6e-7c624c33841e.jpeg

---

If you follow me on [Twitter](https://x.com/kamathsblog) or Instagram, you’ve probably seen that I recently enjoyed a fantastic few days off in Denmark. I needed a proper holiday and decided to explore Denmark, including attending [ROSCon 2024](https://roscon.ros.org/2024/) during the trip. This was my second ROSCon, and I attended as a hobbyist this time, not representing my company. As a hobbyist, my experience was quite different from [2022 in Kyoto](https://kamathsblog.com/roscon-2022-kyoto), but it was still incredible. Here are some highlights from the trip…

## Getting There

Even though it's just a 1-hour flight from Amsterdam to Copenhagen, my journey to Odense involved several modes of transport, making it quite long. So, by the time I arrived at my Airbnb, I was exhausted. I then relaxed for a while before leaving to visit the [Hans Christian Andersen museum](https://hcandersenshus.dk/en/) and walk through the streets of Odense. Finally, I ended up at the local food hall for a pre-ROSCon meetup with others who had arrived early.

%[https://twitter.com/kamathsblog/status/1848076895406932115] 

## Meetups

The entire ROSCon event is organized on [Whova](https://whova.com/), an app where people can connect, ask questions during talks, and chat about interesting topics. People can also set up meetups to discuss specific technical subjects or plan activities like karaoke or watching football at a sports bar. I attended four of these meetups during the week, mostly at [Storms Pakhus](https://stormspakhus.dk/?gad_source=1&gclid=CjwKCAiAxea5BhBeEiwAh4t5K04RXMbqr3kPUiE7SPcAb8gVZfhrR847YTWOlo9fOHfvWxccAWWuyxoCFfkQAvD_BwE), a popular street food market in a former warehouse, a much more fun place to network than at the conference venue.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1732042103248/e196a806-8846-492a-8570-6fae2fa3e1c6.jpeg align="center")

It's a casual way to meet others in the ROS community and learn about the interesting projects they're working on. For example, at the mobile robotics meetup, I met engineers from an inspection robotics company experimenting with Gaussian splatting to create highly accurate world models. At another meetup, we met the founder of a company that makes brick-laying robots, and he showed us some amazing professional videos they made for a client over a weekend! At the aerial robotics meetup, I finally met friends I had only known from Twitter or Discord and made new ones.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1732041884877/3110182c-0a48-46f6-9e70-ec14dad6409d.jpeg align="center")

In contrast, networking at the conference venue felt very commercially driven, which didn't interest me at all as a hobbyist. I'll discuss this more in the next sections.

## UR / MiR Facility Tour

Usually, the first day of ROSCon is reserved for workshops. However, this time they also arranged a trip to the headquarters of [Universal Robots (UR)](https://www.universal-robots.com/) and [Mobile Industrial Robots (MiR)](https://mobile-industrial-robots.com/), along with several [Birds of a Feather (BoF) sessions](https://roscon.ros.org/2024/), so I chose to skip the workshops.

After registering and picking up our ROSCon badges, we took a bus to the new HQ of both these companies (which are both owned by [Teradyne](https://www.teradyne.com/robotics/)), about 20 minutes away from the conference center. We signed in (and received some cool guest badges made from floppy disks) and then had time to explore the office and attend some presentations and demonstrations.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1731954015756/e91ccc14-febf-4528-ab00-6e5e28e365fe.jpeg align="center")

I first attended a presentation on how MiR uses [Foxglove](https://foxglove.dev) to analyze robot data. It was great to see companies using ready-made tools like Foxglove instead of creating new ones from scratch. They explained how they store 30 seconds of raw data from the robot at a time, replacing it with new data every 30 seconds. This way, they can use these logs (rosbags) to analyze and fix the problem if there is an error or failure. They showed us their web-based insights platform where engineers can upload rosbags, check the error logs, and analyze minute details by visualizing the rosbag on the [older open-source version of Foxglove Studio](https://github.com/foxglove/studio), which is hosted on their cloud. They even showed us some of their rosbags and walked us through the process of analyzing an issue from the field.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1731953222233/0071e4d6-4776-4dbe-ba69-167407214a03.jpeg align="center")

Next, I attended a keynote on AI and how both Universal Robots and MiR are using AI in their products. I noticed that they weren't doing anything overly complex or over-engineered, but were focused on solving small but real problems with high reliability, like detecting wooden pallets using their forklift robots or obstacle avoidance with new and innovative sensors. They even demonstrated their [AI accelerator kit](https://www.universal-robots.com/products/ai-accelerator/) - which is simply an NVidia Jetson device and an off-the-shelf depth camera with an interface between NVidia’s AI tools and the Universal Robots arms.

%[https://www.youtube.com/watch?v=uw7STVKKUwo] 

I saw more of this in the next session - a tour of their showroom where they demonstrated various uses of their robot arms and mobile robots. Once again, their focus on reliability and repeatability (which are major requirements in industrial scenarios) was evident in their demonstrations.

%[https://www.youtube.com/watch?v=FyUHQQ-Pyhs] 

One of the coolest things I saw during ROSCon was MiR's method for locating their robots at workstations or docks. During the Foxglove session, the presenter mentioned that they use "markers" detected by their 2D laser scanner. I imagined something like a reflective barcode that the laser could scan. However, their solution turned out to be much simpler. They use a metal sheet bent in a specific way, which the laser scanner can detect through pattern matching. It's simple and effective. They use it everywhere, from docking points at workstations to powering up.

%[https://twitter.com/kamathsblog/status/1848855207767380351] 

Agreed that this method is at least a decade old, but why fix something that isn’t broken? I’ve seen so many other alternative solutions to this, but in comparison, they all feel so over-engineered. This made it clear that you don’t need fancy depth cameras or AI-based solutions, the docking problem (especially in highly structured environments) can be solved using classical tried-and-tested techniques.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1731953235589/6806734a-8d2d-4394-a96a-a0637af0d901.jpeg align="center")

After the tour, we were taken back to the conference center. I had lunch at a cozy cafe and then walked back to the conference center to attend a Birds of a Feather meeting to learn about [Zenoh, the alternative to DDS as a ROS RMW](https://discourse.ros.org/t/ros-2-alternative-middleware-report/33771).

## Zenoh BoF

The BoF began with introductions, followed by a presentation about [Zenoh](https://zenoh.io/), explaining why and how it was designed. The presenter was one of the inventors of [DDS](https://en.wikipedia.org/wiki/Data_Distribution_Service), so he had some insights to share - like the fact that DDS was invented for wired communication between equipment within a naval ship, and how it is not designed for the highly distributed and wireless robotic systems we have today. Zenoh on the other hand was designed from the ground up with robotics in mind. It is completely written in [Rust](https://www.rust-lang.org/), although with bindings for other languages like C++ and Python.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1733057863998/720092a3-acf5-4fa0-85ad-ff0555febe74.jpeg align="center")

I learned during the session and I could write an entire blog post about it, but I would recommend watching this ROSCon talk about it:

%[https://vimeo.com/showcase/11451831/video/1024971621] 

Here’s another amazing talk by the ROS 2 project lead at [Intrinsic](https://www.intrinsic.ai/), from [Foxglove’s Actuate](https://actuate.foxglove.dev/) conference this year, also talking about Zenoh with ROS 2:

%[https://youtu.be/AS7l-iDQpdY?list=PLzr0xSpP0YUF2bL9KW2lnYYe8Mg2clut-] 

During the BoF, they also announced the [release of Zenoh 1.0.0](https://zenoh.io/blog/2024-10-21-zenoh-firesong/), which I am eager to try. It is supported from [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html) onwards, so I am glad I migrated my robotics projects to Jazzy a while ago.

By the end of the day, all the networking felt a bit overwhelming, so I chose to skip the other BoF session on Gazebo and call it a day. I walked back to my Airbnb, dropped off my bags, and decided to visit the Odense harbor to relax for a while before heading to the next meetup, once again at Storms Pakhus, the food hall. Storms became the favorite dinner spot after the conference, and throughout the three days of ROSCon, groups of roboticists filled entire sections of the place.

## Intro and Keynote

The next day, the conference officially began. We kicked off with opening remarks from the organizers and a welcome from the mayor of Odense. With robotics giants like UR and MiR, along with many startups and growing companies in the area, Odense has developed a lively and expanding robotics community. Robots were visible everywhere - from mobile robots at the train station promoting events to a robot arm in the city center playing tic-tac-toe with you, and even a robot arm pouring and serving beer at the ROSCon after-party - and all of them were not just for ROSCOn, but a regular thing in Odense.

%[https://vimeo.com/showcase/11451831/video/1024971401] 

The keynote by [Deana Hood](https://www.linkedin.com/in/deannahood/) from [Vexev](https://www.vexev.com/) was quite different from the other technical talks at ROSCon, and I thoroughly enjoyed it. Deana talked about her journey with ROS and at Vexev, and how they developed a machine to autonomously create 3D vascular scans. She explained how Vexev used open-source software and tools from the ROS ecosystem to build their product and discussed the challenges they encountered along the way. This was probably one of the few talks where I was completely engaged. A must-watch!

%[https://vimeo.com/showcase/11451831/video/1024971800] 

## Talks

Speaking of talks, I realized during my first ROSCon in Kyoto that the talks are all recorded, and the conference is really about networking with like-minded people and interesting companies. So, I had already planned the few talks that I wanted to attend, and I spent the rest of the time in the exhibition area looking at real robots and talking to some amazing people.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1733058061400/6a1b8ed7-27b3-4b91-b5fc-973540f52790.jpeg align="center")

It was great to see several presentations about embedded systems, covering topics from applications with the [Nicla Vision board](https://vimeo.com/showcase/11451831/video/1026030054) to [meta-ROS](https://vimeo.com/showcase/11451831/video/1024969135), [real-time applications](https://vimeo.com/showcase/11451831/video/1024971584), and [CAN-bus implementations](https://vimeo.com/showcase/11451831/video/1026028313). Nothing too relevant to the things I am working on, but great to see that the community has a growing interest in embedded and real-time applications, and I hope to see more of it in future ROSCons.

Another interesting topic covered during the event was [Executors](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html), with two talks about [executors in ROS 2](https://vimeo.com/showcase/11451831/video/1024970052) in general, and the [multi-threaded events executor](https://vimeo.com/showcase/11451831/video/1024972104). I couldn’t attend these talks but I’m definitely making time to watch these recordings later on.

All the recordings are now online on [Open Robotics’ Vimeo channel](https://vimeo.com/showcase/11451831), which I highly recommend everyone to check out. However, in the following sections, I want to discuss the few presentations that caught my attention, out of the few that I attended.

### ROS 2 for Industrial Applications

Besides embedded applications, I was also glad to see an increased interest in industrial applications. From my consulting experience, I have realized that companies that use a lot of automation, mainly using PLCs or industrial buses like EtherCAT are quite reluctant to incorporate modern software like ROS. They normally want to work with proprietary software stacks that come with the robots they use, which means if a company uses robots from different vendors, their software stack ends up being diverse and hence difficult to maintain.

Aiming to change things, [Bosch Rexroth](https://www.boschrexroth.com/en/dc/) and [Stogl Robotics](https://en.stoglrobotics.de/) presented a talk titled “ROS 2 Gateway to Professional 24/7 Applications” where they showed how industrial equipment can connect to ROS 2 using Bosch’s [CtrlX](https://apps.boschrexroth.com/microsites/ctrlx-automation/en/) platform and [Ubuntu Snaps](https://ubuntu.com/core/services/guide/snaps-intro).

%[https://vimeo.com/showcase/11451831/video/1026037209] 

In addition to this talk, Bosch Rexroth had a booth in the exhibition area where they demonstrated this integration with a real robot on display.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1731953391257/e1acee58-60cf-4909-ace7-3ffee515acf2.jpeg align="center")

Besides Bosch, [Siemens](https://www.siemens.com/global/en.html) was also present in the exhibition area, showcasing their interface between ROS 2, industrial buses and their own compute modules.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1731967934580/8d66277b-2c3b-4b41-8a26-eb255f979d82.jpeg align="center")

They also presented this as a talk, but I wasn’t able to attend. After watching the recording, it is definitely worth checking out:

%[https://vimeo.com/showcase/11451831/video/1024971718] 

Overall, with these talks and the demonstrations in the exhibition area, it was great to see OEMs taking the initiative and offering solutions for integrating their industrial automation products with ROS 2, rather than relying on third parties to develop these open-source interfaces, which are often poorly maintained or become unmaintained packages.

### Mobile Robotics Scale-up Leveraging ROS

[Dexory](https://www.dexory.com/) made a big impact at ROSCon this year. They brought their large robot (which is actually their smaller model, and they even had it moving during the event) to Odense. These large robots are used to scan warehouses and analyze inventory. They also delivered two excellent talks at the conference.

%[https://twitter.com/kamathsblog/status/1849020315944247404] 

In their first talk, "Mobile Robotics Scale-up Leveraging ROS," the presenters discussed how they use ROS 2 in design, development, and production. They shared their best practices and key lessons from their journey. Their progress has been remarkable—from a concept diagram at ROSCon 2022 to deploying nearly 100 robots by ROSCon 2024.

%[https://vimeo.com/showcase/11451831/video/1024971160] 

It was clear how open-source software can greatly benefit a robotics company. Their journey would likely have taken much longer if they had developed everything from scratch. It was also very encouraging to see companies giving back by sharing their methods. This is a valuable lesson for many other robotics companies, especially those based in the Netherlands. If you use free and open-source contributions to build your product, you should give back to the community by sharing your knowledge and experiences. Dexory exemplifies this very well.

%[https://twitter.com/facontidavide/status/1848989454943363360] 

Their second talk, "[Accelerating the CI/CD-to-robot Cycle by 10x for 1/10th the Cost](https://vimeo.com/showcase/11451831/video/1024969227)" focused on how they use CI/CD tools in their development process. I'm new to this area, so I didn't understand much of what was discussed. However, I know I'll revisit this talk once I learn more about CI/CD and DevOps/DevEx.

### Migrating a Mobile Manipulator to ROS 2

[Michael Ferguson](https://www.robotandchisel.com/about/) shared his experience of finding a UBR-1 robot online years after the company went bankrupt and restoring it while upgrading the software to ROS 2. He discussed what he learned during this journey, some specific ROS 2 contributions he made related to the UBR-1 robot, and how he documented the process on his blog. He also mentioned his [ROS 2 Cookbook](https://github.com/mikeferguson/ros2_cookbook) repository, where he shares tips and tricks for ROS 2, which I use and recommend to others often. As a hobbyist, this was probably my favorite talk of the entire conference. Michael showed how a hobby project can lead to significant technical development and contributions to a large-scale open-source project like ROS. He has also shared resources from his talk to his very useful blog - [Robot & Chisel](https://www.robotandchisel.com/roscon24/)

%[https://vimeo.com/showcase/11451831/video/1024971060] 

### The Lighthouse Project: from Virtual Reality to Onboard Positioning for Robotics

I was already intrigued when I read the description of this talk, but after I spoke with the [Bitcraze](https://www.bitcraze.io/) team at their booth, I remembered we have some [SteamVR](https://store.steampowered.com/steamvr) base stations at work that are mostly gathering dust, so I had to attend to learn more. [Kimberly McGuire](https://www.linkedin.com/in/knmcguire/) from Bitcraze presented how they reverse-engineered the SteamVR tracking system and developed the [Lighthouse positioning deck](https://www.bitcraze.io/products/lighthouse-positioning-deck/) to position Bitcraze’s tiny Crazyflie drones accurately.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1731967832341/9b2fa5a4-d1ad-4775-9100-0cedc3e7c327.jpeg align="center")

She explained how this tracking system is much cheaper than other commercial motion capture methods while being just as effective not only for drones but also for many other types of robots.

%[https://vimeo.com/showcase/11451831/video/1024972070] 

After the talk, I spoke with Kimberly, and she showed me a small wheeled robot from Pololu that she had modified to work with this positioning system, it was really cool to see it in real life (not just on the screen). I can’t wait to see how I can integrate it into our robots at work, and if my company lets me borrow the base stations, maybe even some of my hobby projects!

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1731967774799/22c117b8-0238-48ae-addd-f5daf755c02e.jpeg align="center")

On a side note, I always tell everyone working with robots that Steam makes amazing gaming hardware that works incredibly well with robots. While many other companies make similar products, Steam's products are much cheaper because they earn most of their money from selling games and don't focus on making profits from hardware sales. I discovered this [when I bought the Steam Deck](https://kamathsblog.com/steam-deck-as-a-robot-controller), and it seems true for SteamVR systems as well.

### Lightning Talks

Finally, the most popular talks of the conference were the lightning talks. These are 2 minute pitches about interesting projects that the presenters have been working on. They also introduced a gong this year to stick to the time limits, and if I was presenting, I would definitely exceed the time on purpose to get gonged! If you had to watch only two ROSCon recordings, I would recommend watching these lightning talk videos!

%[https://vimeo.com/showcase/11451831/video/1026038503] 

%[https://vimeo.com/showcase/11451831/video/1024969633] 

## Exhibition and Demonstrations

As I mentioned earlier, aside from a few talks, I spent most of my time in the exhibition area. This was much more enjoyable - no PowerPoint presentations, just real robots demonstrating various innovative use cases. Here are the few booths and robots that caught my attention, excluding the companies I discussed earlier.

### LG Electronics

The first booth that caught my eye was [LG Electronics,](https://www.lg.com/us/business/robots) which showcased their new [Q9](https://q9.developer.lge.com/) robots, which are compact and advanced smart home companions. These small, two-legged robots have wheels on each leg and stand about 30 cm tall. Equipped with multiple cameras and touch sensors, they use AI for features like face recognition and responding to touch, as shown in the videos I've shared below. They can even display emotions, including emojis, with their expressive eyes.

%[https://www.youtube.com/watch?v=Lidm9NHMme4] 

While LG announced an [SDK](https://q9.developer.lge.com/contents/003001000/view) and compatibility with ROS, it's a shame they aren't selling it yet. They don't even have an estimate of its retail cost once they start selling.

%[https://www.youtube.com/watch?v=2w6-S1tJdbM] 

They also gave a talk about the SDK and ROS 2 compatibility during ROSCon, but I was busy with the ROS and Gazebo jigsaw puzzles in the exhibition area. Here's the recording of LG's talk if you're interested:

%[https://vimeo.com/showcase/11451831/video/1026030733] 

### Roboto

Another impressive product that caught my attention was [Roboto](https://www.roboto.ai), a multi-modal data visualization tool similar to Foxglove, but with some [exciting new features](https://www.roboto.ai/#features). They also offer a data management tool where you can store, analyze, and modify data sets like ROS bags. Additionally, they have an SDK and CLI, and it is compatible with many data formats and frameworks like ROS and PX4. My favorite feature, without a doubt, was [this one](https://www.roboto.ai/post/using-similarity-search-to-classify-events-in-drone-racing-logs):

%[https://twitter.com/kamathsblog/status/1849098986914714078] 

A feature that allows you to select a sensor event, like an unusual IMU reading, and then search for similar events in your data logs. Seems like a handy tool, useful for diagnosing recurring issues, and helps debug and identify the root cause.

### AgileX

[AgileX](https://global.agilex.ai/), a company I had only known for their incredible mobile robots, showcased their new product - a low-cost, lightweight 6dof robot arm called [PiPER](https://global.agilex.ai/products/piper) capable of handling a payload of 1.5kg. They demonstrated it in a leader-follower setup, and it was one of the smoothest demos I've ever experienced! They are selling it for 2,500 USD, which is much more affordable than similar arms from other companies. Since it also has interfaces to ROS, it's likely one of the best platforms available for hobby and research projects. I can't wait to see the projects that are built on this platform!

%[https://twitter.com/kamathsblog/status/1848856780526490039] 

### Robotec and O3DE

Another big topic at ROSCon this year was AI - not only were there several talks on the topic, but there were also a few demos in the exhibition area. [Robotec](https://robotec.org/) demonstrated [RAI](https://github.com/RobotecAI/rai), a framework that provides generative AI capabilities and other integrations for multi-modal robotics applications. They showcased several applications, including one with [Husarion](https://husarion.com/)'s [ROSBot XL](https://husarion.com/manuals/rosbot-xl/overview/) platform, which unfortunately had some technical issues by the time I visited their booth. However, my favorite was the simulated environment generator using text input with [LangChain](https://www.langchain.com/) and [O3DE](https://o3de.org/).

%[https://www.youtube.com/watch?v=q-Af5jB7lGk] 

This wasn't the first time I heard of O3DE. I came across this game engine while researching a work project, as we were looking for alternatives to [Unity3D](https://unity.com/) because the commercial licenses were a bit too expensive. Unfortunately, that project was shelved, but now that I know O3DE has [support for ROS 2](https://docs.o3de.org/docs/user-guide/interactivity/robotics/concepts-and-components-overview/), I'm definitely going to experiment with it when I get the chance.

## Closing and After Party

After all the talks, everyone gathered for the most anticipated part of the conference - the announcement of the ROSCon 2025 location. I had heard rumors during the conference that it would be somewhere in Asia, which excited me. Turns out the next ROSCon is in another country that I haven’t visited before - [Singapore](https://en.wikipedia.org/wiki/Singapore)!

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1731967690951/159af75f-0689-465d-9d88-4a0e292da388.jpeg align="center")

Next, we attended the after-party organized by the [Odense municipality](https://www.odense.dk/) at a local brewery, for ROSCon attendees and members of the [Odense robotics cluster](https://www.odenserobotics.dk/) - a cluster of robotics startups in the city. We enjoyed a delicious dinner and some free drinks, including beers that were poured by a Universal Robot arm. I don’t drink alcohol anymore, but it was still amazing to watch. Additionally, I also got the chance to meet some amazing entrepreneurs from Odense like [Clionadh Martin](https://www.linkedin.com/in/clionadh-martin/) - founder and CTO of [coalescent mobile robotics](https://cm-robotics.com/) - who showed me her fleet management app with simulated robots. ROSCon was over, but I was still getting inspiration from some amazing roboticists. Overall, an awesome end to the event!

%[https://www.youtube.com/watch?v=W99sNQdHqkM] 

## Summary

It was wonderful to meet so many people I had only interacted with on social media, and it was great to see that they were just as amazing in person as they are online. As a hobbyist, I got to talk with many others like me who not only work with robots professionally but also spend their spare time building hobby projects and contributing to this amazing community. Besides talking about robotics, I also had the chance to chat with people about other topics, like photography, trips to [Lego House](https://legohouse.com/en-gb/), and even the best [smørrebrød](https://nl.wikipedia.org/wiki/Sm%C3%B8rrebr%C3%B8d) places in the neighborhood, some of which I got to try during my stay in Odense.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1732042078869/e8059037-9ea8-4398-9ebe-800980506487.jpeg align="center")

I'm glad I took time off work for this, as it gave me a chance to relax after a hectic year, meet like-minded people in a cozy, welcoming environment, and explore the beautiful cities of [Odense](https://nds-nl.wikipedia.org/wiki/Odense), [Billund](https://en.wikipedia.org/wiki/Billund,_Denmark), and [Copenhagen](https://en.wikipedia.org/wiki/Copenhagen). The best part was having no work commitments. I didn't have to take notes to report back to my team, exchange business cards, or promote my company. Most importantly, I could just chat with people about the cool projects they are working on and share my experiences without a commercial aspect to the conversation.

%[https://twitter.com/kamathsblog/status/1849184239943565479] 

Speaking of networking, I honestly got a bit tired of it - it felt like LinkedIn in real life. A lot of the people I met in the exhibition area were just trying to promote their companies and products, which is alright but didn't interest me as a hobbyist. In these situations, I would leave the conference center and go on walks to explore the city of Odense, which was especially beautiful in autumn.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1732042011168/6ddad9ac-6fd5-4c31-8c30-b65c92527ed1.jpeg align="center")

That said, I absolutely love the ROS community and how it brings together people from various backgrounds. As people rightly say, robotics is a team sport, and it was great to see the same diversity among the attendees—from managers to software developers to electronics and mechanical engineers—all coming together to discuss robotics software. I'm already excited about the next ROSCon in Singapore! I'll be attending as a hobbyist again and have convinced some friends to join me for a week-long holiday. It's going to be amazing!

%[https://twitter.com/OpenRoboticsOrg/status/1849124642625118477] 

## Copenhagen Suborbitals

My week of tech didn't end with ROSCon. Two days after the conference (following a day trip to Billund to visit the Lego House), I traveled to Copenhagen for a few days before my flight home. My only planned activity there was visiting [Copenhagen Suborbitals](https://copenhagensuborbitals.com/) - a crowdfunded space program aiming to be the first amateur-developed crewed mission to space.

%[https://twitter.com/kamathsblog/status/1850199518454644804] 

We were given a tour by a team member who explained their rockets, how they build them, and how the organization operates. It was truly inspiring to see a group of volunteers working in their spare time to create something so incredible. They not only build rockets but also use plasma cutters, multi-axis CNC mills, and welding robots to assemble their rockets.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1732042271855/d90b1cf1-92e3-4235-bdb4-69101c0c918f.jpeg align="center")

I was surprised to see that they used Arduino boards for their control system and hobby-grade servo motors to move the fins on some of their rockets. This truly shows the power of open source, and I'm all for it! I can't wait to see them launch a human into space (and bring them back, of course) with a rocket powered by an Arduino. If you're ever in Copenhagen, be sure to check them out. You can find their guided tours on their [website](https://copenhagensuborbitals.com/about-us/guided-tours/) or through [Airbnb experiences](https://www.airbnb.ie/experiences/394401?locale=en&_set_bev_on_new_domain=1732310364_EAMTE1NzM2YjA2MT).

## Projects for 2025

With ROSCon over, I have returned with new knowledge and ideas that I am excited to use in my hobby projects. I already have my day job and a startup idea I'm working on. If you follow me on social media, you know that I'm also experimenting with visual arts and design. So, I am going to limit myself to two robotics projects for the year 2025. The first one is the [SO-Arm100](https://github.com/TheRobotStudio/SO-ARM100), a low-cost open-source robot arm designed by [The Robot Studio](https://github.com/TheRobotStudio) and intended to work with the [HuggingFace 🤗 LeRobot](https://huggingface.co/lerobot) framework.

I had a spool of filament lying around, and several [STS3215](https://kamathsblog.com/driving-serial-servo-motors) motors that were needed for this arm, which were gathering dust, so I decided to build one for myself. It's recommended to build two arms—a leader and a follower arm—for teleoperation and record/replay functions. However, I decided to build only one and plan to implement the record/replay function with this arm. I intend first to use [LeRobot](https://github.com/huggingface/lerobot)’s scripts and integration with [HuggingFace](https://huggingface.co/), and next, I want to integrate it with [dora-rs](https://dora-rs.ai/), a new robotics framework written in Rust. This framework includes a [bridge to ROS 2](https://github.com/dora-rs/dora/tree/main/libraries/extensions/ros2-bridge), so it would be easy to integrate it later on and use ROS 2 tools like RViz and [MoveIt](https://moveit.ai/). Luckily the SO-Arm100 GitHub repository provides a [URDF package](https://github.com/TheRobotStudio/SO-ARM100/tree/main/URDF) as well, which makes things easy.

%[https://twitter.com/kamathsblog/status/1855619727395053830] 

Next, I want to experiment with another Rust-based project called [Rerun](https://rerun.io/). It is a visualization tool for multi-modal data and I’ve been really interested in integrating it in a robotics project. A while ago, I built this compact [Raspberry Pi Zero 2W](https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/) setup with a [PiSugar](https://www.pisugar.com/) battery pack and some sensors. Seems like the perfect tool to experiment with.

%[https://twitter.com/kamathsblog/status/1845079871908684002] 

That said, I can’t wait to get started on learning Rust in 2025 and experiment with these projects. But first, I will take a well-deserved break and enjoy the festivities of December. Next year, I will also be attending a few conferences other than ROSCon - [FOSSDEM](https://fosdem.org/2025/) in Brussels in February, the [Open Hardware Summit](https://2025.oshwa.org/) in Edinburgh at the end of May and finally, ROSCon in Singapore. Hope you all had an amazing 2024, wish you the best for 2025, and see you at these conferences!