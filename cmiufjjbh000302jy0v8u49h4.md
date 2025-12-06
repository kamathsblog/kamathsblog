---
title: "ROSCon 2025 - Singapore 🇸🇬"
datePublished: Tue Dec 02 2025 23:00:00 GMT+0000 (Coordinated Universal Time)
cuid: cmiufjjbh000302jy0v8u49h4
slug: roscon-2025-singapore
cover: https://cdn.hashnode.com/res/hashnode/image/upload/v1762687247289/44fbce7b-b527-49ee-a2f0-8fcc24dfd5ba.png

---

It’s been a month since I returned from Singapore, and after fighting jetlag and a messed-up sleep cycle, I'm finally free and fresh enough to start writing about my [ROSCon 2025](https://roscon.ros.org/2025/) experience. By this time, you might have already read some blogs about this year’s ROSCon, but mine will be a bit different. I use these blogs as a reference for myself, and there are quite a few topics I want to come back to during the upcoming winter holidays. So, besides personal experiences from ROSCon, I’ve embedded all the interesting videos and topics that will be a part of my holiday playlist. A little warning, this means that this post will be long. Feel free to jump to a particular section using the table of contents.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1765033439653/438980d3-edce-430e-a91c-1a7a58551b57.jpeg align="center")

Once again, ROSCon was a self-sponsored trip, as a hobbyist and part-time freelancer, and a part of my holiday itinerary (I call them 'ROSCon holidays'). I spent a week in Singapore, nearly 8 days, and explored the city after the conference days. However, now that I’m back, I'm actually thankful for the Dutch weather, compared to the heat in Singapore. What I do miss, though, is the food, especially the hawker centers and the late-night food options. More about this later. Let's go back to the first day of ROSCon.

# <s>ROSCon</s> Day 1:

I didn’t want to attend the workshops, so I decided to skip this day. But I should definitely have arrived a bit earlier, at least to fight the jetlag and acclimatize. Plus, while I should have been resting after arriving in Singapore, I ended up having quite an eventful first day in Singapore.

%[https://twitter.com/kamathsblog/status/1982738649646293237] 

I touched down in Singapore after a 13-hour direct flight, breezed through immigration (I’m still surprised at how seamless it was), and after a quick coffee, hopped on the MRT towards [Kampong Gelam](https://www.visitsingapore.com/neighbourhood/featured-neighbourhood/kampong-gelam/), my home for the next week. By this time, I hadn’t stepped out of the air-conditioning, so I had no idea what the weather felt like outside. Note that I had flown in from Amsterdam, where it was windy, rainy, and 8 degrees. I stepped out in 32 degrees, blaring sun, no clouds or breeze, which was a real shock as I walked to my hotel. I don’t think I ever got used to the weather, though…

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1765033433251/f98a98a3-833d-46ee-8458-a5ac647f713e.jpeg align="center")

After a quick trip to the Gardens by the Bay to watch an amazing light show, with some other ROSCon attendees, I headed to the Aerial Robotics meetup at a nearby pub. Although I haven't worked on aerial robotics in ages, these meetups are always fun and casual, not strictly about aerial robots. After a drink and some enlightening conversations, I set off for food, but not before checking out the massive LED wall at the conference venue, just across from the pub.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1763232292935/bee3cfe1-784b-44c4-ace2-a753d7b00d67.jpeg align="center")

Next stop, [Satay Street](https://www.laupasat.sg/sataystreet/). Every evening, the street next to [Lau Pa Sat](https://www.laupasat.sg/), a famous hawker center, is blocked off and turns into a lively spot lined with stalls offering all kinds of satay. I sat down with a small bunch of chicken and squid satay and an ice-cold glass of freshly squeezed sugarcane juice, a really satisfying meal.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1765033729415/4309d6db-8511-41e4-be45-dc9350424df8.jpeg align="center")

# ROSCon Days 2 - 3:

I had planned my schedule for the two days on my flight to Singapore and knew my focus areas. Ever since my first ROSCon, I've learned that the conference is about talking to people, discussing ideas, and sharing experiences rather than sitting through all the talks, especially since they will be recorded and [available online](https://vimeo.com/showcase/11987695) later. So, I planned to sit only for talks that were actually relevant to my current work and to my planned projects for the coming year. And of course, the lightning talks for inspiration. The remaining time was scheduled for visiting the exhibition areas, attending meetups, meeting new people, and catching up with friends from previous ROSCons and people I've known for a while, but only via social media.

## Keynotes

Just like last year, the keynote speeches are always really interesting. This year, [Tiffany Cappellari](https://www.linkedin.com/in/tiffany-cappellari-199174149/) and [Sebastian Castro](https://www.linkedin.com/in/sebastian-a-castro/) shared their experiences using ROS 2 (and ROS tools such as [ros2\_control](https://control.ros.org/rolling/index.html)) and other open-source tools for their AI-based research projects at the [Robotics and AI Institute (RAI)](https://rai-inst.com/). If this is of interest to you, do read [Sebastian Castro’s blog](https://roboticseabass.com/2025/10/31/roscon-2025-highlights-from-singapore/) - he talks more about it there. He also has different interests from me, so his experiences are quite different, too.

%[https://vimeo.com/1136158846] 

Next, [Michael Carroll](https://x.com/carromj) from [Intrinsic](https://www.intrinsic.ai/) shared insights on the current state of ROS 2 and what lies ahead. I'm really excited to dive deeper into [Kilted](https://docs.ros.org/en/rolling/Releases/Release-Kilted-Kaiju.html) and explore the [future roadmap, which includes support for embodied AI, Rust workflows](https://osralliance.org/osra-initiatives-announced-at-roscon-2025/), and all the exciting features planned for the next ROS 2 release, [Lyrical Luth](https://docs.ros.org/en/rolling/Releases/Release-Lyrical-Luth.html). I also found out that they usually release new ROS 2 versions on [World Turtle Day](https://www.worldturtleday.org/), May 23rd, so I'll have a nice present for my birthday on May 24th!

%[https://vimeo.com/1136205502] 

## Zenoh

*Context: After last year’s ROSCon, I switched all my current ROS 2 projects from* [*FastDDS*](https://fast-dds.docs.eprosima.com/en/stable/) *to* [*Zenoh*](https://zenoh.io/)*. It works seamlessly as an alternative to DDS, although I never actually had the headaches with FastDDS that people seem to complain about. I was definitely curious to see what* [*ZettaScale*](https://www.zettascale.tech/) *was working on, especially microcontroller support, and I’m glad they addressed it during the conference.*

On Day 1, they had a full workshop and a mini workshop on Zenoh, focusing on the new features of Zenoh and [rmw\_zenoh](https://github.com/ros2/rmw_zenoh). I am keeping an eye out for the workshop content to be made available online. I followed their [2024 workshop](https://github.com/ZettaScaleLabs/roscon2024_workshop) via their GitHub repository, which was really helpful, though I'd recommend the hands-on workshop for complete beginners.

[Yadu](https://www.linkedin.com/in/yadunundvijay/) from Intrinsic and [Julien Enoch](https://www.linkedin.com/in/julienenoch/) from Zettascale covered the high-level details in their talk **“Zenoh Strikes Back: From a New Hope to Tier-1”**, covering [Zenoh’s tier-1 support](https://docs.ros.org/en/rolling/Installation/RMW-Implementations/Non-DDS-Implementations/Working-with-Zenoh.html) from [ROS 2 Kilted](https://docs.ros.org/en/kilted/) onwards, and some exciting examples that have given me some inspiration for upcoming projects.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1763225006821/5ff839f0-1332-4be2-9cca-2350620fd06f.jpeg align="center")

My biggest takeaway was Zenoh’s microcontroller support with [zenoh-pico](https://github.com/eclipse-zenoh/zenoh-pico) and its ROS wrapper [pico-ros](https://github.com/Pico-ROS/Pico-ROS-software), which I plan to use in future projects, replacing [micro-ROS](https://micro.ros.org/), which, in my opinion, is a little overkill for small applications. This allows me to use ROS for small microcontroller projects, giving me access to all the visualization, logging, and diagnostics tools in the ROS ecosystem.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1763233762483/39495aba-92ef-4787-8d2f-5a91dae4ccb0.jpeg align="center")

Combined with the other presented improvements, I was convinced to continue using Zenoh for my current projects and switch to Zenoh entirely for future ones. This talk is a must-watch for people interested in this topic.

%[https://vimeo.com/1136377715] 

If this does not convince you, [Dexory](https://www.dexory.com/) delivered a fantastic presentation, “**From DDS to Zenoh: Migrating the Dexory Autonomy ROS Stack—Configuration, Performance, and External Integration”**, about their gigantic robot (still the coolest robot I’ve seen at a ROSCon), and how they migrated their complex software stack from CycloneDDS to Zenoh.

%[https://twitter.com/kamathsblog/status/1849020315944247404] 

It was really quite impressive how (relatively) easy it was to migrate. It was good to know they are seeing actual improvements with a massive system of 200+ nodes, extremely large datasets, and external integrations. They shared some tips and best practices, which I don’t think I need at the moment, but I’m definitely bookmarking their talk to watch later.

%[https://vimeo.com/1136375669] 

Another talk I’m bookmarking is **“rmw\_what? Implementing the ROS 2 Middleware Interface**,” which explains how the [ROS 2 Middleware Interface](https://github.com/ros2/rmw) [(RMW)](https://github.com/ros2/rmw) works. I arrived a little late for this, but I was able to see the part where [Christophe Bédard](https://github.com/christophebedard) from [Apex.AI](http://Apex.AI) explained RMW using a fun email-based implementation called [rmw\_email](https://github.com/christophebedard/rmw_email). I’m definitely curious to watch the entire talk now.

%[https://vimeo.com/1136204122] 

## Industrial / Controls

*Context: The main robotics challenges that I usually come across are still in the industrial sector, especially with big robot arms that are either quite old (5-10 years), or automation that includes proprietary software. And this is both from the perspective of an independent roboticist and an engineering consultant. I was curious to see how ROS plays a part in this landscape, and how mature it is for industrial applications. I also wanted to know how ROS 2/ros2\_control can be used with PLC/fieldbus-based communications and proprietary software stacks from robot manufacturers.*

Luckily, [Dr. Denis Stogl](https://www.linkedin.com/in/denisstogl/) from [b»robotized](https://en.b-robotized.com/) had all the answers. In the workshop on Day 1, they used [ros2\_control](https://control.ros.org/) with Zenoh-Pico on an ESP32, and heard a lot of nice things about it, and even saw the final demo a few times during the conference. Luckily, they already have their [2025 workshop material](https://control.ros.org/rolling/doc/resources/roscon2025_workshop.html) available online, and I plan to explore it at my own pace during the upcoming holidays.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1763897009536/7505877c-361c-41c0-ac55-17bf737edc1e.png align="center")

During a mini workshop on Day 1 (that I also missed), Denis showcased their [b»controlled box](https://en.b-robotized.com/b-controlled-box), an industrial controller with ready-to-use ROS 2 and ros2\_control support. I did, however, get to see a similar showcase in Odense last year, but I’m curious to know what’s new with this controller platform in 2025.

Fortunately, Denis covered the key updates to ros2\_control in two talks I attended: **“ROS-Controls Project Update”** and **“ros2\_control Goes Industrial”**. The first talk highlighted the impressive updates to the [ros2\_control](https://control.ros.org/rolling/index.html) package, including the support for a wide range of actuators and robots, from hobbyist to industrial. Key takeaways were the introduction of asynchronous hardware updates, real-time improvements, and enhanced introspection/diagnostics.

%[https://vimeo.com/1136205235] 

The next day's talk focused on industrial applications, offering significant insights into common industrial protocols, their use cases, and the benefits of using ros2\_control. It also provided clear steps for integrating these protocols with ros2\_control. I did not get the chance to talk about the b»controlled box with Denis in person, but jetlag left me too sleepy in the afternoons to approach anyone, so I sat for some talks instead. Fortunately, I now know how to reach out to him, and he seems approachable for questions.

%[https://vimeo.com/1136205404] 

Earlier in the afternoon, [Doosan Robotics](https://www.doosanrobotics.com/en/) presented their talk **“Toward Scalable Collaborative Robot Controllers: Internalizing ROS 2 With Zenoh”**, which links ros2\_control with Zenoh. I wish I could have joined, but I was busy at a meetup. I’ve added this talk to my list of talks to watch during the upcoming holidays

%[https://vimeo.com/1136204650] 

## Open-RMF

*Context: Open-RMF immediately came to mind when they announced the city for ROSCon 2025. Developed with Singaporean authorities, every Open-RMF application I've heard of is based there, making it a must-see topic. In my free time, I’ve been vibe-coding a web-based UI for monitoring my DIY wheeled robots and* [*SO-101/100*](https://huggingface.co/docs/lerobot/en/so101) *arms, and I was considering using Open-RMF for fleet management and monitoring. So, I was eager to learn about its latest features and maturity levels, and to see examples of its deployment in Singapore. And the talks did not disappoint!*

The track had five talks, showcasing [Open-RMF](https://www.open-rmf.org/) and all the varied real-world examples in different environments:

* **Open-RMF Project Update**: [Michael Gray](https://greyxmike.info/) from Intrinsic provided a detailed overview of the latest in Open-RMF, highlighting their new interface-based architecture. A key takeaway was their effort to integrate with the [VDA5050](https://github.com/VDA5050/VDA5050) standard used by most existing industrial mobile robots in the EU, enhancing compatibility for industrial deployments. Michael also introduced two new tools: [Crossflow](https://github.com/open-rmf/crossflow), a low-code workflow editor, and [Site Editor](https://github.com/open-rmf/rmf_site), an improved replacement for [traffic-editor](https://osrf.github.io/ros2multirobotbook/traffic-editor.html). Both are Rust-based projects with ROS interfaces.
    
    %[https://vimeo.com/1136163138] 
    
* **Robots at Your Service: Deploying Open-RMF in Singapore’s Hospitality Industry**: The team from [KABAM Robotics](https://kabam.ai/) showcased some really cool examples of RMF-powered robots in the hospitality industry, primarily used to deliver items to hotel rooms. They discussed the requirements for such projects, explained different workflows, and introduced a tool that enables automated phone calls to hotel room telephones directly from the robot.
    
    %[https://vimeo.com/1136206211] 
    
* **Optimizing Hospital Robotics Deployments With Open RMF: Implementing Emergency Handling and Dynamic Task Orchestration**: Next, the team from [CHART](https://www.cgh.com.sg/chart) showcased Open-RMF deployments in Singapore hospitals, highlighting unique challenges in the hospital environment, especially during emergency scenarios.
    
    %[https://vimeo.com/1136163298] 
    
* **Orchestrating Interoperable Indoor Robots at Scale With Open-RMF: The Changi Airport Experience**: After hotels and hospitals, the [Changi Airport Group](https://www.changiairport.com/en/corporate.html) shared how they're using Open-RMF to scale cleaning operations at Changi Airport. They cleverly use flight schedules and a smart scheduler to plan cleaning when terminals aren't busy. I also loved how they matched robot colors with the (human) cleaning staff's uniforms, really cute.
    
    %[https://vimeo.com/1136163354] 
    
* **Workflows for Multi-Agent Orchestration**: This talk by Michael delved deeper into Crossflow, the tool introduced in his first presentation. I wish I could stay longer for this talk, but I had to leave for a meetup. Another addition to my holiday watchlist.
    
    %[https://vimeo.com/1136377685] 
    

Overall, this track was easily my favorite at ROSCon 2025. Witnessing real-world examples of Open-RMF and ROS across such diverse scenarios was fascinating, and I’m genuinely impressed by how supportive the Singapore government is of these initiatives. While I'm definitely inspired to explore Open-RMF in the coming year, I don’t think it will be helpful in my current or upcoming projects. For now, I will stick to my vibe-coded, hacky, browser-based monitoring tool (which I will clean up and share one day).

## Meetups

Meetups are one of my favourite parts of ROSCon. Last year, it got a bit overwhelming when I joined every other meetup, but this year I limited it to a select few topics I’m actually interested in. It is always fun to meet new people, share ideas, and gain inspiration.

### Mobile Manipulators

[Stephanie Eng](https://www.linkedin.com/in/stephanie-eng-00a9281b4/) from [Rockwell Automation](https://www.rockwellautomation.com/) organized a meetup for those working with mobile manipulators - robot arms mounted on mobile robots. The main challenge discussed was controlling the entire robot as a single unit, rather than managing the arm and mobile base separately. Currently, the most common method is to use [MoveIt](https://moveit.ai/) for the arm and [Nav2](https://docs.nav2.org/) for the mobile base, treating them as separate entities. We shared our experiences and some hacky workarounds, but the session was too short to arrive at a meaningful conclusion. But it was good to know that there’s a shared interest in solving this problem, and hopefully the community finds a solution.

![Mobile Manipulators meetup. Photo credit: Stephanie Eng](https://cdn.hashnode.com/res/hashnode/image/upload/v1762803277098/1363a6ed-cbd7-4b87-890f-aace911bda90.jpeg align="center")

*\[Mobile Manipulators Meetup. Photo credit: Stephanie Eng on Whova\]*

### AI in Robotics Software Development

I attended several impromptu meetups on AI tools, mainly as a user, but also because I am involved with AI tools for software development for my full-time job. I was also curious to see how they are used in the robotics industry. While I didn't get much feedback from larger companies (that probably have enterprise licenses for these tools), [Cursor](https://cursor.com/) seems to be a favorite among freelancers and startups / smaller companies. Key takeaways for reliable use:

* Always provide AI agents with context, including rules, project requirements, and coding styles. You can even use [LaTeX in markdown format](https://docs.github.com/en/get-started/writing-on-github/working-with-advanced-formatting/writing-mathematical-expressions) for math-heavy code; apparently, it works really well.
    
* These tools work best with open-source projects and libraries, but then proprietary/legacy libraries are rare at ROSCon.
    
* Good prompting is definitely important for good results. Leverage tools such as [custom prompts in GitHub copilot](https://github.com/github/awesome-copilot) or [MCP servers](https://code.claude.com/docs/en/mcp) in Claude / Cursor. This very cool [ROS 2 MCP server](https://github.com/kubja/ros2_mcp_gateway) was shared by an attendee in the Whova app, which I intend on using for my next ROS 2 project.
    
* [Claude Code](https://www.claude.com/product/claude-code) excels in context awareness and handling large codebases, while Cursor is popular for general-purpose programming. Nobody really talked about any other tools; one person mentioned [Gemini](https://gemini.google.com/app), and quite a few (researchers/students) mentioned using [NotebookLM](https://notebooklm.google.com/) for research.
    

![Shared by Stephanie Eng on Whova](https://cdn.hashnode.com/res/hashnode/image/upload/v1762802487988/04683458-7efb-4e09-b40d-82ac8986850b.jpeg align="center")

*\[Meme source: Stephanie Eng on Whova\]*

I'll stick with Claude for now until my current project wraps up, then I might switch back to Cursor for a comparison to decide which one to use long-term.

### Freelancing / Life as an Independent Roboticist

This was hands down my favorite and most inspiring meetup at the conference. [Jennifer Buehler](https://jennifer-buehler.com/), an independent roboticist, organized it to share ideas and tips about working independently as a contractor or freelancer. Hearing from long-term freelancers about their journeys and how they manage projects with different clients was really enlightening. As a part-time freelancer funding my robotics hobby and trips like ROSCon, this session inspired me to register with the Chamber of Commerce here and do it more formally going forward. The tips on workplace insurance, liabilities, and taxes were super useful.

![[Davide Faconti sharing his experience as an independent roboticist, juggling personal projects, and general tips about working independently. Photo credit: Kimberly McGuire]](https://cdn.hashnode.com/res/hashnode/image/upload/v1762802726638/62d2e0a9-e28e-41e3-a58f-1a9e7a99e07e.jpeg align="center")

*\[*[*Davide Faconti*](https://www.linkedin.com/in/facontidavide/) *sharing his experience as an independent roboticist, juggling personal projects, and general tips about working independently. Photo credit: Kimberly McGuire on Whova\]*

Jennifer also created a community platform to connect with other independent roboticists, network, share opportunities, and collaborate on projects. This is perfect for me, as I can't take on some projects due to conflicts of interest with my full-time consulting job, and I met others in the same boat. Now, when I get a project I cannot work on, I have a community to refer to instead of just declining the opportunity. And it’s certainly nice to have a community, as working by yourself can be quite isolating without a connection to other like-minded people.

![Meme source: Kimberly McGuire on Whova](https://cdn.hashnode.com/res/hashnode/image/upload/v1762802861294/81962372-161f-4353-a942-9b6f469ff68c.jpeg align="center")

*\[Meme source: Kimberly McGuire on Whova\]*

It was also good to meet [Kimberly McGuire](https://www.mcguirerobotics.com/), another independent roboticist, who had just jumped into the freelancing world. I had met her earlier at the ROS Aerial meetup and had a nice, long conversation about freelancing and a potential new project we could collaborate on.

Besides full-time, experienced freelancers, I also met others who were aspiring to be one, and it was interesting to hear their perspective as well. As somebody who is in the middle of these two groups - I’ve done small freelancing projects, but I’m aspiring to formalize it into a business - this entire session was quite enlightening.

## Exhibition Area

This is another essential part of any ROSCon: sponsoring companies showcasing their products and services, as well as an opportunity to speak with their engineers. I spent a lot of my time here, talking to some interesting people and learning about all these amazing products, some of which would be useful to me in the coming year.

There were certainly some standouts. [Intrinsic](https://www.intrinsic.ai/) had an incredible booth where they showcased their products like [Flowstate](https://www.intrinsic.ai/flowstate) (and some ROS stickers, which I nearly missed out on again). [Clutterbot](https://www.clutterbot.com/) brought their toy collection robot for homes. It is a cute robot, but it’s still in its conceptualization stage. [Kabam](https://kabam.ai/) [Robotics](https://kabam.ai/), a major local robotics company specializing in security robots, brought some of its platforms, but unfortunately, they were all stationary.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1763233796801/1a85ece9-998a-4e55-9a02-7ec8296ff655.jpeg align="center")

However, I was able to see one of their robots in action near the [Merlion](https://www.wisemove.sg/post/the-merlion-what-does-it-mean-to-singapore-then-and-now) after the first day of talks, which was surprising but pretty cool.

%[https://twitter.com/kamathsblog/status/1983202405266387394] 

At the [Foxglove](https://foxglove.dev/) stand, [Mat Sadowski](https://www.linkedin.com/in/mateuszsadowski/) brought his SO-100 Leader/Follower robots and visualized them on Foxglove. Mat used the [SO-100 URDF](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation) that I had worked on, so I was glad to see a little contribution of mine showcased at ROSCon. I’d made it yellow to match the [LeRobot](https://huggingface.co/lerobot) logo colors, but it probably should have been made purple for the Foxglove demo (and also to match the really cool purple leader and follower arms), hopefully for next ROSCon.

It was really lovely to finally meet Mat and discuss hobby projects, as well as using Foxglove for them (especially the [Foxglove SDK](https://docs.foxglove.dev/docs/sdk) for non-ROS projects). We talked about the [Robotics and Simulation Dev room at FOSDEM 2026](https://fosdem.org/2026/schedule/track/robotics-and-simulation/), and I also got some [Weekly Robotics](https://www.weeklyrobotics.com/) stickers from him.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1763299800100/e5f3105f-623d-4190-8ffb-17753dcc9a28.jpeg align="center")

[Robotis](https://en.robotis.com/) had a big stall and showcased its products: a [lineup of robots for physical AI](https://ai.robotis.com/#overview-of-the-robotis-physical-ai-lineup) and their [Dynamixel actuators](https://www.dynamixel.com/whatisdxl.php). With the [smaller arms](https://ai.robotis.com/omx/hardware_omx.html), they had a 6-cup stacking challenge using the leader and follower arms, and the best ones would win prizes. Unfortunately, I’m left-handed and had to do it with my right hand, but I was lucky to clock in at 27 seconds. The winner did it in an impressive 16 seconds.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1763233845596/91ec38a3-4d77-4e39-a8fa-0392b6f40d15.jpeg align="center")

They also showed their [AI Worker](https://ai.robotis.com/ai_worker/introduction_ai_worker.html), a dual-armed, (top) half-humanoid (with [Robotis’ OMY platform](https://ai.robotis.com/omy/introduction_omy.html)). I like these kinds of humanoids, as I feel the legs are mostly useless if the robot is mostly stationary and only occasionally moves on flat factory floors. *Use wheels ffs*. They also showcased their [AI platform](https://ai.robotis.com/), which I might try out with my SO-101 in the future, as it is integrated with [LeRobot](https://ai.robotis.com/omx/setup_guide_lerobot.html). Their tiny motor, used in their robotic hand, is truly impressive, but as I’m writing this post, I am unable to find links to either the motor or the hand. I am hoping they post more details soon.

%[https://www.youtube.com/watch?v=sYmO_tBlhag] 

[AgileX](https://global.agilex.ai/) and [D-Robotics](https://en.d-robotics.cc/) showcased their products at a small stall, but with some impressive demonstrations. Still, I was left a little disappointed by Chinese companies like Unitree and DJI, especially since they produce some amazing products and are only a 3-4-hour flight away.

%[https://www.youtube.com/watch?v=sqxh9Wv1ibI] 

I was definitely fascinated by D-Robotics’ product lineup of controller boards, single board PCs, and depth cameras, and I’m looking forward to using their products in some future projects.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1763233916425/34baa704-bf53-4527-9af8-ad524cc5c121.jpeg align="center")

[Sony](https://ai.sony/projects/robotics/) presented their new [dToF Depth Camera](https://www.sony-semicon.com/en/technology/industry/tof.html), which seemed quite impressive in their demo application using a [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/). However, they seem to be focusing on the B2B market currently and do not have the sensor available to purchase for consumers (especially hobbyists like me). A bit of a bummer, but I’m glad to see Sony active in the space.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1763233966496/0cadd0b9-6e39-4128-8e4b-687ff78d7a45.jpeg align="center")

While I had a fantastic time, I had much higher expectations, especially after Kyoto and Odense. I felt that the space was smaller this year, which meant no massive robots like the one from Dexory, and most of the exhibited robots were mostly stationary. I saw more live robot demonstrations at another event I attended later in the week. It was called [SWITCH](https://www.switchsg.org/), and I visited it after ROSCon at the [Marina Bay Sands expo](https://www.marinabaysands.com/expo-and-convention.html), which is connected to the iconic hotel and mall. I never meant to attend this event, but it was too hot outside, I had a few hours to kill, and I did not want to spend my time in a mall.

I met some amazing robots from Singapore there, and I have a couple of highlights. First, [Rolo Robotics](https://www.rolorobotics.com/) and their chicken-and-fries vending robot; it would have been a big hit to have their machine at ROSCon, especially during the breaks and for meetups. Next, an interesting application from [Lenovo](http://lenovo.com/us/en/glossary/ai-robots/?srsltid=AfmBOoo6A-IZUAvZ7Th7Rvhb7Dxco5NVzko9RsGsNjcnBXmtw15RWRSn) (I never knew that Lenovo also made robots), where they collaborated with [IBM’s AI solutions](https://www.ibm.com/watson) and an impressive sensor suite to detect issues such as gas leaks and fires, and it is currently deployed at Changi Fire Station. The robot, called [Daystar GS,](https://www.red-dot.org/project/lenovo-daystar-bot-gs-69754) features ROS interfaces.

%[https://twitter.com/kamathsblog/status/1984088446324109658] 

## General thoughts on ROSCon 2025

Let's start with the positives. While there were some shortcomings, I'm really pleased with how I focused on specific areas of interest this time, rather than attending every track and meetup like I did last year. That approach was overwhelming, so narrowing down my sessions helped manage my time and sanity. I plan to watch the recordings of missed sessions at my own pace as and when I need them; I’ve also got a playlist for the upcoming holidays.

This time management allowed me to connect with more like-minded people and discuss shared interests, which was inspiring for my current and future projects, both professionally and as a hobbyist/freelancer. The community, as always, was fantastic. There were people from all walks of life, ranging from those with no knowledge of ROS and a curiosity to learn more, to individuals like me who worked with ROS for fun, and to those who actually built the framework as a full-time job. As I mentioned earlier, it inspired me to take freelancing more seriously as a business, rather than just a means to fund my trips to conferences like ROSCon.

Beyond the conference (and the intense weather), Singapore is a really vibrant city. I stayed in Singapore for a few days after ROSCon, visiting local attractions, hanging out in cool neighborhoods, and enjoying the best food. I've been dreaming about the food since I registered, and it exceeded my expectations. Even the conference food was really good, especially during the coffee breaks. Speaking of food, I nearly finished the bingo card that Kimberly shared on Whova (I had to make some substitutions due to my shellfish allergy).

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1762795883160/5c4f25df-d177-469f-97d4-5196ed0daf90.jpeg align="center")

Now, onto the not-so-great parts. Compared to [ROSCon 2022 in Kyoto](https://kamathsblog.com/roscon-2022-kyoto) and [ROSCon 2024 in Odense](https://kamathsblog.com/roscon-2024-odense), this year’s venue could have been better. The screens were small and far from the speaker, making it hard to focus on both the slides and the speaker at the same time. At times, the seats in front of the speaker were completely empty, with people choosing to sit in front of the slides instead. I wasn’t a speaker, but I bet it would have been weird for the speakers, too…

Overall, it wasn’t too bad for a conference this size, but there was room for improvement. Maybe my expectations were too high after Kyoto and Odense. Also, I haven’t been to many developer conferences either. But that was my only complaint, about the venue. But ROSCOn is more about the people and the community, who were as open, approachable, and inspiring as ever, making up for the venue's shortcomings. [Josh Newans](https://github.com/joshnewans) from [Articulated Robotics](https://www.youtube.com/@ArticulatedRobotics) has made a fantastic video from ROSCon that showcases how vibrant the event actually is. He’s definitely chosen the best people to interview, and everyone seems to echo the same point - the community is the best thing about ROSCon - and I completely agree. By the way, if you watch Josh’s videos but haven’t read the [Articulated Robotics blog](https://articulatedrobotics.xyz/tutorials/) yet, you are really missing out.

%[https://www.youtube.com/watch?v=-c2e5jkZc6s] 

This is why I keep attending ROSCons whenever I can, but ROSCon 2026 is in Toronto, which I’m not entirely sure I can attend. For those who are going, hope you have fun, and don’t forget to shitpost on Whova! The food won’t be as good as in Singapore (a month later, and I still miss it), but I bet the weather will be perfect. Following the trend, I am assuming ROSCon 2027 will be in Europe, and I will definitely be there!

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1762875241632/64e96fe8-52ce-40c2-87e8-f5c9a7f0202a.jpeg align="center")

# What’s next

Returning to the Netherlands, it took me a week to get back to feeling normal (some melatonin helped for sure), and another few weeks to write this blog. But I do return with some great memories, a haul of souvenirs, and a massive set of ideas, which in many ways line up with some plans I already had before flying to Singapore.

As I mentioned earlier, ROSCon inspired me to pursue robotics as a side gig more formally, and I have some plans for that. I will write more about this in another post. For now, I want to keep doing projects and blogging about them, because it is fun. This also helps with exposure to potential clients, collaborators, or just fun people you could have drinks with at ROSCon.

For my projects, I first need a new robot platform, especially since I retired the AKROS mecanum robot. I have been thinking about this for a while and have made a few prototypes. Since [LeKiwi](https://github.com/SIGRobotics-UIUC/LeKiwi/tree/main) was launched, I have been tempted to build one. I have been working on a more compact version of the LeKiwi robot base using some smaller wheels that I had (and salvaged parts from the AKROS build). Unfortunately, it was too small to carry a SO-100/101 arm on top, so I was already going through the LeKiwi BOM, and considering upgrading.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1764985636148/540571e6-ef95-45fe-b061-5e730e8695d6.jpeg align="center")

At ROSCon, I finally got to meet [Juan Miguel Jimeno](https://x.com/joemeno), the creator of [Linorobot](https://github.com/linorobot), a project I based my [AKROS](https://kamathsblog.com/retiring-the-akros-platform) robot on. Amongst other things, we had a nice discussion about how buying a robot kit is so much easier and mostly cheaper than designing one and sourcing all the parts yourself, especially if the goal is to work on software. So, I went ahead and ordered the [LeKiwi kit](https://wiki.seeedstudio.com/lerobot_lekiwi/) from [SeeedStudio](https://www.seeedstudio.com/), and was able to assemble it in the last few weeks. I’m keeping the smaller robot, though. It’s a fun little toy, and there are a few design details I’m really proud of. But for now, let’s focus on the LeKiwi build.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1764985793422/38f36d9b-eb6a-4b12-8a76-52cec8bf89cb.jpeg align="center")

I was also inspired at the mobile manipulators meetup, so eventually want to a SO-100/101, but for now I will focus on the base platform, and slowly build up from here. I have some plans, and I will write about them soon.

On the software side, I also wanted to dig deeper into [ros2\_control](https://control.ros.org/rolling/index.html) and [Zenoh](https://zenoh.io/), and ROSCon has definitely inspired me to do so. I plan to follow the [ros2\_control workshop](https://control.ros.org/rolling/doc/resources/roscon2025_workshop.html), which includes [Zenoh-Pico](https://github.com/eclipse-zenoh/zenoh-pico) and [Pico-ROS](https://github.com/Pico-ROS/Pico-ROS-software) as well. Then, I want to write a ros2\_control package for the LeKiwi base. Another priority, inspired by [talks](https://vimeo.com/1136376117) and conversations at ROSCon, is [Rust](https://doc.rust-lang.org/book/). I finally want to start learning it. There are quite a few [Rust-based robotics tools](https://robotics.rs/) out there, but I want to work with [ros2\_rust](https://github.com/ros2-rust/ros2_rust), and experiment with [dora-rs](https://dora-rs.ai/) if I have the time.

I also want to visit a few conferences in Europe in 2026 and hope to meet some of you there. I surely hope that [ROSCon UK](https://roscon.org.uk/2025/) happens again next year in the same location, another chance to visit Edinburgh will be amazing. I will be attending [FOSDEM](https://fosdem.org/2026/schedule/track/robotics-and-simulation/) in Brussels in February and the [Open Hardware Summit](https://2026.oshwa.org/) in Berlin in May. I really want to visit [Hackaday Berlin](https://hackaday.com/tag/berlin/) as well, but they don’t announce it a year in advance like the other conferences, so I hope I find the announcement in time to book my trip.

Looks like I’ve got an exciting and busy year ahead, but first some downtime before the winter holidays, and a nice playlist of talks to watch during the cold, dark evenings. Hope you have an amazing winter break and see you in 2026!