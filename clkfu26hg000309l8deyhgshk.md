---
title: "Upgrading my Prusa Mini"
datePublished: Sun Jul 23 2023 19:29:35 GMT+0000 (Coordinated Universal Time)
cuid: clkfu26hg000309l8deyhgshk
slug: upgrading-my-prusa-mini
cover: https://cdn.hashnode.com/res/hashnode/image/stock/unsplash/drTIJgAc58w/upload/260a4916ee134065cb80362eb159bf5b.jpeg
tags: 3d-printing, prusa, prusa-mini, octoprint, octopi

---

For a change, this post is not entirely robotics related. Last week, I received a new Raspberry Pi Zero 2 W, which I had purchased specifically to install [Octoprint](https://octoprint.org/), in order to access and control my [Prusa Mini+](https://www.prusa3d.com/category/original-prusa-mini/?gad=1&gclid=Cj0KCQjwn_OlBhDhARIsAG2y6zMS515nZivJvK1z1Yf99x-ue_dTBuK-QGnRB66b3hKkH8Gne2XsZ3gaAnPlEALw_wcB) over the internet. But first, I had to make some hardware upgrades.

My first step was to add a lamp. I searched through [Printables](https://www.printables.com/) and found [this lamp mount](https://www.printables.com/model/337150-prusa-mini-light-mount). It is actually a cold shoe mount, which means I can attach anything to it, like a camera. This mount press-fits on top of the z-axis, which was a problem as I had already printed and attached [a spool holder](https://www.printables.com/model/327378-prusa-mini-simple-spool-holder) there. So, I decided to fuse the two 3D models and created [this spool + lamp holder](https://www.printables.com/model/523146-simple-spool-shoe-mount-lamp-holder-for-prusa-mini).

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1690139833045/d68ce6c1-3df4-4d1f-97fe-dc0fe2e529a8.jpeg align="center")

While at it, I also printed and attached a [handle](https://www.printables.com/model/14610-prusa-mini-handle) to the Prusa Mini, and added a [filament sensor holder](https://www.printables.com/model/24951-prusa-mini-filament-sensor-holder).

%[https://twitter.com/kamathsblog/status/1680286978334511105] 

I eventually realized that a full spool of filament causes the 3D printer to wobble and went in search of another alternative. I could have gone back to the original spool holder that came with the printer, but I had already disassembled that and used the bearings for another 3D print that I'll talk about later. I came across two potentially useful prints on Printables - a [diagonal spool holder](https://www.printables.com/model/383327-diagonal-spool-holder-for-prusa-mini) and a [z-axis foot](https://www.printables.com/model/360734-prusa-mini-z-axis-foot). I first fused the [diagonal spool holder + lamp holder](https://www.printables.com/model/533116-diagonal-spool-holder-shoe-mount-lamp-for-prusa-mi) just like earlier.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1690139943210/178c103c-ebb9-4e72-be30-45d303e3cdc9.jpeg align="center")

This by itself was able to fix the instability issues, and with the z-axis foot, the printer is now completely stable. I eventually removed the filament sensor holder as it wasn't in the right location anymore, and it was mostly also in the way of the handle.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1690139961646/80d1fff6-07f4-46a3-9aea-4122aa0c4aaa.jpeg align="center")

Next, I decided to look for ways to attach the RPi Zero and a RPi Camera v3 to the printer. There are a lot of nice options out there, but I eventually found this [LCD screen cover](https://www.printables.com/model/489728-prusa-mini-display-box-with-raspberry-pi-zero-2-an) that's been modified to house a RPi Zero and a RPi camera. This was perfect for me, and the prints worked and fit together as expected.

%[https://twitter.com/kamathsblog/status/1683136299962716160] 

## Octoprint

With the hardware ready, I next set up [OctoPi](https://github.com/OctoPrint/OctoPi-UpToDate) (a RPi OS with OctoPrint) with my Raspberry Pi Zero 2W. The [Camera v3](https://www.raspberrypi.com/products/camera-module-3/) does not work on the [normal releases](https://github.com/OctoPrint/OctoPi-UpToDate/releases) of OctoPi. I had to look for the releases that said ['from branch camera-streamer'](https://github.com/OctoPrint/OctoPi-UpToDate/releases?q=from+branch+camera-streamer&expanded=true), which supports v3. The [latest release with Octoprint 1.9.2](https://github.com/OctoPrint/OctoPi-UpToDate/releases/tag/1.0.0-1.9.2-20230720144556) did not work for me, so I tried the [one before that](https://github.com/OctoPrint/OctoPi-UpToDate/releases/tag/1.0.0-1.9.1-20230627083607) which worked as expected. I followed the [normal installation procedure](https://all3dp.com/2/octoprint-setup-how-to-install-octopi-on-a-raspberry-pi/) and was able to get access to both the printer and the camera. I first installed the [OctoEverywhere](https://octoeverywhere.com/) plugin on OctoPrint and then installed [OctoApp](https://plugins.octoprint.org/plugins/octoapp/) on my Android phone. With this, I was all set to access my printer on my phone from anywhere.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1690140066379/77e97184-07b5-42b3-bb83-fed6a0c87d58.png align="center")

Something I still need to work on is the camera position. With the v3, the camera FOV does not capture the entire print bed. I have two potential solutions in mind - either change the camera - I have a RPi camera with a fisheye lens, or add an intermediate link between the camera and the LCD cover to help me move the camera around. Once I can get this right, I can then configure some timelapse settings.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1690140138449/93db3ee4-aa1e-4631-ac58-13150487d312.jpeg align="center")

## Other Prints

Other than the Prusa Mini+ upgrades, I also had a little 3D printing spree...

### Upcycling empty Prusament spools

Firstly, I had just finished my first two Prusament spools, and was thinking of upcycling them into something else. I started by looking through Printables and found these two amazing prints - [a turntable](https://www.printables.com/model/117059-prusament-spool-turntablerotating-stand) and a [set of shelves](https://www.printables.com/model/57186-prusament-spool-shelves). I first put together the turntable (this is where I used 3 of the bearings from the original Prusa filament spool holder), which used 2 sides of 1 spool. Then using legs and connectors from the set of shelves, I added a shelf to the top of the turntable.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1690140198148/d688b1c1-5138-41b6-8dd1-e9b13bcee0fc.jpeg align="center")

I'm so happy at how it came out, and I also love how it matches the table and the stadia controller (also my 3-wheeled robot)

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1690140220231/135d1f23-59ff-44a3-9770-ed8c13e67fd6.jpeg align="center")

### Gridfinity

I also decided to explore the [Gridfinity](https://gridfinity.xyz/) universe and build myself a soldering station. I started off with a [4x4 base](https://www.printables.com/model/291285-gridfinity-baseplate-4x4-for-screwing-down), and then printed a [4x2 container for an Omnifixo kit](https://www.printables.com/model/341180-omnifixo-helping-hands-display-and-travel-case-gri), and some internal containers which I modified to hold a set of 9 tweezers, a wire cutter, and a ruler. I also printed and added two 1x1 containers - [one for a Pokit multimeter](https://www.printables.com/model/340502-pokit-compact-multimeter-gridfinity-1x1), and [one for some soldering iron tips](https://www.printables.com/model/421641-gridfinity-soldering-iron-tip-holder). Then, I printed a 2x1 [TS100 soldering iron + brass wool holder](https://www.printables.com/model/268093-gridfinity-soldering-iron-brass-wool-holder) (this is where the final bearing went, from the original Prusa filament spool holder), a 2x1 [solder spool holder](https://www.printables.com/model/430301-gridfinity-solder-holder), a 1x1 [TS100 soldering iron holder](https://www.printables.com/model/523926-gridfinity-holder-for-ts100pinecil), a 1x1 [solder remover/sucker holder](https://www.printables.com/model/534222-gridfinity-solder-sucker-holder-x2), and an [empty 2x1 container](https://www.printables.com/model/459621-gridfinity-parametric-container-fusion360) for spares. Finally, this is what the soldering station looks like:

%[https://twitter.com/kamathsblog/status/1683196617355059200] 

## Next Up

Next up, I want to get back to the omniwheel robot. Now, I have managed to set up the on-board RPi Zero 2W with [ROS 2 Humble](https://docs.ros.org/en/humble/index.html), and all the necessary teleop twist packages to [generate twist commands using a Stadia controller.](https://adityakamath.hashnode.dev/teleop-with-game-controllers#heading-stadia-controller) Next, I want to write the [hardware interface for the serial bus servo motors](https://adityakamath.hashnode.dev/driving-serial-servo-motors). I want to first move the motors directly using the joystick. Then, I will work on the [ros2\_control](https://control.ros.org/master/index.html) controller for the 3-wheeled system. Meanwhile, I love how the colors of the robot match with the Stadia controller and other things in the background...

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1690140440331/95b0cee0-572c-4b87-bfe1-4c4a341b5b8c.jpeg align="center")