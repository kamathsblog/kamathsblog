---
title: "ROS 2 and VPNs"
datePublished: Wed Aug 16 2023 16:23:30 GMT+0000 (Coordinated Universal Time)
cuid: clldxzbqv00000al3du13309f
slug: ros-2-and-vpns
cover: https://cdn.hashnode.com/res/hashnode/image/stock/unsplash/PSpf_XgOM5w/upload/a22259ade8f7f043c52b8f04abd0f850.jpeg

---

This one is a bit of a side-quest - Originally, I intended to work on [ros2\_control](https://control.ros.org/master/index.html) for my 3-wheeled robot, but I got sidetracked by a new group robotics project. One of our challenges was to find a way to control/monitor multiple outdoor robots (on different wireless networks) from a single ground station. As a proof-of-concept, I first decided to try and set up my own VPN using [Wireguard](https://www.wireguard.com/). I soon realized that this wasn't so straightforward and decided to use a commercial VPN service instead.

## Husarnet

My first and only choice was [Husarnet](https://husarnet.com/), a ROS/ROS 2 focused VPN service, after meeting their developers at [ROSCon 2022](https://adityakamath.hashnode.dev/recap-roscon-2022-kyoto). I started with a setup involving two devices - my laptop with [WSL2](https://learn.microsoft.com/en-us/windows/wsl/install) and a Raspberry Pi 4, both running Ubuntu 22.04 and [ROS 2 Humble](https://docs.ros.org/en/humble/index.html). I did the following on both devices:

1. Install Husarnet using the recommended method in [this tutorial](https://husarnet.com/docs/platform-linux-install/).
    
2. Install the Husarnet daemon service by running `sudo husarnet daemon service-install` and enable it using `sudo systemctl enable husarnet.service`
    
3. Connect both devices to Husarnet using [the Husarnet dashboard](https://husarnet.com/docs/tutorial-linux-begin/)
    
4. Follow this tutorial for using [ROS 2 on Husarnet](https://husarnet.com/docs/tutorial-ros2/)
    

In the last tutorial, the recommended [Husarnet-DDS](https://husarnet.com/docs/tutorial-ros2/#option-1-using-husarnet-dds-recommended) did not work for me, I kept getting errors when running `husarnet-dds singleshot` , so I followed the [manual configuration](https://husarnet.com/docs/tutorial-ros2/#option-2-manual-dds-configuration) steps using [Fast DDS](https://fast-dds.docs.eprosima.com/en/latest/) with the [Simple discovery protocol](https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/simple.html) - the default [discovery mechanism](https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery.html) for finding devices on the network.

In the [XML config](https://husarnet.com/docs/tutorial-ros2#preparing-a-configuration), I used Husarnet dashboard hostnames instead of IPv6 addresses. One thing to note is that communication worked only when the hostname from the Husarnet dashboard matched the device hostname. My final setup looked like this:

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1691866573746/db3f1cf6-a45c-426b-9010-0a1cf2b0adbc.png align="center")

I tested both devices across different WiFi networks for a day. Everything worked as expected. I was able to run the [ldlidar node](https://github.com/linorobot/ldlidar) on one RPi connected to [a LD06 lidar](https://www.inno-maker.com/product/lidar-ld06/) and list/echo the topics from WSL2. I did not actively monitor the network, but the latency seemed minimal.

To visualize it, I also ran [foxglove\_bridge](https://github.com/foxglove/ros-foxglove-bridge/tree/main) on WSL2 to see the laser scan on [Foxglove Studio](https://console.foxglove.dev/my-org-117/dashboard) running on my Windows host. I had the RPi connected to my home network, and my laptop (with Windows and WSL2) connected to my phone's 5G hotspot. For this setup, the latency was reasonable - I was able to physically move the lidar and see the point cloud change in real-time.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1691863624754/c5762166-a2b3-4461-8689-0ecf2c21f67f.png align="center")

I tried it multiple times across a couple of days and I came across a few more issues:

1. On the RPi, the Husarnet daemon did not consistently start during boot, despite having the daemon service enabled
    
2. On both WSL2 and the RPi, the [Husarnet Client](https://github.com/husarnet/husarnet) does not immediately join the network (assuming the daemon started correctly)
    

To fix the second issue, I had to create a service to join the correct network during boot (using the [CLI tools)](https://husarnet.com/docs/cli-guide). I also had to make changes on WSL2 to do this at boot (I had to edit `/etc/wsl.conf` since services aren't supported).

I'm glad I posted about this on Twitter, because I was recommended (by multiple people) a different alternative - Tailscale.

%[https://twitter.com/macjshiggins/status/1687099420842901504] 

## Tailscale

[Tailscale](https://tailscale.com/) is another P2P VPN service based on the Wireguard protocol but unlike Husarnet, is not specifically designed for ROS/ROS 2. Once again, I started with 2 devices - my laptop and a RPi4.

First, I had to disable Husarnet by running `sudo husarnet daemon stop`. Next, I disabled the services and removed the Husarnet hostnames from `/etc/hosts`. I also disabled the [DDS configuration](https://husarnet.com/docs/tutorial-ros2#option-2-manual-dds-configuration) by removing the environment variable set in the Husarnet tutorial (`unset FASTRTPS_DEFAULT_PROFILES_FILE`). Finally, I installed Tailscale on both devices:

1. Install Tailscale using [this quickstart guide](https://tailscale.com/kb/1017/install/) on all my devices (RPis, WSL2 and Windows).
    
2. Configure Tailscale by running `sudo tailscale set --operator=$USER -accept-dns=true --accept-routes=true` and run Tailscale `tailscale up`
    
3. Define DDS by adding `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` to `~/.bashrc`
    

I then repeated the demo from earlier but I encountered an issue - I could only list the laser scan topic from devices on the same wifi network, but couldn't list them from devices connected to the hostspot. I tried connecting both devices to the same network, but it still didn't work with WSL2. When it worked with 2 RPis, I came to the conclusion that the issue was WSL2. I also found the [Tailscale troubleshooting guide](https://tailscale.com/kb/1023/troubleshooting/) and realized that some features don't work well on WSL.

### Simple Discovery

I was so wrong. The issue wasn't with WSL - I just hadn't configured the [Fast DDS discovery mechanism](https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery.html#discovery-mechanisms) yet. I first set up the Simple discovery protocol using an [XML config](https://gist.github.com/adityakamath/a6751504434b3424ebfe6bae3eec82fa) similar to Husarnet - the only difference being Tailscale uses UDPv4 by default instead of UDPv6. So, I modified the Husarnet XML config using [the Simple Discovery Settings document](https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/simple.html) as a reference and used Tailscale hostnames instead of IPv4 addresses.

I was able to change the hostname using Tailscale's admin console and used these changed names in the XML file, which worked, unlike Husarnet. My XML config can be seen below:

%[https://gist.github.com/adityakamath/a6751504434b3424ebfe6bae3eec82fa] 

I then set the environment variable by adding the following line to my `~/.bashrc` file: `export FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_config>/<config>.xml`.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1691866806914/a2577702-1dc5-4222-8e77-c816638d2d10.png align="center")

With this, the demo started working exactly as expected. Once again, visually, the demo was identical to Husarnet. However in this case, over multiple uses, I did not have a single issue during boot - and this was with a standard Tailscale setup, I did not have to do any extra configuration.

### Discovery Server

[The Discovery Server](https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery_server.html#discovery-server) is a discovery mechanism that uses a centralized client-server architecture instead of a distributed one. [This mechanism results in much lower network traffic compared to Simple discovery](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#compare-fast-dds-discovery-server-with-simple-discovery-protocol). Detailed background info can be found in [Fast DDS Discovery Server tutorial](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#using-fast-dds-discovery-server-as-discovery-protocol-community-contributed) in the ROS 2 documentation. For setting it up, I also recommend [RoboFoundry's Medium blog post](https://robofoundry.medium.com/how-to-setup-ros2-fast-dds-discovery-server-3843c3a4adec). After resetting environment variables from the previous setup, I did the following on two devices:

1. Followed RoboFoundry's blog post.
    
    1. Use the default super client XML config and set one device as a server. Here, I used the Tailscale hostnames instead of the IPv4 addresses.
        
    2. Restarted ROS 2 daemon - don't forget this!
        
    3. Set the `FASTRTPS_DEFAULT_PROFILES_FILE` environment variable to the XML config
        
2. ***Update: This step does not seem to be needed when the super client configuration is used, which allows access to all available discovery information. However, this is needed when setting up devices as servers or clients:*** Set the `ROS_DISCOVERY_SERVER` environment variable by adding to `~/.bashrc`: `export ROS_DISCOVERY_SERVER="<server tailscale IP address>:11811"` where 11811 is the default port. ***Note***\*:\* here, the Tailscale hostname does not work, it needs to be an IPv4 address - which can be found by running `tailscale status`. Within the double quotes, multiple servers can be set by separating the addresses with a semi-colon.
    
3. Run the server on the RPi using the command `fastdds discovery -i 0 -l <IPv4 address or hostname> -p <port number>`
    

This was my final XML config:

%[https://gist.github.com/adityakamath/6340080a8c111d17e9649285e1d2d508] 

I was able to successfully repeat the demo from earlier by running a server on the RPi. More use cases are explained in the "[Advanced use cases](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#advance-use-cases)" section of the tutorial linked above. My setup was simple and looked like this:

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1691877451793/a881e875-a965-4012-958e-6cad55ec36e3.png align="center")

This worked as expected, but since it was still one device communicating with another - there wasn't any visible performance improvement. I am sure as I scale up and add more nodes and devices, the benefit would be more evident.

### Next Steps with Tailscale

There are a few things I wanted to try but did not have time for it:

1. [Discovery Server with Husarne](https://husarnet.com/blog/ros2-dds-discovery-server)t. It works, but I've made up my mind with Tailscale now, so I won't be trying this anytime soon.
    
2. [Cyclone DDS](https://cyclonedds.io/docs/cyclonedds/0.9.1/index.html) - I'm curious how it compares with FastDDS. I will leave it for later once I have more devices and nodes running.
    

However, my immediate focus is on filtering topics within the network. I do not want to expose all topics from the devices to the ground station - only the ones necessary for visualization and teleoperation. I don't think Fast DDS explicitly provides a feature for this, but there seems to be a way:

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1692203352379/a52a906b-8104-40fa-a42d-a9d09c7237e0.jpeg align="center")

I came across this diagram in the ROS2 Discovery Server documentation's "[discovery partitions](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#discovery-partitions)" section, which looks promising. If I understand this correctly, I can set the `ROS_DISCOVERY_SERVER` only on certain terminals running some specific nodes from my robot. So only those topics and messages are available to the ground station. This will only work when devices are [configured as servers or clients](https://fast-dds.docs.eprosima.com/en/latest/fastdds/use_cases/wifi/discovery_server_use_case.html#discovery-server-redundancy-scenario-setup) - instead of super clients. I still need to try it out but looks like this will solve my problem (a topic for a future post).

## Conclusion

After spending a few days working with both Husarnet and Tailscale (I used the free tiers for both VPNs) - I noticed some differences:

1. Tailscale free tier allows 3 users and up to 100 devices. Husarnet allows only 1 user and 5 devices.
    
2. Tailscale allows users to have multiple accounts, and easily switch between them both via the admin console and via CLI. <s>This is a very convenient feature that Husarnet does not offer.</s> *Update: turns out Husarnet also offers this feature, see the next section.*
    
3. Tailscale provides developer tools and a lot more control over the network <s>unlike Husarnet.</s> *Update: looks like Husarnet is catching up, see the next section.*
    
4. Automatically connects to Tailscale during booth, and there are no services to deal with separately. I did not have any connection issues, unlike Husarnet.
    
5. Tailscale also has [an Android app](https://tailscale.com/download/android) from where I can connect to the network. From here, using [JuiceSSH](https://juicessh.com/), I was also able to ssh into devices on the network.
    

Besides the above, both networks are just as good and I did not encounter any network issues with either VPN during my experiments. Even setup and installation were easy for both VPNs, taking only a couple of minutes. But for now, and because of the above reasons, I prefer Tailscale. I will update this article with new observations as I keep working with Tailscale and add on more nodes and devices...

## Update:

Since I posted this article, I received a Twitter DM from Dominik at Husarnet (turns out he's the guy I talker to at ROSCon 2022) and had a nice conversation about some of the issues and shortcomings I mentioned above. Turns out I missed a few things:

1. **Switching between different user accounts:** This was one of the reasons why I chose Tailscale, but apparently, I completely missed the fact that Husarnet also offers this feature. Dominik shared a screenshot with example networks and how he switches between them with different user accounts using the Husarnet CLI.
    
    ![](https://cdn.hashnode.com/res/hashnode/image/upload/v1692292022482/50565bfe-c8c8-4907-b064-56ab88b49e86.jpeg align="center")
    
2. **More control over the network:** Tailscale offers Access Control List (ACL) functionalities, which Husarnet currently does not. However, they are working on a new dashboard with the ACL feature as well as the ability to manage networks via the CLI. Sounds very promising!
    
3. **Issues with Husarnet-DDS:** I was unable to run `husarnet-dds singleshot` , the final command to set up Husarnet-DDS when I first tried it. I did not save the error info, and hence we were unable to solve this issue in the chat. However, Dominik shared some common issues that other users also faced - Now, I haven't tried this, and I am not entirely sure if this will fix the issue for me, but here are the suggestions that were shared:
    
    1. An old version of Husarnet Client was installed - Husarnet-DDS talks with the client over the HTTP API which was introduced in Husarnet 2.0
        
    2. Users often forget to run `chmod +x` after downloading Husarnet-DDS (I think this is what is potentially wrong with my installation)
        
    3. The binary was downloaded for the wrong device architecture
        

One one of my earlier tweets, there were a few concerns about support from Husarnet - but after this, looks like they are quite proactive in solving user issues (on Twitter at least). Once I get some time, I plan on trying Husarnet out again, especially to see if I can fix the Husarnet-DDS installation, and if not, to at least get the error message and logs so that I can send them to the folks at Husarnet.

I was also recently reminded of the [eProsima Fast DDS Monitor](https://www.eprosima.com/index.php/products-all/eprosima-fast-dds-monitor), which took me back to [this talk](https://vimeo.com/showcase/9954564/video/767140681) from ROSCon 2022 in Kyoto. Maybe I will also use this opportunity to set up monitoring tools and quantitatively analyze both Husarnet and Tailscale networks. Looks like there will be another VPN-related post in the near future...