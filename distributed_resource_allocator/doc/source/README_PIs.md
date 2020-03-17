# MOSAIC Raspberry PIs Setup

## Setting up the Raspberry PIs with Wifi Router

1. Log in into each pi using a monitor and connect to the wifi network.
2. Then in the network manager, got to 'Edit Connections...', select the
network provided by the router, click edit and the general tab check option
'All users may connect to this network'. Make sure that option 'Automatically
connect to this network when it is available' is also checked. That will allow
the system to connect to the wifi without having to login first.   

Access to the Raspberry PI through VM (parallels).

In Parallels go to Hardware-> Network and change the Source to Network Adapater (Bridged Network).
Then the VM will connect to the WiFi as a separate machine. Do not use other adapters
(cables/USB connected for ethernet) otherwise it will not connect.


## Enabling NTP to sync time

### In the raspberry Pi (NTP client)
1) edit file /etc/systemd/timesyncd.conf
$ sudo nano /etc/systemd/timesyncd.conf
Add the NTP provider's IP address: NTP=ubuntustation.local

2) Restart systemd-timesyncd.service
```
$ sudo systemctl restart systemd-timesyncd.service
```


3) We probably don't need this! MAYBE??? set ntp sync to true:
```
$ sudo timedatectl set-ntp true
```

### In the base station (NTP server, in port 123)

1) Disable systemd-timesyncd.service
```
$ sudo  systemctl disable systemd-timesyncd.service
$ sudo timedatectl set-ntp false
```

2) Install ntp
```
$ sudo apt-get install -y ntp
```

3) Setting up NTP server configuration

Set up the /etc/ntp.conf file to have the content below. Here we assume that the
network IP range is 10.0.0.0.

```
# /etc/ntp.conf, configuration for ntpd; see ntp.conf(5) for help

driftfile /var/lib/ntp/ntp.drift

# Enable this if you want statistics to be logged.
#statsdir /var/log/ntpstats/

statistics loopstats peerstats clockstats
filegen loopstats file loopstats type day enable
filegen peerstats file peerstats type day enable
filegen clockstats file clockstats type day enable

# Specify one or more NTP servers.

# Use servers from the NTP Pool Project. Approved by Ubuntu Technical Board
# on 2011-02-08 (LP: #104525). See http://www.pool.ntp.org/join.html for
# more information.
pool 0.ubuntu.pool.ntp.org iburst
pool 1.ubuntu.pool.ntp.org iburst
pool 2.ubuntu.pool.ntp.org iburst
pool 3.ubuntu.pool.ntp.org iburst

# Use Ubuntu's ntp server as a fallback.
pool ntp.ubuntu.com

# Access control configuration; see /usr/share/doc/ntp-doc/html/accopt.html for
# details.  The web page <http://support.ntp.org/bin/view/Support/AccessRestrictions>
# might also be helpful.
#
# Note that "restrict" applies to both servers and clients, so a configuration
# that might be intended to block requests from certain clients could also end
# up blocking replies from your own upstream servers.

# By default, exchange time with everybody, but don't allow configuration.
restrict -4 default kod notrap nomodify nopeer noquery limited
restrict -6 default kod notrap nomodify nopeer noquery limited

# Local users may interrogate the ntp server more closely.
restrict 127.0.0.1
restrict ::1

# Needed for adding pool entries
restrict source notrap nomodify noquery

# Clients from this (example!) subnet have unlimited access, but only if
# cryptographically authenticated.
#restrict 192.168.123.0 mask 255.255.255.0 notrust
restrict 10.0.0.0 mask 255.255.255.0 nomodify notrap

# If you want to provide time to your local subnet, change the next line.
# (Again, the address is an example only.)
broadcast 10.0.0.255
#broadcast ubuntustation.local
broadcast 224.0.1.1


# If you want to listen to time broadcasts on your local subnet, de-comment the
# next lines.  Please do this only if you trust everybody on the network!
#disable auth
#broadcastclient

#Changes recquired to use pps synchonisation as explained in documentation:
#http://www.ntp.org/ntpfaq/NTP-s-config-adv.htm#AEN3918

#server 127.127.8.1 mode 135 prefer    # Meinberg GPS167 with PPS
#fudge 127.127.8.1 time1 0.0042        # relative to PPS for my hardware

#server 127.127.22.1                   # ATOM(PPS)
#fudge 127.127.22.1 flag3 1            # enable PPS API

server 127.127.1.0 prefer
fudge 127.127.1.0 stratum 10
#server ubuntustation.local
```


3) Restart the server/base station

4) Check status:
```
$ timedatectl
```
We should see something like:
```
Local time: Fri 2019-02-08 15:08:55 PST
  Universal time: Fri 2019-02-08 23:08:55 UTC
        RTC time: Fri 2019-02-08 23:08:55
       Time zone: PST8PDT (PST, -0800)
 Network time on: no
NTP synchronized: yes
 RTC in local TZ: no
```

4) MAYBE??? set ntp sync to true
```
$ sudo timedatectl set-ntp true
```


If the server has to disconnect and reconnect to the local network, then
on the server side run the following to restart the ntp server
```
$ sudo /etc/init.d/ntp stop
$ sudo /etc/init.d/ntp start
```

## SSH into PIs

Connect to the same (wifi) network

If terminal prevents access (WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED)
We need to remove host hey first.
```
$ ssh-keygen -f "/Users/<username>/.ssh/known_hosts" -R 10.42.0.1
```

To ssh into the ubuntu vm running roscore, you might need to install openssh-server


## Ubiquity Robotics Environment Variables

The image provided by Ubiquity has some ROS env setup which can be found in
.bashrc
/etc/ubiquity/env.sh


## Multi Master FKIE
Install it through apt-get_transform
```
$ sudo apt-get install ros-kinetic-multimaster-fkie
```

In ~/.bashrc make sure we have the following:
```
source /opt/ros/kinetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash
source /etc/ubiquity/env.sh
export ROS_PARALLEL_JOBS=-j2 # Limit the number of compile threads due to memory
export ROS_WORKSPACE=/home/ubuntu/catkin_ws/
export HOSTNAME
```


In the above setup export HOSTNAME allows we get the name of the vehicle in
the launch file by using $(env HOSTNAME)

In /etc/ubiquity/env.sh you will see that ROS_HOSTNAME=$HOSTNAME.local
and ROS_MASTER_URI=$HOSTNAME:11311. So the base station computer (ubuntustation)
has also to set up these env variable this way.


## Multicast
Check if  multicast is enabled
```
$ cat /proc/sys/net/ipv4/icmp_echo_ignore_broadcasts
```

If 0, then it is enabled. If 1, it is not.

To temporary enable the multicast feature, execute the following command, however,
when the computer restarts, this configuration will be lost.
```
$ sudo sh -c "echo 0 >/proc/sys/net/ipv4/icmp_echo_ignore_broadcasts"
```

To permanently enable the multicast feature, edit the /etc/sysctl.conf file and add the following line, or uncomment it, if it already exists, and change its default value
```
net.ipv4.icmp_echo_ignore_broadcasts = 0
```

In order for the changes to take effect, execute the following command:
```
$ sudo service procps restart
```

To check which multicast groups are already defined in a computer, execute the following
command.
```
$ netstat -g
```

This command will report all the IP addresses enabled for multicast for each of the network
interfaces available, both for IPv4 and IPv6. The standard IP address for multicast is 224.0.0.1,
that should appear on the list provided by the last command, and it is the one we will use.
At this point, to check whether the multicast feature is working or not, execute the following
command, at any computer.

```
$ ping 224.0.0.1
```

If everything is configured properly, you should get a reply from each computer in the
common network at each iteration.


## ROS Setup

Copy the three ros packages:
- pose_publisher
- vehicle_launcher
- vehicle_network_manager

Change the initial position of the vehicle in the main.launch file of pose_publisher

Change the network ion_file argument in the main.launch file in vehicle_network_manager to

TODO: move this to ENV variable
"/home/ubuntu/rtd_dtn_mesh/9X_current_info.txt" where X is the rpi number

To start the vehicle launcher automatically at boot time,
please check how to set up an OS service in the topics below.


## OS Services (basics)
To show a service:
```
$ systemctl show <service>.service
```
To check if it's running (and where the file is located):
$ sudo systemctl status <service>.service
To find:
```
$ find / -name <service>.service
```

Where to find them, usually:
/etc/systemd/system/
/lib/systemd/system/
/usr/lib/systemd/system/


Instruction on how to create a service:
https://www.linode.com/docs/quick-answers/linux/start-service-at-boot/


## Existing OS Services in (Ubiquity) PI

By default there are a few services that run at startup:

### roscore.service
/etc/systemd/system/roscore.service
This one starts roscore. It can be disabled by:
$ sudo systemctl disable roscore.service
or stopped by:
```
$ sudo systemctl stop roscore.service
```

### magni-base.service
/etc/systemd/system/magni-base.service
This one runs a script (/usr/sbin/magni-base) starts a launch file (roslaunch magni_demos teleop.launch) for a default robot. They recommend disabling it
if not using their robot.
```
$ systemctl disable magni-base.service
```

### pifi.service
/lib/systemd/system/pifi.service
The only other non-standard running by default is pifi, their headless network management tool

If we must run code before roscore, you can add Before=roscore.service to the systemd service file for that code.


## Setting up OS Service to Start ROS Vehicle Launchers at Boot Time


### Create the Service File
Create a Unit file to define a systemd service in /lib/systemd/system/mams_pi.service with sudo permission, for example by:
```
$ sudo nano /lib/systemd/system/mams_pi.service
```

with the following content:

```
[Unit]
Description=ROS Launch start for MAMS packages
Requires=roscore.service
PartOf=roscore.service
After=NetworkManager.service time-sync.target roscore.service
[Service]
Type=simple
User=ubuntu
ExecStart=/bin/bash /home/ubuntu/catkin_ws/src/vehicle_launcher/scripts/start_mams_pi.sh

[Install]
WantedBy=multi-user.target
```

This defines a simple service that will launch the code from mams at boot time. It will
actually launch after roscore.service is active.
The critical part is the ExecStart directive, which specifies the command that will be run to start the service. In this case, the ros package vehicle_launcher should already come with
the script. Make sure the script is executable:
```
$ chmod +x /home/ubuntu/catkin_ws/src/vehicle_launcher/scripts/start_mams_pi.sh
```

In the script start_mams_pi.sh change the value of the variable VEHICLE_NAME to
the name of the pi node. For example, node_91 or node_95.

### Start and Enable the Service Permanently
Once you have a unit file, you are ready to test the service:
```
$ sudo systemctl start mams_pi
```

Check the status of the service:
```
$ sudo systemctl status mams_pi
```

Finally, use the enable command to ensure that the service starts whenever the system boots:
```
sudo systemctl enable mams_pi
```

Reboot the system and check if the service is activity_end_states:
```
$ sudo systemctl status mams_pi
```
You can also check if the ros topics are available as expected:
```
$ rostopic list
```
