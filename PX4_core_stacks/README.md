# px4_core_stacks
Core software for a PX4 drone containing, for example, forked mavros and mavlink for PX4 in one place. They should be very stable and well-tested for pixhawk-based drone
## Known issues (for new machine setup)

1. Could NOT find geographic_msgs

sudo apt-get install ros-melodic-geographic-msgs

2. Cannot find module named "future"

sudo apt install python-pip
pip install future

3. Could NOT find GeographicLib (missing: GeographicLib_LIBRARIES GeographicLib_INCLUDE_DIRS)

sudo apt-get install libgeographic-dev ros-melodic-geographic-msgs

4. NTP Server

sudo apt-get install ntp ntpdate
sudo gedit /etc/ntp.conf

then change from line 21 or so:

#pool 0.ubuntu.pool.ntp.org iburst
#pool 1.ubuntu.pool.ntp.org iburst
#pool 2.ubuntu.pool.ntp.org iburst
#pool 3.ubuntu.pool.ntp.org iburst
server 0.dk.pool.ntp.org
server 1.dk.pool.ntp.org
server 2.dk.pool.ntp.org
server 3.dk.pool.ntp.org

5. Troubles with Realsense
a) Project 'cv_bridge' specifies '/usr/include/opencv'
change the file:

/opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake
change this line:

set(_include_dirs "include;/usr/include;/usr/include/opencv")
to

set(_include_dirs "include;/usr/include;/usr/include/opencv4")

b) missing ddynamic_reconfigure
apt-get update apt-get install ros-kinetic-ddynamic-reconfigure

6. Fix Fan not running problem with TX2 + Auvidia carrier board: [link](https://jade.wtf/tech-notes/j121-fan/)

7. Fix problem of mavros: UAS: GeographicLib exception: File not readable /usr/share/GeographicLib/geoids/egm96-5.pgm

Manually copy (with root privilege) the contents of GeographicLib.zip to a folder (created with root privilege): /usr/share/GeographicLib

8. FCU: DeviceError:serial:open: Permission denied or No such file or directory

Add your user to group dialout and then reboot: 

sudo adduser $USER dialout