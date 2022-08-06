# Drone_code
## Passwords and ssh info 
Vicon Computer pass
```bash
AU_parrot_2018
```



Connect to ASL_Vicon wi-fi password
```bash
1234567890
```


 Jetson TX2 drone
password
```bash
lea
```

```bash
ssh lea@192.168.0.59
```

```bash
./JetsonTX2_drone
```

 Upboard
password
```bash
1234
```
connect to asl_vicon wifi


```bash
ssh anoertoft@192.168.0.29
```
kaffe
ssh kaffe@192.168.0.24

```bash
ssh kaffe@192.168.0.24
```


etc folder:

## px4

px4 version
1.11.3

installation guide 
https://docs.px4.io/main/en/config/firmware.html

download the file px4_fmu-v5_default.px4 and upload from this https://github.com/PX4/PX4-Autopilot/releases/tag/v1.11.3
## Tips

- Drone can only disarm in manual control


##Tips
Cannot Switch to offboard mode:
- Problem with height estimate 
- problem with parameter
-Problem with EKF2 estimate
- mavros baudrate 


# Testing Checks
## A) Vicon System Check
- All markers are placed securely in their designated postion
- Infrared cameras are green and ready 
- The correct drone name is ticked **(IMPORTANT)** 
- Check that all markers are visible and non blinking
- Check when running the vicon feeder and mocap that they topic is available and that they match.
- Check that the Vicon is not paused


## B) Battery Check 
- Battery is charged (Voltage>11.7).
- Securely fastened to the drone.



## C) Cables Check
- No hanging cabled
- No cables near the propeller
- No exposed wires
- All legs are placed and secured

## D) R.C Check
- Vehicle arms and disarms
- Mode change is possible


## D) Running the code on the drone
- run the Jetson bash
- Check ping
- Check the Vicon feeder 



#


# Run MPC instructions

- open 4 tabs in terminator
- ssh to drone (lea)
```bash
ssh lea@192.168.0.59
```
- run TX2 startup script
```bash
 ./JetsonTX2_drone_start.sh
```
- run mavros 
```bash
roslaunch simple_mavros mavros.launch
```

- run vicon bridge vicon.launch
```bash
roslaunch vicon_bridge mocap_bridge_vicon.launch
```
- run mocap bridge
```bash
roslaunch mocap mocap_bridge_vicon.launch
```
- run nmpc
```bash
roslaunch nmpc_pc_learning nmpc_pc_learning_indoor.launch
```
- connect laptop to drone
```bash
export ROS_MASTER_URI=http://192.168.0.60:11311
export ROS_IP=192.168.0.72
```

```bash
roslaunch dji_m100_trajectory m100_trajectory_v2_indoor.launch 
```

- **on laptop:** provide trajectory 

```bash
run rqt_reconfigure rqt_reconfigure
```

- tick on circular trajectory



# TODO - 6/8
- Upload code on kakao - check
- Clean the repo from other folders - check
- run NMPC, also with VICON 
- run the NMPC with hovering from trajectory
