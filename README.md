# drone_code
## Jetson TX2 drone
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

## Upboard
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

## Run MPC instructions

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
geometric controller script
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
- *on laptop:* provide trajectory 

```bash
roslaunch dji_m100_trajectory m100_trajectory_v2_indoor.launch 
```

- tick on circular trajectory




