# cv_perception

This package is based on ROS-melodic with Ubuntu 18.04.

## 1. Usage

### 1.1 Basic usage


```
# Launch all nodes
roslaunch lfi_launch all.launch

# Launch camera nodes
roslaunch lfi_launch camera.launch

# Launch GPS node
roslaunch lfi_launch gps.launch

# Launch data recorder node
roslaunch data_collector dumpper.launch
```

Robot vision configuration file is at:
```
cv_perception/config/lawn_mover.json
```
**Please remember to set the camera name(`B601A/B601B/601C`) in above configure file to get the right calibration. The camera name can be found at the bottom of the camera.**

All the dataset recorded in Nanjing, China were using `B601C`, and other two cameras has shipped to US.
***

### 1.2 Change camera record resolution
- Edit the `camera_record_res` in `config/lawn_mover.json` 

Please note that our previous dataset was recorded with 3840 x 2160, and currently the camera record resolution has set to 1280 x 760. **So if you want to switch between replay and real time launch mode, please remember to modify this paramter.**

```
"camera_record_res" : [1280, 720]
```

***

### 1.3 Replay recorded dataset
```
# single play
rosbag play <dataset_name>

# loop mode
rosbag play -l <dataset_name>
```
***

### 1.4 Show panoramic image
```
# opencv window
rosrun image_wrapper display_image

# rostopic
rosrun image_wrapper publish_topic
```
***

### 1.5 Record data
Set the `<function_dump>` to `1` in `config/lawn_mover.json`, and `dump_path` to where you would like to store the data. You can also set which sensor to be recorded and its paramters such as resolution.

```
"dump_path" : "/data/log3/",

"camera_dump_raw":1,
"camera_dump_raw_width":1280,
"camera_dump_raw_height":760,

"camera_dump_pano": 1,
"camera_dump_pano_width":793,
"camera_dump_pano_height":367,

"imu_dump" : 1,
"gps_dump" : 1
```


## 2. Published Topics
| Topic Name                    | Msg Type               | Function                      | Comment |
|-------------------------------|------------------------|-------------------------------|---------|
| /IMU_data                     | sensor_msgs/Imu        | External IMU sensor data      |         |
| /PowerVoltage                 | std_msgs/Float32       | Battery Voltage               |         |
| /mobile_base/sensors/imu_data | sensor_msgs/Imu        | Stm32 onboard IMU sensor data |         |
| /odom                         | nav_msgs/Odometry      | Chassis Odometry data         |         |
| /ublox_gps/fix                | sensor_msgs/NavSatFix  | GPS location data             |         |
| /ublox_gps/navheading         | sensor_msgs/Imu        | GPS IMU data                  |         |
| /ublox_gps/navsat             | ublox_msgs/NavSAT      | GPS localization method       |         |
| /usb_cam/camera_info          | sensor_msgs/CameraInfo | Camera raw infomation         |         |
| /usb_cam/image_raw            | sensor_msgs/Image      | Camera raw image data         |         |
| /camera/wimage                | sensor_msgs/Image      | Camera unwrap image data      |         |

## 3. Nodes Introduction

### 2.1 image_wrapper
Provoding panoramic image unwrap function. 

- `display_image` will pop out an opencv style window with the unwrap image.
- `publish_topic` will establish a publisher topic at `/camera/wimage`.

Required configure JSON file:
```
cv_perception/config/<camera_name>.json
```
If you are viewing recorded data with 3840x2160, it's highly recommended to set the `"display_ratio"` to `5`, otherwise please do not change anything.

The default recording resolution will be 1280 x 720, and the unwrap image's resolution will be 793 x 367.

***
### 2.2 web_video_server
This node could start RTMP streaming and provide a web page to view both the original camera image or the unwrap panoramic image. This node was launched by `camera.launch`.

The default web address is `http://<device_ip>:8080`.
***

### 2.3 rtcm
This node could output the ntrip network RTK localization data to a specific serial port. The configure file is at
```
cv_perception/lfi_launch/gps.launch
```
You can replace the content of the `in` paramter with your own local ntrip services.

This node will also launch the ublox_gps moudle, which could provide the robot GPS infomation.
***

### 2.4 data_collector
This node could record image, IMU, and GPS data to your specific folder.
