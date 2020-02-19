# tank_lidar

## Notes
> ### Creating a catkin Package
>> 1. cd ~/catkin_ws/src
>> 2. catkin_create_pkg tank_lidar roscpp rospy std_msgs message_generation


> ### Building a catkin workspace and sourcing the setup file
>> 1. cd ~/catkin_ws
>> 2. catkin_make
>> 3. . ~/catkin_ws/devel/setup.bash

## Clone and build the package
> ### tank_camera
>> 1. cd ~/catkin_ws/src
>> 2. git clone https://github.com/KhairulIzwan/tank_lidar.git
>> 3. cd ~/catkin_ws
>> 4. catkin_make
>> 5. . ~/catkin_ws/devel/setup.bash

## Required Package
> ### rplidar
>> 1. cd ~/catkin_ws/src
>> 2. git clone https://github.com/Slamtec/rplidar_ros.git
>> 3. cd ~/catkin_ws
>> 4. catkin_make
>> 5. . ~/catkin_ws/devel/setup.bash
>>
>>> #### How to run rplidar ros package ref:https://github.com/robopeak/rplidar_ros/wiki
>>>> Check the authority of rplidar's serial-port :
>>>>> ls -l /dev |grep ttyUSB
>>>> Add the authority of write: (such as /dev/ttyUSB0)
>>>>> sudo chmod 666 /dev/ttyUSB0


