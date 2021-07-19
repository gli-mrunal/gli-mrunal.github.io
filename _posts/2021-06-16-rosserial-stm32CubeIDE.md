---
title: Rosserial stm32cube IDE
author: Mrunal
date: 2021-06-09 
categories: [ros, stm32]
tags: [ros,stm32CubeIDE]
math: true
mermaid: true
---

## Setup
Inside Ubuntu VirtualBox, 
```sh
> cd catkin_ws/
> ls
> cd src
catkin_ws/src > git clone https://github.com/yoneken/rosserial_stm32.git
catkin_ws/src > cd ..
catkin_ws > catkin_make
>
```

[git clone rosserial_stm32](https://github.com/yoneken/rosserial_stm32)

## Generate code part (Inc folder)

```sh
catkin_ws > cs
catkin_ws/ src >  ls
catkin_ws/ src >  cd rosserial_stm32/
catkin_ws/ src / rosserial_stm32 > ls
catkin_ws/ src / rosserial_stm32 > cd src
catkin_ws/ src / rosserial_stm32 / src > ls
ros_lib    rosserial_stm32

catkin_ws/ src / rosserial_stm32 / src > cd rosserial_stm32

catkin_ws/ src / rosserial_stm32 / src / rosserial_stm32 > ls

make_libraries.py
```

```sh
catkin_ws/ src / rosserial_stm32 / src / rosserial_stm32 > mkdir Inc

catkin_ws/ src / rosserial_stm32 / src / rosserial_stm32 > ls

Inc   make_libraries.py

catkin_ws/ src / rosserial_stm32 / src / rosserial_stm32 > rosrun rosserial_stm32 make_libraries.py ~/catkin_ws/src/ rosserial_stm32/src/rosserial_stm32

```
## Part - 2 Stm32CubeIDE
 Send this `Inc` folder with all the repositories to yourself through email/ usb. 
 Rename this folder as 'ROS' so that the Inc folders in stm32CubeIDE does not get confused with this newly generated Inc folder using ROS. 

Place this folder inside the stm32CubeIDE project : ` "Your Project name" -> Core -> Inc -> ROS`
This ROS folder is copied inside the stm32CubeIDE. If you don't from chatter project Inc. 
Copying the ROS in the files folder of Windows directory does not work.

After the header files and source code file mainpp.h and mainpp.cpp are added along with necessary changes in main.c and hardware.h then Debug the program as ros_test1.elf. 

Connect the stm32f303re board to the laptop. 
Right click the project and click `Debug as` -> configurations -> debug (on the dialog box)

The debug configuration is as follows: 

the debug 

![image](\Images\ros_stm32cube\debug_configuration.PNG)



- [Part 1](https://www.youtube.com/watch?v=MZ3hHldJqO4)

- [Part 2](https://www.youtube.com/watch?v=7Nwue0bPGnU)

then switch to Debug mode. 
Press >| on the right had side 
Then press pause button beside it. 
If it does not show following then press the >| nad pause button again till it shows as same as the image below.

![image](\Images\ros_stm32cube\debug_1.PNG)

## Part 3
```
 git clone 

 ```
From the [rosserial_stm2 github link](https://github.com/seo2730/rosserial_stm32) go to ros-drivers rosserial github link.

 > [git clone ros-drivers/rosserial](https://github.com/ros-drivers/rosserial)

```
rosifyme9@rosifyme9:~/Workspaces/catkin_rosserial_example/src$ git clone https://github.com/ros-drivers/rosserial.git

rosifyme9@rosifyme9:~/Workspaces/catkin_rosserial_example/src$  cd ~
rosifyme9@rosifyme9:~$ catkin_make
```

[`Terminal 1`]
```
rosifyme9@rosifyme9:~/Workspaces/catkin_rosserial_example$  roscore
```
[`Terminal 2`]
```
rosifyme9@rosifyme9:~/Workspaces/catkin_rosserial_example$ source /opt/ros/melodic/setup.bash


rosifyme9@rosifyme9:~/Workspaces/catkin_rosserial_example$ source ~/Workspaces/catkin_rosserial_example/devel/setup.bash


rosifyme9@rosifyme9:~/Workspaces/catkin_rosserial_example$ rosrun rosserial_python serial_node.py /dev/ttyACM0

```

> rosrun rosserial_server serial_node _port:=/dev/ttyYourBoard _baud:=YourBaudRate

[`Terminal 3`]

```
rosifyme9@rosifyme9:~/Workspaces/catkin_rosserial_example$ rostopic list


rosifyme9@rosifyme9:~/Workspaces/catkin_rosserial_example$ rostopic echo /num

```

![image](\Images\ros_stm32cube\num.PNG)


