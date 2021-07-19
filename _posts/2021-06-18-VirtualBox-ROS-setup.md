---
title: VirtualBox for ROS Noetic - get up and running!
author: Mrunal
date: 2021-06-18 
categories: [ros]
tags: [ros]
math: true
mermaid: true
---

##  Setup Virtual Box

 ![image](\Images\Virtual_Box\p1.PNG)

 ![image](\Images\Virtual_Box\p2.PNG)

 ![image](\Images\Virtual_Box\p3.PNG)

 ![image](\Images\Virtual_Box\p4.PNG)
 
 Go to Device -> Add guest additions

 ![image](\Images\Virtual_Box\p5.PNG)

 reboot Virtual Machnine after that.

##  Setup ROS Noetic 

[ros-noetic install link](http://wiki.ros.org/noetic/Installation/Ubuntu)

```sh
rosifyme9@rosifyme9:~$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

rosifyme9@rosifyme9:~$ sudo apt  install curl 

rosifyme9@rosifyme9:~$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

rosifyme9@rosifyme9:~$ sudo apt update

rosifyme9@rosifyme9:~$ sudo apt install ros-noetic-desktop-full

rosifyme9@rosifyme9:~$ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

rosifyme9@rosifyme9:~$ gedit .bashrc

rosifyme9@rosifyme9:~$ source ~/.bashrc

rosifyme9@rosifyme9:~$ roscore

```

Verify ROS noetic installation by typing `roscore` into the terminal.

##  Create your catkin_ws with ROS Noetic

Follow [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) to set up your own catkin environment.

```sh
rosifyme9@rosifyme9:~$ mkdir -p ~/catkin_ws/src
rosifyme9@rosifyme9:~$ cd ~/catkin_ws/
rosifyme9@rosifyme9:~/catkin_ws$ catkin_make

```
The `catkin_make` command takes some time run. Before `catkin_make` only `src` folder is present inside `catkin_ws`. After `catkin_make` command is run, the following contents are added to `catkin_ws` workspace.

```
rosifyme9@rosifyme9:~/catkin_ws$ ls

build   devel   src

```


To activate your newly created catkin_ws workspace, you need to source the `setup.bash file` that is located in the `devel folder` of your catkin_ws workspace. And to do so you need to add the following command in the `.bashrc file` accessed by

```sh
 rosifyme9@rosifyme9:~$ gedit .bashrc

 ```

command added to `.bashrc` file:

```
source /home/rosifyme9/catkin_ws/devel/setup.bash
```

You save and close the `.bashrc file` and then type the following command in the terminal to source again the `.bashrc` for your `catkin_ws` workspace to be activated as the default repository.

```sh
rosifyme9@rosifyme9:~$ source .bashrc
```

To verify that the `catkin_ws` is activated as the default repository, type the following command into the terminal:

```sh
rosifyme9@rosifyme9:~$ roscd
rosifyme9@rosifyme9:~/catkin_ws/devel$ ls
rosifyme9@rosifyme9:~/catkin_ws/devel$ clear
rosifyme9@rosifyme9:~/catkin_ws/devel$ cd 
```

## Git Clone 

Now, you can put the sample code of any ROS github repository into your `catkin_ws` workspace.

```sh

rosifyme9@rosifyme9:~$ cd catkin_ws/src

rosifyme9@rosifyme9:~/catkin_ws/src$ sudo apt install git

rosifyme9@rosifyme9:~/catkin_ws/src$ git clone -b ros-noetic https://github.com/aniskoubaa/ros_essentials_cpp.git

```

Now `ros_essentials_cpp` package is inside the `src` folder.

Then go to `catkin_ws` workpace directory and compile the whole workpace with `catkin_make`.

```sh
rosifyme9@rosifyme9:~/catkin_ws$ catkin_make
```

## Test ROS C++ program

[`Terminal 1`]

```sh
rosifyme9@rosifyme9:~/catkin_ws$ roscore
```

[`Terminal 2`]

```sh
rosifyme9@rosifyme9:~/catkin_ws$ rosrun ros_essentials_cpp talker_node

[ INFO] [1623981343.757482340]: [Talker] I published Hello World 0

[ INFO] [1623981345.758473480]: [Talker] I published Hello World 1

[ INFO] [1623981347.770005580]: [Talker] I published Hello World 2

[ INFO] [1623981349.758124693]: [Talker] I published Hello World 3

[ INFO] [1623981351.758149533]: [Talker] I published Hello World 4

```
[`Terminal 3`]

```sh
rosifyme9@rosifyme9:~/catkin_ws$ rosrun ros_essentials_cpp listener_node

[ INFO] [1623981500.296506095]: [Listener] I heard: [Hello World 1]

[ INFO] [1623981502.297265847]: [Listener] I heard: [Hello World 2]

[ INFO] [1623981504.298286422]: [Listener] I heard: [Hello World 3]

[ INFO] [1623981506.302636596]: [Listener] I heard: [Hello World 4]

[ INFO] [1623981508.296446162]: [Listener] I heard: [Hello World 5]

```
## Test ROS Python3 program

```sh
rosifyme9@rosifyme9:~$ sudo apt install python-is-python3

rosifyme9@rosifyme9:~$ python
Python 3.8.5 (default, May 27 2021, 13:30:53) 
[GCC 9.3.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> exit()

```

[`Terminal 1`]

```sh
rosifyme9@rosifyme9:~/catkin_ws$ roscore
```

[`Terminl 2`]

```
rosifyme9@rosifyme9:~$ rosrun ros_essentials_cpp talker.py
[INFO] [1623981890.969294]: hello world 0
[INFO] [1623981891.970715]: hello world 1
[INFO] [1623981892.970506]: hello world 2
[INFO] [1623981893.970936]: hello world 3

```


[`Terminal 3`]

```sh
rosifyme9@rosifyme9:~/catkin_ws$ rosrun ros_essentials_cpp listener.py

[INFO] [1623981907.974698]: /listener_35156_1623981907865I heard hello world 17
[INFO] [1623981908.976414]: /listener_35156_1623981907865I heard hello world 18
[INFO] [1623981909.975961]: /listener_35156_1623981907865I heard hello world 19
[INFO] [1623981910.994349]: /listener_35156_1623981907865I heard hello world 20
[INFO] [1623981911.983249]: /listener_35156_1623981907865I heard hello world 21
```
