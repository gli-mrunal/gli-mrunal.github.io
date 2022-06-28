---
title: Using rosserial_arduino with stm32f303re in Visual Studio Code with PlatformIO
author: Mrunal
date: 2021-03-14 
categories: [ros, stm32]
tags: [rosserial_arduino, stm32]
math: true
mermaid: true

---

## Setup 

![Setup](\Images\rosserial_arduino\set_up.PNG)

# Setup for running chatter rosserial_arduino code on stm32f303re in Visual Studio Code with PlatformIO


## step 1
Install `PlatformIO IDE` and `C/C++ IntelliSense, debugging, and code browsing` extensions in `Visual Studio Code`.

Create stm32f303re project in platformIO named as `303_test_arduino`. Note that there are no libraries in the `lib` folder of the project now.


### Installing on the ROS workstation
There two option of how to install realted libraries
### option 1 (Recommended):
You can instal rosserial for Arduino by running

```sh
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-indigo-rosserial
rosrun rosserial_python
```


## step 2
Then run roscore in one terminal 
```sh
rosifyme9@rosifyme9:~/Documents/PlatformIO/Projects/303_test_arduino/lib$ roscore

```

This downloads the ros_lib library in PlatformIO projects libraries folder 

```sh
rosifyme9@rosifyme9:~$ cd Documents
rosifyme9@rosifyme9:~/Documents$ ls
Inverted-Pendulum-STM32-master  PlatformIO
rosifyme9@rosifyme9:~/Documents$ cd PlatformIO/
rosifyme9@rosifyme9:~/Documents/PlatformIO$ ls
Projects
rosifyme9@rosifyme9:~/Documents/PlatformIO$ cd Projects/
rosifyme9@rosifyme9:~/Documents/PlatformIO/Projects$ ls
303_test_arduino  ROS_STM32_Project
rosifyme9@rosifyme9:~/Documents/PlatformIO/Projects$ cd 303_test_arduino/
rosifyme9@rosifyme9:~/Documents/PlatformIO/Projects/303_test_arduino$ ls
include  lib  platformio.ini  src  test
rosifyme9@rosifyme9:~/Documents/PlatformIO/Projects/303_test_arduino$ cd lib
rosifyme9@rosifyme9:~/Documents/PlatformIO/Projects/303_test_arduino/lib$ ls
README
rosifyme9@rosifyme9:~/Documents/PlatformIO/Projects/303_test_arduino/lib$ rm -rf ros_lib
rosifyme9@rosifyme9:~/Documents/PlatformIO/Projects/303_test_arduino/lib$ ls
README
rosifyme9@rosifyme9:~/Documents/PlatformIO/Projects/303_test_arduino/lib$ rosrun rosserial_arduino make_libraries.py .
```


You get the following output. 


```sh
Exporting to ./ros_lib
Exporting actionlib

  Messages:
    TestRequestActionResult,TwoIntsFeedback,TestRequestAction,TestRequestGoal,TestActionGoal,TestRequestActionGoal,TestGoal,TwoIntsActionResult,TestRequestResult,TwoIntsAction,TestActionFeedback,TestActionResult,TestFeedback,TestAction,TwoIntsActionFeedback,TwoIntsActionGoal,TestRequestActionFeedback,TestRequestFeedback,TestResult,TwoIntsGoal,TwoIntsResult,

Exporting actionlib_msgs

  Messages:
    GoalStatusArray,GoalStatus,GoalID,

Exporting actionlib_tutorials

  Messages:
    AveragingActionResult,AveragingResult,FibonacciFeedback,FibonacciGoal,FibonacciActionGoal,FibonacciActionResult,AveragingActionFeedback,FibonacciAction,AveragingFeedback,FibonacciResult,AveragingAction,FibonacciActionFeedback,AveragingActionGoal,AveragingGoal,

Exporting bond

  Messages:
    Constants,Status,

Exporting control_msgs

  Messages:
    FollowJointTrajectoryGoal,JointTrajectoryFeedback,JointControllerState,PointHeadAction,FollowJointTrajectoryActionResult,FollowJointTrajectoryActionGoal,JointJog,PointHeadFeedback,FollowJointTrajectoryActionFeedback,SingleJointPositionActionFeedback,SingleJointPositionAction,JointTrajectoryAction,GripperCommandActionFeedback,GripperCommandFeedback,PointHeadGoal,JointTrajectoryActionResult,GripperCommandResult,GripperCommandActionGoal,SingleJointPositionGoal,JointTrajectoryActionFeedback,PointHeadActionGoal,PointHeadResult,JointTrajectoryGoal,SingleJointPositionResult,JointTrajectoryActionGoal,SingleJointPositionActionGoal,PidState,SingleJointPositionActionResult,PointHeadActionResult,GripperCommandAction,GripperCommand,FollowJointTrajectoryResult,SingleJointPositionFeedback,JointTrajectoryControllerState,PointHeadActionFeedback,GripperCommandActionResult,JointTolerance,JointTrajectoryResult,FollowJointTrajectoryFeedback,GripperCommandGoal,FollowJointTrajectoryAction,

  Services:
    QueryCalibrationState,QueryTrajectoryState,

Exporting control_toolbox

  Services:
    SetPidGains,

Exporting controller_manager_msgs

  Messages:
    ControllerState,ControllersStatistics,HardwareInterfaceResources,ControllerStatistics,

  Services:
    LoadController,SwitchController,ListControllerTypes,ReloadControllerLibraries,ListControllers,UnloadController,

Exporting diagnostic_msgs

  Messages:
    DiagnosticArray,DiagnosticStatus,KeyValue,

  Services:
    AddDiagnostics,SelfTest,

Exporting dynamic_reconfigure

  Messages:
    IntParameter,DoubleParameter,ConfigDescription,SensorLevels,Config,BoolParameter,GroupState,StrParameter,Group,ParamDescription,

  Services:
    Reconfigure,

Exporting gazebo_msgs

  Messages:
    ODEPhysics,ContactState,ODEJointProperties,ModelState,ContactsState,ModelStates,LinkStates,WorldState,LinkState,

  Services:
    SetLightProperties,GetLinkProperties,GetJointProperties,ApplyBodyWrench,BodyRequest,JointRequest,GetWorldProperties,SpawnModel,SetLinkProperties,SetModelState,SetLinkState,GetModelState,DeleteModel,SetJointProperties,GetLinkState,SetJointTrajectory,SetPhysicsProperties,ApplyJointEffort,DeleteLight,GetPhysicsProperties,GetModelProperties,SetModelConfiguration,GetLightProperties,

Exporting geometry_msgs

  Messages:
    Point32,Polygon,InertiaStamped,Wrench,Point,Pose2D,TwistStamped,PoseWithCovariance,PoseWithCovarianceStamped,WrenchStamped,Inertia,QuaternionStamped,AccelWithCovarianceStamped,Quaternion,Vector3,PointStamped,TwistWithCovarianceStamped,Pose,AccelWithCovariance,Vector3Stamped,Transform,Accel,TwistWithCovariance,PolygonStamped,TransformStamped,PoseArray,AccelStamped,Twist,PoseStamped,

Exporting laser_assembler

  Services:
    AssembleScans2,AssembleScans,

Exporting map_msgs

  Messages:
    PointCloud2Update,ProjectedMap,OccupancyGridUpdate,ProjectedMapInfo,

  Services:
    GetPointMap,GetPointMapROI,GetMapROI,SetMapProjections,ProjectedMapsInfo,SaveMap,

Exporting nav_msgs

  Messages:
    GetMapActionResult,GetMapAction,GetMapActionGoal,OccupancyGrid,GetMapResult,GetMapActionFeedback,Path,GetMapFeedback,GetMapGoal,GridCells,Odometry,MapMetaData,

  Services:
    SetMap,GetMap,LoadMap,GetPlan,

Exporting nodelet

  Services:
    NodeletLoad,NodeletList,NodeletUnload,

Exporting pcl_msgs

  Messages:
    PointIndices,Vertices,PolygonMesh,ModelCoefficients,

  Services:
    UpdateFilename,

Exporting polled_camera

  Services:
    GetPolledImage,

Exporting ros_essentials_cpp

  Messages:
    IoTSensor,

  Services:
    AddTwoInts,

Exporting roscpp

  Messages:
    Logger,

  Services:
    GetLoggers,Empty,SetLoggerLevel,

Exporting roscpp_tutorials

  Services:
    TwoInts,

Exporting rosgraph_msgs

  Messages:
    TopicStatistics,Clock,Log,

Exporting rospy_tutorials

  Messages:
    HeaderString,Floats,

  Services:
    AddTwoInts,BadTwoInts,

Exporting rosserial_arduino

  Messages:
    Adc,

  Services:
    Test,

Exporting rosserial_mbed

  Messages:
    Adc,

  Services:
    Test,

Exporting rosserial_msgs

  Messages:
    Log,TopicInfo,

  Services:
    RequestParam,

Exporting rviz

  Services:
    SendFilePath,

Exporting sensor_msgs

  Messages:
    PointCloud,JoyFeedback,RegionOfInterest,Illuminance,PointField,MultiEchoLaserScan,TimeReference,Joy,ChannelFloat32,CameraInfo,JoyFeedbackArray,Imu,RelativeHumidity,BatteryState,MultiDOFJointState,PointCloud2,Image,Range,LaserEcho,NavSatStatus,CompressedImage,MagneticField,LaserScan,FluidPressure,NavSatFix,JointState,Temperature,

  Services:
    SetCameraInfo,

Exporting shape_msgs

  Messages:
    SolidPrimitive,MeshTriangle,Plane,Mesh,

Exporting smach_msgs

  Messages:
    SmachContainerInitialStatusCmd,SmachContainerStatus,SmachContainerStructure,

Exporting std_msgs

  Messages:
    UInt16,Int64,UInt16MultiArray,UInt8,ColorRGBA,Float64MultiArray,Int16MultiArray,Header,ByteMultiArray,Bool,Float32MultiArray,Empty,UInt32,Int32MultiArray,Int16,MultiArrayDimension,UInt32MultiArray,Duration,Float32,String,Time,UInt8MultiArray,UInt64,Char,UInt64MultiArray,Int8MultiArray,Int64MultiArray,Float64,Int8,MultiArrayLayout,Byte,Int32,

Exporting std_srvs

  Services:
    Trigger,Empty,SetBool,

Exporting stereo_msgs

  Messages:
    DisparityImage,

Exporting tf

  Messages:
    tfMessage,

  Services:
    FrameGraph,

Exporting tf2_msgs

  Messages:
    LookupTransformAction,LookupTransformActionResult,LookupTransformActionFeedback,LookupTransformActionGoal,LookupTransformFeedback,LookupTransformResult,TF2Error,LookupTransformGoal,TFMessage,

  Services:
    FrameGraph,

Exporting theora_image_transport

  Messages:
    Packet,

Exporting topic_tools

  Services:
    MuxAdd,MuxDelete,MuxSelect,MuxList,DemuxList,DemuxDelete,DemuxSelect,DemuxAdd,

Exporting trajectory_msgs

  Messages:
    JointTrajectoryPoint,MultiDOFJointTrajectoryPoint,JointTrajectory,MultiDOFJointTrajectory,

Exporting turtle_actionlib

  Messages:
    Velocity,ShapeActionResult,ShapeGoal,ShapeActionFeedback,ShapeAction,ShapeActionGoal,ShapeFeedback,ShapeResult,

Exporting turtlesim

  Messages:
    Color,Pose,

  Services:
    TeleportRelative,TeleportAbsolute,Kill,Spawn,SetPen,

Exporting visualization_msgs

  Messages:
    InteractiveMarker,InteractiveMarkerUpdate,InteractiveMarkerPose,InteractiveMarkerControl,InteractiveMarkerFeedback,ImageMarker,InteractiveMarkerInit,MarkerArray,MenuEntry,Marker,


```

## step 3

Now copy paste the chatter rosserial_arduino program from the [http://wiki.ros.org/rosserial_arduino link ](http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World) into `src` folder `main.cpp`.

Copy paste the following code in src direcory of `303_test_arduino` project.

```c++
#include <Arduino.h> // This is important to run arduino code in Visual Studio Code PlatformIO

/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

// Use the following line if you have a Leonardo or MKR1000
//#define USE_USBCON

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
```

## step 4
Then press `PlatformIO build` and `PlatformIO Upload` bottons at the bottom of Visual Studio code to upload the code in stm32f303re board.

## step 5
Then run the rosserial_arduino program in terminal as follows:

`[termianl 1]`
> roscore

`[termianl 2]`

```sh
rosifyme9@rosifyme9:~/Documents/PlatformIO/Projects/303_test_arduino/lib$ rosrun rosserial_python serial_node.py /dev/ttyACM0
[INFO] [1619899134.042238]: ROS Serial Python Node
[INFO] [1619899134.057081]: Connecting to /dev/ttyACM0 at 57600 baud
[INFO] [1619899136.229088]: Requesting topics...
[INFO] [1619899136.616917]: Note: publish buffer size is 512 bytes
[INFO] [1619899136.619600]: Setup publisher on chatter [std_msgs/String]
```

`[termianl 3]`

```sh
rosifyme9@rosifyme9:~/Documents/PlatformIO/Projects/303_test_arduino/lib$ rostopic list
/chatter
/diagnostics
/rosout
/rosout_agg

rosifyme9@rosifyme9:~/Documents/PlatformIO/Projects/303_test_arduino/lib$ rostopic echo chatter
data: "hello world!"
---
data: "hello world!"
---
data: "hello world!"
---
data: "hello world!"
---
data: "hello world!"
---
data: "hello world!"
---
data: "hello world!"
---
data: "hello world!"
---
data: "hello world!"
---



rosifyme9@rosifyme9:~/Documents/PlatformIO/Projects/303_test_arduino/lib$ rostopic info /chatter
Type: std_msgs/String

Publishers: 
 * /serial_node (http://rosifyme9:35759/)

Subscribers: None


```