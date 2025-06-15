Assignment 3 - FAQ 
=========================

[TOC]

This document needs to be read in conjunction with A1 specifications that details the topic and service names and message types used for each of the projects.

## What will occur at VIVA

We will execute your code, from the sole package you supply by completing a `ros2 run` for any nodes that you have developed, as detailed in your doxygen document. 

1) Topics and Services

We will check your nodes subscribes to specified topics, publishes to specified topics and provides the required services. This will be completed by running the ROS CLI `ros2 topic` and `ros2 service` commands as detailed in week 9.

2. Start and Abandon Mission

When your code is running, it should ONLY perform the mission when both of the conditions are met

- It receives goals on the the topic name stipulated in A3 specification 
- It has a service call made on the service name with the request data portion set to `true`. In the below `/THE_SERVICE_NAME`  is specific to each project, refer A3 specification for details
  - `ros2 service call /THE_SERVICE_NAME std_srvs/srv/SetBool '{ data: true }'`

Your code should immediately abandon the mission, stopping the audi and landing the quadcopter if the service call made on the service name with the request data portion set to `false`. Again in the below `/THE_SERVICE_NAME`  is specific to each project, refer A3 specification for details

- `ros2 service call /THE_SERVICE_NAME std_srvs/srv/SetBool '{ data: false }'`

This file is to assist students in preparing their Assignment 3 (A3) individual project and should be used in collection with `a3_skeleton` package and week 12 package `a3_support`, as well as content over week 10 and 11 . These packages are to be found in your github repositories [scratch](../../scratch) and [tutorial / week12](./) folders. Each of these packages contain a certain type of scaffolding of content. Unlike Assignment 1/2 where we interacted with your assignment via a library (and therefore supplied interface classes that you had to inherit from), in the final Assignment 3 we interact with your code at the level of topics/services. Topics and services allow us to affect your code behaviour at rum-time (at execution).  

In A3 having the correct topics/services forms part of marking criteria, and therefore we can not provide a package with all these laid out. Instead, it is your task to set them out depending on the project. In order for your package to compile/function you need to alter `package.xml` and `CMakeLists.txt` with the requirements (this has been introduced in `week10` and then reinforced in `quiz5` and `week11` material) and then modify your code with the topics/subscribers.

The specifics of topics/service and how your code should behave are specified on canvas in the Assignment specification. This document serves to indicate how you could develop your submission. Your A3 needs to be:

- one package
- two nodes minimum
- interact with topics and services as described in the specifications
- process laser/sonar data to undertake the mission 
- contain a library and ros bags for unit testing (the unit tests YOU develop)
- have a control loop running at guaranteed rate and does not have tight loop that blocks (prevents it from stopping)
- contains a switch via a ros parameter (only for D/HD)

Below contains a breakdown of what each package brings conceptually.

#### a3_skeleton

The support package contains an example of two nodes `foo` and `bar` which are compiled separately into executables `foo_node` and `bar_node`. They can also be run together using the `both_nodes` executable. The package.xml and the CMakeLists.txt facilitate building a library, linking it to ROS dependencies, assembling unit testing and distributing the ros bag for testing via `data` folder.

`Foo Node`

The `foo` node subscribes to topic `orange/laserscan` listening for `sensor_msgs::msg::LaserScan` from the ackerman platform. It also subscribes to `goal` topic listening to `geometry_msgs::msg::Pose`. Finally, it listens for a `std_srvs::srv::Trigger` type service call on `detect` service name, and it runs the server (it takes requests and responds).

Together with the service call we have 5 functions that are invoked at different times (two callbacks, the service, a thread and a timer).
A function runs tied to a timer (triggered every 500ms) and a thread of execution is tied to threadFunction (which has a sleep of 1s). 
All of the timer/callback/thread arrangement is achieved in constructor of the class.

We will use ROS INFO to indicate when a function is invoked (thread, timer, service) and use the `LaserProcessing `class from a number of functions. 
This is an example of mechanisms available to scafold your code. You have to decide what is best mechanism. 

Note, the all processing is done outside the node that subscribes to ROS (the node that contains the 'plumbing'. 
Please consider this when creating your own code.

**Bar Node**

The `bar` node is designed to periodically call a service (`/detect`) to check for detection in the laser via `foof` node.  The node therefore uses the client end of the service. A separate thread running at 1Hz manages functionality of publishing random poses and markers. The random poses 
are published to topic (`/goal`), which the `foo` node is listenining to. It visualize the poses using  markers published to a topic (`/visualization_marker`) under the namespace `sample1`. The markers are continuously  added to the array, each marker a cylinder shape like a puck with orange colour with lifespan of 20s.

**Library**

The requirement of Assessment 3 is to deliver code that has unit tests and a library (on the sensor data). The `a3_skeleton` contains a library (`LaserProcessing`) which is unit tested and has ros bags for the test. The library is also used inside the `Foo` class. 

The library has two public methods, it is thread safe as internally implements all necessary mutexes so the user can call the functions from a number of threads.  This library is tested via unit tests, where the single public function is tested against a ros bag that contains data.

The package.xml and the CMakeLists.txt facilitate building a library, linking it to ROS dependencies, assembling unit testing and distributing the ros bag for testing via `data` folder.  The `LaserProcessing` library is not a node, and does not connect to any topics/services. It can not be unit tested otherwise (we refer here to ROS2 Level 1 Unit Tests). In unit testing required, a ros bag is provided which has a particular scenario (as discussed in in  [tutorial / week12](./)). Just like in quiz5, you do not need to launch or run any nodes, nor run rviz, to undertake unit testing. 

To run your unit tests you need to build the package containing the unit test (using colcon build) and then run then with `colcon test --event-handlers console_cohesion+` . So in quiz5 our package was called `quiz5` and therefore the syntax to build and run unit tests was:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select quiz5
colcon test --event-handlers console_cohesion+ --packages-select quiz5
```

In the context of A3, you could consider `Sample` as your either class that does higher level tasks (mission). In A3 you can make your mission node work with laser data instead of controller node - given the task of analysing laser data (perception) informs the control (goals etc).

#### a3_support

Supplying goals is a essential task for this assessment (P/C) portion requires reasoning from goals supplied. This package is there to support supplying goals from a file. The  other concepts relevant to the A3 is the way parameters and remapping is achieved. Parameters allow us to pass information at the beginning of execution of the node (like and argv / arc in standard C++). Remapping allows us to publish/subscribe to a different topic to that in the code without recompiling.

**Do not need to integrate this code or node in your submission, do not submit it as your own code. It is there for you to test your submission**

Example GOALS for each project are supplied, the process to publish them so they are available to the ROS system and your A3, the use of ros parameter to load alternative files is the focus of week 12 activity outlined in [WEEK 12 TUTORIAL](./TUTORIAL.md).

In summary, the `goal_publisher` node loads goals from a file supplied and publishes them on two topics (`/goals` and `/visualization_marker`). 

Introducing [ros parameters](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html), this package allows to load different set of goals via command line (FYI: depending on project replace `ACKERMAN.TXT` with `QUADCOPTER.TXT`) as well as change the topic they are published to (let's say `/mission/goals`)	

```bash
ros2 run a3_support goals_publisher --ros-args --remap goals:=/mission/goals -p filename:=$HOME/ros2_ws/install/a3_support/share/a3_support/data/A3_TERRAIN.TXT
```
