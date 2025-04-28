Week 8 - Quadcopter Support
=========================

The code needs  **pfms2ros** package (version 3.0.8 or higher), 

```bash
sudo apt update
sudo apt install pfms-ros-integration
```

While developing code you do not need to run the simulator. 

When testing code you need to run the simulator with the launch file for assignment 2.

```bash
ros2 launch pfms a2.launch.py
```
In order to visualise the entire map you can use optional `gui:=true` parameter. This uses another whole core on your computer.
```bash
ros2 launch pfms a2.launch.py gui:=true
```

Tutorial
===============================

We have three examples of using the `QUADCOPTER` platform, in moving it, getting `LaserScan` and `Sonar` data.  These are essential for the quizzes and the assessment tasks.

### Control

In the `command_quadcopter.cpp` we aim to send some control to the `Quadcopter` to move it about. To do so we firts must send the TAKEOFF command.
```
pfmsConnectorPtr->send(pfms::PlatformStatus::TAKEOFF);
```
In order to check that has had an effect we need to monito the `Odometry` and can abandon sending the command once the `Quadcopter` has lifted in height (`odo.position.z`) or we have sent the `TAKEOFF` a reasonable amount of times. 

Then we can send a command, I suggest to send a forward velocity of 0.1 m/s, to see how the platform behaves.

### Laser

In the `receive_laser.cpp` we aim to receive `pfms::sensor_msgs::LaserScan` messages from a laser mounted on the `Quadcopter`. If we have made a connection the `pfms::PlatformType::QUADCOPTER` we `read` this message type.

In a loop, read the laser messages and display the `range` and `bearing` to objects. Show the ranges that are within the `range_min` and `range_max` range readings. Utilise the `angle_increment` , `angle_min` and `angle_max` to get the `range` and `bearing`. Find teh closest object to the platform.

### Sonar

In the `receive_sonar.cpp` we aim to receive `pfms::sensor_msgs::SonarLaserScan` messages from a laser mounted on the `Quadcopter`. If we have made a connection the `pfms::PlatformType::QUADCOPTER` we `read` this message type.

In a loop, read the laser messages and display the `range` to objects. Show the range that is within the `range_min` and `range_max` range readings. Which `range` reading is available?

