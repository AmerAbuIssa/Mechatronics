Quiz 4
======

Part A
------

**PREAMBLE**

The Quiz reinforces concepts of Constructor, the logic required for overall control via threading, using laser data and finally an overall control in reaching goals and using laser data.

We have been provided the scaffolding of the `Controller` and `Quadcopter` as per Assignment 2. Just like the assessment, functionality has to be implemented across the base class `Controller` and derived class `Quadcopter`. The code will not compile until functions are implemented,  you can leverage your Assignment 2 code for the quiz.

 The quiz has been developed against **pfms2ros** debian package ( version 3.0.8 or above ).  You will receive an error if you do not have this version and can upgrade via:
```
sudo apt update
sudo apt install pfms-ros-integration
```

Before running the unit tests make sure you have run the simulator `ros2 launch pfms quiz4.launch.py`.   In between running tests either restart the simulator `ros2 service call /reset_world std_srvs/srv/Empty` or the entire launch.

We have been provided some tests to let us debug, isolate, fix issues and create the Quadcopter class. The unit tests are in [`utest.cpp` in ./a/test folder](./a/test/utest.cpp) and they can be run from build directory via `./test/utest` and you can also find them in vscode (select `utest` to run).  There are four tests provided in the `QuadcopterTest` suite (to further assist A2 development).  

**TASK 1 - Initialisation of class**

All member variables belonging to the class need to be initialised to default values. When we have a base and derived class the initialisation can occur in either the [Base Class Constructor (Controller)](./a/controller.cpp)  or the [Derived Class Constructor (Quadcopter)](./a/quadcopter.cpp) .  Initialise the variables in the appropriate constructor so that the `Constructor` tests pass. 

We also need to check that all function in the interface return values as specified, and they can operate even if no goals are yet set (the constructor does not take goals, therefore any function can be called when the object of the class is created.) 

HINT: Check what is being tested in `QuadcopterTest, Constructor` test, look where the variables exist and consider where they should be initialised. Consider that you would also have another derived class `Ackerman` in Assignment 2.  

**TASK 2 - Checking obstacles**

The quad copter is equipped with a laser scanner, which allows it to detect obstacles. We have supplied the `LaserProcessing` class, and have initialised it in the constructor of `Quadcopter`. The role of the `LaserProcessing` class is to detect the centre of a obstacle present in the laser scan.  In this quiz there is only one obstacles present in the laser scan, therefore it's location can be approximated by the centre of all the valid laser readings (range vector). A reading is valid if it is reported as being in between the minimum and maximum range. For details of using laser data refer to Week 8 quad example, where we obtained and examine laser data. Implement the `getObstacles` function of `Quadcopter` class, which return the centre of the obstacle in the world coordinates. We examined local to global conversion in Quiz3 and it would be very useful here. So to do this task you will need to call `laserProcessingPtr_->getObstacles` and use the platform odometry to the to world (global) coordinates. 

**TASK 3 - Overall logic**

Examine `QuadcopterTest, StartingLogic` which shows the overall functionality expected from Platforms in Assignment 2 in commencing execution. The platform will start in IDLE until goals are set (via `setGoals`) AND `run` is called. At which point the status changes to `RUNNING` and the platform commences motion. As the `run` call is non blocking, it should return immediately.  Consider that `run` kicks off execution by starting/triggering another thread, as the control needs to monitor the platform position and control it to reach one goal at a time (supplied via `setGoals`).

The function  [reachGoal in Quadcopter](./a/quadcopter.cpp) is private, and could be used to control the platform. Unlike assignment 1, here there are a series of goals, so you will need to consider how this function will be called to execute (HINT, think threading and mutex/convars). 

The test here emphasises that the state reported is correct, that run is not blocking, the platform commences motion by changing state and moving.

 **TASK 4 - Checking obstacles after reaching last goal**

This test builds upon Assessment 2 and the checking obstacle logic of TASK 3. However, here we need the full control logic of Assessment 2 and `getObstacles` is called after the last goal is reached. 

For the full control logic, consider how to control the platform towards the goals(s). Unlike the husky in Assessment 1 here we need to use the orientation of the platform and the direction we need to head towards, we can control simultaneously up/down and forward/backward and left/right. In essence we can control motion in full 3d  without having to rotate the platform on the spot. 

There is a caveat here, if we stop suddenly, the platform will exhibit an enormous amount of inertia  and for a moment would be looking at the floor. To circumvent this at present I would recommend coming to a slow stop at the very last goal. If you do not complete this, despite all logic being correct the laser will see the ground and your code will not detect objects correctly. 

In the code provided here, the `reachGoal` will terminate as it does not have any control logic, I would suggest to implement it and think about how it will commence and run as a thread.


Part B
-------

To undertake testing and developing your code you only need to add symbolically link quiz4 part b to your ros2_ws. 

For instance my git is ~/git/pfms-2024a-alalemp (change for your location of your git) and therefore only once do i need to complete below.

```bash
cd ~/ros2_ws/src
ln -s ~/git/pfms-2024a-alalemp/quizzes/quiz4/b 
```

Open up the code by opening the `ros2_ws/src` folder in vscode. You should be able to see a `b` folder. If it is in red you have not linked correct folder path. We need to implement a single function in  [analysis](./b/src/analysis.h) class.

When you need to compile your code you need to be in folder `~/ros2_ws/`and compile via a`colcon` command, you need to execute the command every time you change the code and want to execute the code with the changes. This can not be done via vscode.

You can either build all packages in `ros2_ws` via `colcon build --symlink-install` or you can specify a single package `colcon build --symlink-install --packages-select quiz4_b`for instance.

To check the unit test, in the terminal you run

```bash
ros2 run quiz4_b utest
```

**TASK 5 - Count characters**

Counts the number of characters (including spaces, special characters and numbers) in the string supplied: for instance "foo boo 12" has 9 characters. 

