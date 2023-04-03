# Robot_RandomWalk
A simple simulation of a vacuum robot.
This is an Apple Silicon adaptation for running ROS and robotic software. 
Before following the next two steps, install Docker ([installation instructions for Mac](https://docs.docker.com/docker-for-mac/install/).

## 1. Setup
1. Open a terminal and clone this repository with the command `git clone https://github.com/jaismith/ros-apple-silicon`
2. Enter in the cloned repository folder, `cd ros-apple-silicon`
3. Run `docker-compose up --build`

(`ros.env` contains environment variables for ROS that can be modified before running the command in step 3.)

## 2. Running a ROS gazebo simulation for testing
Once the other terminal shows the following type of messages 

    Starting ros-apple-silicon_novnc_1 ... done
    Starting ros-apple-silicon_ros_1   ... done

open another terminal:
1. Run `docker-compose exec ros bash` (`docker-compose up` has to be running)
2. Run `source /opt/ros/melodic/setup.bash`
3. Run `roslaunch turtlebot3_gazebo turtlebot3_world.launch` and you should see a number of messages, including `[ INFO] [1617035063.438483400, 0.126000000]: DiffDrive(ns = //): Advertise odom on odom `

To see whether it was successful, 
5. Open your browser to `localhost:8080/vnc.html` and click connect.
6. The robotic simulator is now running in your browser.

## 3. To terminate

In the terminal open for step 2., press ctrl+c, which will stop the execution of the simulator. Once that is stopped -- you should see it as the terminal can accept commands -- press ctrl+d to exit the Docker container.

Afterwards, in the terminal open for step 1., press ctrl+c. Once terminated, you should see the following messages

    Stopping ros-apple-silicon_ros_1   ... done
    Stopping ros-apple-silicon_novnc_1 ... done

At this point, both terminals can be closed if you wish.
