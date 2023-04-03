# Robot_RandomWalk
A simple simulation of a vacuum robot.
This is an Apple Silicon adaptation for running ROS and robotic software. 
Before following the next two steps, install Docker ([installation instructions for Mac](https://docs.docker.com/docker-for-mac/install/).

## 1. Setup
1. Open a terminal and clone this repository with the command `git clone https://github.com/jaismith/ros-apple-silicon`
2. Enter in the cloned repository folder, `cd ros-apple-silicon`
3. Run `docker-compose up --build`

(`ros.env` contains environment variables for ROS that can be modified before running the command in step 3.)

## 2. Running a ROS gazebo simulation for testing the 
Once the other terminal shows the following type of messages 

    Starting ros-apple-silicon_novnc_1 ... done
    Starting ros-apple-silicon_ros_1   ... done

open another terminal:
1. Run `docker-compose exec ros bash` (`docker-compose up` has to be running)
2. Run `source /opt/ros/melodic/setup.bash`
3. To make the simulation less computationally expensive on your computer, you can run the 2D simulation we have seen in class. To install it, you can run `apt install ros-melodic-nav2d-tutorials`. 
4. After running `roscore`, here is the command to start the simulation: `rosrun stage_ros stageros /opt/ros/melodic/share/nav2d_tutorials/world/tutorial.world`.

To see whether it was successful, 
5. Open your browser to `localhost:8080/vnc.html` and click connect.
6. The robotic simulator is now running in your browser.

## 3. To terminate

In the terminal open for step 2., press ctrl+c, which will stop the execution of the simulator. Once that is stopped -- you should see it as the terminal can accept commands -- press ctrl+d to exit the Docker container.

Afterwards, in the terminal open for step 1., press ctrl+c. Once terminated, you should see the following messages

    Stopping ros-apple-silicon_ros_1   ... done
    Stopping ros-apple-silicon_novnc_1 ... done

At this point, both terminals can be closed if you wish.
