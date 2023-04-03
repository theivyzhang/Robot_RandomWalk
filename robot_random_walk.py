#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Ivy Aiwei Zhang
# Date: March 29th 2023

# Import of python modules.
import math # use of pi.
import random

# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan' # name of topic for Stage simulator. For Gazebo, 'scan'

# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

# Velocities that will be used (feel free to tune)
LINEAR_VELOCITY = 1 # m/s
ANGULAR_VELOCITY = math.pi/4 # rad/s

# Threshold of minimum clearance distance (feel free to tune)
MIN_THRESHOLD_DISTANCE = 0.5 # m, threshold distance, should be smaller than range_max
# Field of view in radians that is checked in front of the robot (feel free to tune)
MIN_SCAN_ANGLE_RAD = -10.0 / 180 * math.pi;
MAX_SCAN_ANGLE_RAD = +10.0 / 180 * math.pi;
# RESTRICTION

class RandomWalk():
    def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY, min_threshold_distance=MIN_THRESHOLD_DISTANCE,
        scan_angle=[MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD]):
        """Constructor."""

        # Setting up publishers/subscribers.
        # Setting up the publisher to send velocity commands.
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        # Setting up subscriber receiving messages from the laser.
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)

        # Parameters.
        self.linear_velocity = linear_velocity # Constant linear velocity set.
        self.angular_velocity = angular_velocity # Constant angular velocity set.
        self.min_threshold_distance = min_threshold_distance
        self.scan_angle = scan_angle
        
        # Flag used to control the behavior of the robot.
        self._close_obstacle = False # Flag variable that is true if there is a close obstacle.

    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities.
        twist_msg = Twist()

        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    def _laser_callback(self, msg):
        """Processing of laser message."""
        # Access to the index of the measurement in front of the robot.
        # NOTE: assumption: the one at angle 0 corresponds to the front.
        
        if not self._close_obstacle:
            # Please double check the LaserScan message http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
            # Find the minimum range value between min_scan_angle and max_scan_angle
            min_range = msg.ranges[0]
            assert self.min_threshold_distance < msg.range_max
            for i in range(len(msg.ranges)):
                angle = msg.angle_min + (i*msg.angle_increment)
                if angle >= self.scan_angle[0] and angle <= self.scan_angle[1] and (msg.ranges[i] < min_range or min_range is None):
                    min_range = msg.ranges[i]

            # If the minimum range value is closer to min_threshold_distance, change the flag self._close_obstacle
            if min_range is not None and min_range < self.min_threshold_distance:
                self._close_obstacle = True


    def spin(self):
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.
        while not rospy.is_shutdown():
            # Keep looping until user presses Ctrl+C

            # If the flag self._close_obstacle is False, the robot should move forward.
            # Otherwise, the robot should rotate for a random amount of time
            # after which the flag is set again to False.
            # Use the function move already implemented, passing the default velocities saved in the corresponding class members.

            if not self._close_obstacle:
                self.move(LINEAR_VELOCITY, 0)
            else:
                start_time = rospy.get_rostime()

                random_angle = random.uniform(-math.pi, math.pi)  #random angle in radian

                duration = abs(random_angle/self.angular_velocity) # get the duration

                while not rospy.is_shutdown():
                    if rospy.get_rostime() - start_time >= rospy.Duration(duration):
                        break

                    if (self.angular_velocity < 0):
                        self.angular_velocity = self.angular_velocity * -1

                    self.move(0 ,self.angular_velocity)

                self._close_obstacle = False
            ####### ANSWER CODE END #######

            rate.sleep()
        

def main():
    """Main function."""

    # 1st. initialization of node.
    rospy.init_node("random_walk")

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(2)

    # Initialization of the class for the random walk.
    random_walk = RandomWalk()

    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(random_walk.stop)

    # Robot random walks.
    try:
        random_walk.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    """Run the main function."""
    main()
