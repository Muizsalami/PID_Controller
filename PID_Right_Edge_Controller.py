
import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Here, i initialize my PID controller constants used to adjust the robot's response
kp = 0.8 # proportional gain
ki = 0.001 # Integral gain
kd = 0.04 # Derivative gain

# Initializing variables for the PID controller
previous_error = 0
integral_error = 0

mynode_ = None
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front1': 0,
    'front2': 0,
    'fleft': 0,
    'left': 0,
}
twstmsg_ = None

# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if ( twstmsg_ != None ):
        pub_.publish(twstmsg_)


def clbk_laser(msg):
    global regions_, twstmsg_
    
    regions_ = {
        #LIDAR readings are anti-clockwise
        'front1':  find_nearest (msg.ranges[0:5]),
        'front2':  find_nearest (msg.ranges[355:360]),
        'right':  find_nearest(msg.ranges[265:275]),
        'fright': find_nearest (msg.ranges[310:320]),
        'fleft':  find_nearest (msg.ranges[40:50]),
        'left':   find_nearest (msg.ranges[85:95])
        
    }    
    twstmsg_= movement()

    
# Find nearest point
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=10), 10)

# Function to compute the PID controller output
def pid_calc(error):
    global previous_error, integral_error, kp, ki, kd

    #calculate proportional, integral, and derivative terms
    P = kp * error

    if abs(integral_error + error) < 0.2:
        integral_error = integral_error + error
    I = ki * integral_error

    D = kd * (error - previous_error)

    # Combine terms to get the final output
    output = P + I + D

    # Update previous error for the next iteration
    previous_error = error

     # Log PID components for debugging
    mynode_.get_logger().info(f"PID Calculation - P: {P:.2f}, I: {I:.2f}, D: {D:.2f}, Output: {output:.2f}")

    return output

# Movement logic for the robot based on PID controller
def movement():
    global regions_, mynode_

    # setpoint(Desired distance) is defined
    setpoint = 0.4
    
    #create an object of twist class, used to express the linear and angular velocity of the turtlebot 
    msg = Twist()
    current_distance = regions_['fright']  # I use the front right sensor range to derive current distance
    error = setpoint - current_distance    # calculation of error using desired and current distance

    # Logging current distance and error for debugging
    mynode_.get_logger().info(f"Current distance: {current_distance:.2f}, Error: {error:.2f}")

    # calling the PID calculation to calculate the angular velocity
    pid_output = pid_calc(error)
    msg.linear.x = 0.1  # constant forward speed
    msg.angular.z = pid_output  # my output will determine the angular velocity

    return msg

#used to stop the robot
def stop():
    global pub_
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub_.publish(msg)


def main():
    global pub_, mynode_

    rclpy.init()
    mynode_ = rclpy.create_node('reading_laser')

    # define qos profile (the subscriber default 'reliability' is not compatible with robot publisher 'best effort')
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )

    # publisher for twist velocity messages (default qos depth 10)
    pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)

    # subscribe to laser topic (with our qos)
    sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)

    # Configure timer
    timer_period = 0.2  # seconds 
    timer = mynode_.create_timer(timer_period, timer_callback)

    # Run and handle keyboard interrupt (ctrl-c)
    try:
        rclpy.spin(mynode_)
    except KeyboardInterrupt:
        stop()  # stop the robot
    except:
        stop()  # stop the robot
    finally:
        # Clean up
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()