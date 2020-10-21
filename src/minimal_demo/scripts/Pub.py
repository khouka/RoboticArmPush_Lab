#!/usr/bin/env python 
import rospy
from std_msgs.msg import Float64

# Import other necessary libraries

class <Class name>():
    def __init__(self):
        rospy.init_node('<node name>', anonymous=True)
        self.pub = rospy.Publisher('<ROS topic name', <message type>, queue_size=10)
        self.rate = rospy.Rate(<Integer>) 
    # Define a subclass
    def <subClass name>(self):  
        # To publish the message
        self.pub.publish(<the value>)

def main():
    obj = <Class name>()
    # obj.<subclass name>
        

if __name__ == '__main__':
    try:
       main() 
    except rospy.ROSInterruptException:
        pass

