# -*- coding: utf-8 -*-
"""
Created on Wed Mar  7 15:53:29 2018

@author: Toby Stephen Reed || 13487582
"""
import rospy
import numpy
import cv2

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 

# An array of waypoints the robot loops through to navigate around the map.
waypoints = [[2.0, -5.0], [-4.0, 2.0], [0.0, 0.0], [3.0, 4.0], [3.0, 2.0], [3.0, -1.0]]

class robotsAssignment:
    def __init__(self):
        self.bridge = CvBridge()
        
        cv2.namedWindow("Processed Vision", 1)
        cv2.namedWindow("Mask View", 2)
        cv2.startWindowThread()
        
        # Get the laser data
        self.laser_sub = self.laser_sub = rospy.Subscriber("/turtlebot/scan", LaserScan, self.laser_call)
        # Get the turtlebots view        
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw", Image, self.image_call)
        # ALlow us to retrieve data for the MCL movement        
        self.pose_sub = rospy.Subscriber("/turtlebot/amcl_pose", PoseWithCovarianceStamped, self.amcl)
        # Publish waypoints as goals
        self.moveBasePublish = rospy.Publisher('turtlebot/move_base_simple/goal', PoseStamped, queue_size = 10)
        # Publish the command velocity when it see's an object        
        self.publish = rospy.Publisher("/turtlebot/cmd_vel", Twist, queue_size=1)
        # Pretty self explanatory but this is used for creating a new movement object         
        self.twist = Twist()  
        # This is used to allow the initial initilaisation of the big mask but is set to false after,
        # allowing editing to the map after.
        self.firstCreation = True
        
        # Define the colour ranges
        self.redFound = False
        self.greenFound = False
        self.blueFound = False
        self.yellowFound = False
        self.allFound = False
        
        # Wait until it has initilised everything
        rospy.sleep(3)
        
    # Add our subscribed AMCL data to the variable
    def amcl(self, data):
        self.amclData = data
        
    # Adaption of Chapter 13: On Patrol - Programming Robots with ROS, Quiqley et al.
    def waypointToGoal(self, waypoints):
        goal_pose = PoseStamped()
        
        # Get the map from the bot
        goal_pose.header.frame_id = "/map"
        
        # Loop through the waypoints
        for goal in waypoints:
            ifMoving = True
            '''
            This section sets the position of the goal to the required elements 
            of the goal array, whilst setting the Z value to zero as it is always
            looking for a ground level object. Whilst setting the orientation to
            the value specified by the AMCL data. 
            '''
            goal_pose.pose.position.x = goal[0]
            goal_pose.pose.position.y = goal[1]
            goal_pose.pose.position.z = 0.0

            print(goal_pose.pose.position.x, goal_pose.pose.position.y)
            
            goal_pose.pose.orientation = self.amclData.pose.pose.orientation
            self.moveBasePublish.publish(goal_pose)
            
            print(str(goal_pose.pose.orientation))
            
            '''
            This while loop runs whilst the turtlebot is moving it checks whether
            the robots current location is at a goal position and if it is it stops
            moving and moves to the next goal.
            '''
            while ifMoving:
                if(-0.5 < (self.amclData.pose.pose.position.x - goal[0]) < 0.5) and (-0.5 < (self.amclData.pose.pose.position.y - goal[1]) < 0.5):
                    ifMoving = False
            rospy.sleep(2)
                    #print(str(ifMoving))

   #Adaption of Chapter 7: Wander-bot - Programming Robots with ROS, Quiqley et al.
    def laser_call(self, data):
        global distance
        # Limit Laser
        data.angle_min = -1
        data.angle_max = 1      
        distance = min(data.ranges)
        # Gets the object head on
        self.rangeOfObstacle = data.ranges[len(data.ranges)/2]
        # Distance from an object
        self.distance = min(data.ranges)
                     
    # Taken and adapted from my own Go To the colour workshop
    def image_call(self, data):
        lower_green = numpy.array([ 50,  150,  100])
        upper_green = numpy.array([90, 255, 255])
        
        lower_blue = numpy.array([ 100,  150,  50])
        upper_blue = numpy.array([150, 255, 255])
        
        lower_yellow = numpy.array([ 30,  200,  100])
        upper_yellow = numpy.array([50, 255, 195])

        lower_red = numpy.array([ 0,  100,  100])
        upper_red = numpy.array([10,255,255])
        
        self.set = numpy.array([0,0,0])
        
        # Convert the image message to a BGR image
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
     
        # Create the HSV image
        hsvImage = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        
        # Create the colour masks
        greenMask = cv2.inRange(hsvImage, lower_green, upper_green)
        blueMask = cv2.inRange(hsvImage, lower_blue, upper_blue)
        yellowMask = cv2.inRange(hsvImage, lower_yellow, upper_yellow)
        redMask = cv2.inRange(hsvImage, lower_red, upper_red)
        self.bigMask = cv2.inRange(hsvImage, self.set, self.set)

        # Get the image dimensions
        height, self.width, depth = self.image.shape
    
        self.search_top = 1 * height / 4
        self.search_bot = 3 * height / 4
        
        redMask[0:self.search_top, 0:self.width] = 0
        greenMask[0:self.search_bot:height, 0:self.width] = 0
        yellowMask[0:self.search_bot:height, 0:self.width] = 0
        blueMask[0:self.search_bot:height, 0:self.width] = 0
        
        # Create the mega mask
        if(self.redFound == False):
            self.bigMask += redMask
        if(self.blueFound == False):
            self.bigMask += blueMask            
        if(self.greenFound == False):
            self.bigMask += greenMask       
        if(self.yellowFound == False):
            self.bigMask += yellowMask
            
        self.bigMask[0:self.search_top, 0:self.width] = 0
        self.bigMask[self.search_bot:height, 0:self.width] = 0
       
        # All the features in the big mask
        M = cv2.moments(self.bigMask)
        
        if(M['m00'] > 3000000 and distance <= 3.0):    
            self.bigMask = self.bigMask
            # Create the circle in the center of the screen by using centroids          
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            cv2.circle(self.image, (cx, cy), 20, (0,0,255), -1)

            err = cx - self.width/2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 100
            self.publish.publish(self.twist)
             
            if distance <= 1.5:
                self.twist.linear.x = 0
                self.publish.publish(self.twist)
		# If the amount of pixels in the current mask = true and that colour hasn't been found then mark
		# it off and print it to the console
                if(cv2.inRange(hsvImage, lower_red, upper_red).sum() > 50 and self.redFound != True):
                    self.bigMask -= redMask
                    self.redFound = True 
                    print("Red pole found.")
                elif(cv2.inRange(hsvImage, lower_yellow, upper_yellow).sum() > 50 and self.yellowFound != True):
                    self.bigMask -= yellowMask
                    self.yellowFound = True 
                    print(self.yellowFound)
                    print("Yellow Pole Found.")
                elif(cv2.inRange(hsvImage, lower_green, upper_green).sum() > 50 and self.greenFound != True):
                    self.bigMask -= greenMask
                    self.greenFound = True 
                    print(self.greenFound)
                    print("Green Pole found.")                    
                elif(cv2.inRange(hsvImage, lower_blue, upper_blue).sum() > 50 and self.blueFound != True):
                    self.bigMask -= blueMask
                    self.blueFound = True 
                    print(self.blueFound)
                    print("Blue Pole found.")
                if(self.checkComplete() == True):
                    print("Found all the objects.")
                    
	# Show the mask and image
        cv2.imshow("Processed Vision", self.image)
        cv2.imshow("Mask View", self.bigMask)
        
    # If the task is complete
    def checkComplete(self):
        self.complete = False
        
        if(self.redFound == True and self.blueFound == True and self.yellowFound == True and self.greenFound == True):
            self.complete = True
        return self.complete
def main():
    rospy.init_node('robotsAssignment', anonymous=True)
    object = robotsAssignment()
    object.waypointToGoal(waypoints)
    rospy.spin()
    
distance = 1

if __name__ == "__main__":
    main()
    
    
