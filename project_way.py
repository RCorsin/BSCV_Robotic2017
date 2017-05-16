#!/usr/bin/env python

import roslib; roslib.load_manifest('rbx1_nav')
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs import *
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi
import numpy as np


class MoveBaseWayPoint():
    def __init__(self):
        self.number = 0                 #init the number of position we want to reach
        self.goal = MoveBaseGoal        #init the future goal
        self.WayPointsLists = list()    #create a list which will contain the goals
        self.index = 0                  #init an index
        self.over = 0                   #to know if we need to close the program
        self.MarkersLists = list()      #a list of markers
        self.success = 0                #this will contain the number of goals reached
        self.getnumber()                #use the function getNumber
    
        
        
    def getnumber(self):        

        #this function will be used to get the number of goals we want to reach at the end

        rospy.init_node('listener', anonymous=True) #we create our node

        while self.number < 1:      #we don't want a number of goals less than 1 so we 
                                    #create a loop to obtain a positive integer
            try:    
                self.number = int(raw_input('Enter the number of wanted waypoints: '))
                                    #we check if the number is an integer

                if self.number < 1: #if the number is less than one              
                    rospy.loginfo("Not a positive number")  #we ask a positive number
            except ValueError:      #if the input is not an integer
                rospy.loginfo("Not a positive number")      #we ask a positive number

        rospy.loginfo("Now you can enter your differents waypoints")    #we display that we can register our way points
        self.index = 0                          #we reset our index
        while self.index < self.number:         #we want to do it until we got the good number of goals
            rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.callback) #we subscribe to the goal node, if the goal node is used with a MoveBaseActionGoal we will use the function callback
            rospy.spin()    #used to let only one instruction pass, this will wait another instruction

    def callback(self, msg):    #this function will be used each time we create a new nav goal
        
        if self.over == 0:  #if we hadn't finish the program
            new_move = actionlib.SimpleActionClient("move_base", MoveBaseAction) #"new_move" will be a variable containing a action, here a MoveBaseAction to move the robot
            new_move.wait_for_server()  #we are waiting the robot server
            new_move.cancel_all_goals() #We cancel every goals to avoir a movement of the robot

            self.goal = msg     #goal (a MoveBaseGoal) become the new goal we entered

            self.add_markers(self.goal) #with the goal we add a new markers on this position

            if self.index == 0: #if this is the first move we allow it to not be compare
                self.WaypointsLists = self.WayPointsLists.append(self.goal) 
                    #We add the new goal to the goals list
                self.index = self.index +1
                    #we increment the index

            if self.WayPointsLists[self.index-1] != self.goal:
                    #we compare the new goal to the last one to avoid some useless move
                self.WaypointsLists = self.WayPointsLists.append(self.goal)
                    #We add the new goal to the goals list
                self.index = self.index +1
                    #we increment the index        

            if self.index >= self.number and not rospy.is_shutdown():
                    #if the index is equal or greater than the number of goals 
                rospy.loginfo("Procurement finished")   #we entered all the goals
                self.over = 1
                self.recurrence() #we call the function to reach goals  




    def recurrence(self):       #this function will be used when every goals will be inputed, this will ask to the robot to reach every goals, one by one in the order of input

        self.index = 0
                #we reset our index
        while self.index <= self.number and not rospy.is_shutdown():
                #if the index is still less than the number of goals
            self.goal = self.WayPointsLists[self.index]
                #our goal become the umpteenth value of the list of goals
            self.index = self.index + 1
                #we increase the index
            
            self.move(self.goal)
                #we call the move function with the new goal
            if self.index == self.number:
                #if we did all goals
                self.shutdown()
                #we can shutdown the program

    def move(self, goal):       #this function will make move the robot
            
            new_move = actionlib.SimpleActionClient("move_base", MoveBaseAction) #"new_move" will be a variable containing a action, here a MoveBaseAction to move the robot
            new_move.wait_for_server()  #we are waiting the robot server

            my_goal = MoveBaseGoal()    #This variable contain a MoveBaseGoal it will contain position of goals

            #We are creating our goal item by item to avoid a problem because of the conversion between MoveBaseAction and MoveBaseGoal

            my_goal.target_pose.header.frame_id = "/map"; # the frame id is used to determined the topic used
            my_goal.target_pose.pose.position.x = goal.goal.target_pose.pose.position.x #the spacial position X
            my_goal.target_pose.pose.position.y = goal.goal.target_pose.pose.position.y #the spacial position Y
            my_goal.target_pose.pose.orientation.z = goal.goal.target_pose.pose.orientation.z   #the orientation of the robot
            my_goal.target_pose.pose.orientation.w = goal.goal.target_pose.pose.orientation.w


            new_move.send_goal(my_goal)     #we are sending to the robot his goal converted

            rospy.loginfo("sending goal N*" + str(self.success+1)) #we display we are going to the umpteenth point

            new_move.wait_for_result() #we are waiting the robot to reach his destination

            state = new_move.get_state()    #we create a variable like a boolean to know if the robot is arrived
            if state == GoalStatus.SUCCEEDED:   #if we reach the destination
                rospy.loginfo("Goal N*" +str(self.success+1) + " succeeded") #we display that we arrived to the umpteenth goals       
                self.suppr_marks(self.success)      #we can suppress the marker from this destination
                self.success = self.success + 1     #we increase the variable containing the number of goals reached




    def add_markers(self, pos): #this function will add a marker to the position marked by goal
        marker = Marker()       #we create a new marker, a marker has an architecture pretty similar as the MoveBaseGoal
        
        marker.header.frame_id = "/map" #the topic
        
        marker.id = self.index  #the index, it need to be unique
        marker.ns = "Marker"    #some additional name if we need

        marker.action = marker.ADD  #the action permit to create, modify or delete a marker, here we create it
       
        marker.type = marker.ARROW  #the type of the marker, here an arrow

        marker.lifetime = rospy.Duration(0) #the life time of the marker, a 0 is equal to infity
      
        
        marker.pose.position.x = pos.goal.target_pose.pose.position.x   #position of the marker
        marker.pose.position.y = pos.goal.target_pose.pose.position.y
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = 0.0     #orientation of the marker
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = pos.goal.target_pose.pose.orientation.z
        marker.pose.orientation.w = pos.goal.target_pose.pose.orientation.w

        marker.header.stamp = rospy.Time.now() #we setup his internal timer to the actual time

        marker.scale.x = 0.75   #the size (in meter) of the marker
        marker.scale.y = 0.08
        marker.scale.z = 0.08

        marker.color.a = 1.0    #the color of the arrow in RGBA (red blue green alpha) , alpha need to be at 1 to not be invisible
        marker.color.b = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0

        marker.text = ("Goal N" + str(self.index)) #additionnal text 

        self.MarkersLists.append(marker) #we add this marker on the list of markers
        self.Marker_Publisher()         #we call the publisher function


    def Marker_Publisher(self): #this function publish all markers
        self.pub = rospy.Publisher('/waypoint_markers', Marker , queue_size=10) #we create a publisher, here we are publishing to the topic 
            # /waypoint_markers, we are publishing a marker.

        ind = 0     #the index

        while ind < len(self.MarkersLists):        #while the index is less than the lenght of the list

            self.pub.publish(self.MarkersLists[ind])    #we publish the umpteenth marker
            ind = ind +1                                #we increment the value of the index




    def suppr_marks(self, index):       #this function will suppress our marker

#        kill_marker = Marker()          

        if len(self.MarkersLists) > 0: #If the length of the list is greater than 0
            self.MarkersLists[index].action = self.MarkersLists[index].DELETE  #we change the action of marker to delete
        #with this he will delete himself on rviz
        
        self.Marker_Publisher() #we call the publisher
            


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        new_move = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        new_move.wait_for_server()
        new_move.cancel_all_goals()

        rospy.sleep(2)
        rospy.sleep(1)

        rospy.signal_shutdown("done")
        sys.exit('done')
        # Stop the robot




if __name__ == '__main__':
    try:
        MoveBaseWayPoint()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
        pass




