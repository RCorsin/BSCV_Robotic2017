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
        self.over = 0
        self.dir = Marker()
        self.MarkersLists = list()
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
        
        if self.over == 0:  
            new_move = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            new_move.wait_for_server()
            new_move.cancel_all_goals()

            self.goal = msg     #goal (a MoveBaseGoal) become the new goal we entered

            rospy.loginfo(msg)


            self.add_markers(self.goal)


            
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

            rospy.loginfo(self.index)
            rospy.loginfo(self.number)
            
            self.move(self.goal)
                #we call the move function with the new goal
            if self.index == self.number:
                #if we did all goals
                self.shutdown()
                #we can shutdown the program

    def move(self, goal):
            
            new_move = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            new_move.wait_for_server()



            my_goal = MoveBaseGoal()
            my_goal.target_pose.header.frame_id  = 'Talker N1'
            my_goal.target_pose.header.frame_id = "/map";
            my_goal.target_pose.pose.position.x = goal.goal.target_pose.pose.position.x
            my_goal.target_pose.pose.position.y = goal.goal.target_pose.pose.position.y
            my_goal.target_pose.pose.orientation.z = goal.goal.target_pose.pose.orientation.z
            my_goal.target_pose.pose.orientation.w = goal.goal.target_pose.pose.orientation.w
            rospy.loginfo(my_goal)


            new_move.send_goal(my_goal)
            rospy.loginfo("sending goal")

            new_move.wait_for_result()

            state = new_move.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded")  
                self.suppr_marks()      

         #   if not finished_within_time:
          #      new_move.cancel_goal()
           #     rospy.loginfo("Timed out achieving goal")
           # else:
            #    state = new_move.get_state()
             #   if state == GoalStatus.SUCCEEDED:
              #      rospy.loginfo("Goal succeeded!")
                

             

    def add_markers(self, pos):
        marker = Marker()
        
        marker.header.framer_id = "marker N[self.index]"
        marker.type = arrow

        marker.pose.position.x = pos.goal.target_pose.pose.position.x
        marker.pose.position.y = pos.goal.target_pose.pose.position.y
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = pos.goal.target_pose.pose.orientation.z
        marker.pose.orientation.w = pos.goal.target_pose.pose.orientation.w

        marker.scale.x = 1.0
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.a = 1.0
        marker.color.b = 0.0
        marker.color.r = 0.0
        marker.color.g = 1.0

        self.MarkersLists.appends(marker)
        self.Marker_Publisher()


    def Marker_Publisher(self):
        ind = 0
        marker = Marker()
        while ind < len(self.MarkersLists):
            marker = self.MarkersLists(ind)
            pub = rospy.Publisher('visu_marker', Marker() ) 
            pub.publish(marker)
            ind = ind +1




    def suppr_marks(self):
        if len(self.MarkersLists) >= 1:
            del self.MarkersLists[0]
        self.Marker_Publisher()
            





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




