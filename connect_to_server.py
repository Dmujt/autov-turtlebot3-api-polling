import sys
import time
import json
import requests

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion


#server endpoint to access
QUEUE_ENDPOINT = "https://msu-turtlebot-813.herokuapp.com/api/pickup_requests/"
MAX_POLL_TIME = 2

#go to place
class GoToPose():
    def __init__(self):

        self.goal_sent = False

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
        
        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

        self.parent_node = {} #need to set later

    def goto(self, pos, quat):
        # Send a goal
        self.goal_sent = True

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	    # Start moving
        self.move_base.send_goal(goal)

	    # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(180)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

#class to initiate connection to the server and start polling for navigation requests
class QueueServer():
    #constructor
    def __init__(self, apikey, nav_node):
        self.navigator = nav_node

        #the request being worked on by the bot
        self.activeRequest = {}
        #api key to authenticate with
        self.x_api_key = apikey
        self.x_bot_lat = 0.0
        self.x_bot_long = 0.0
        #Turtlebot status (from one of the states in the diagram)
        self.x_bot_state = 'stationary'
        rospy.loginfo("Starting Connection...")

    #headers for authentication in the request
    def req_headers(self):
        return {
            'X-API-KEY': str(self.x_api_key),
            'X-BOT-LAT': str(self.x_bot_lat),
            'X-BOT-LON': str(self.x_bot_long),
            'X-BOT-STATE': str(self.x_bot_state)
        }

    #poll the server for any new requests in the queue
    def start_polling(self):
        rospy.loginfo("Checking Queue...")
        self.x_bot_state = 'stationary'
        req = requests.get(url=QUEUE_ENDPOINT + "/last", headers=self.req_headers())
        self.parse_request(req)

    #update the state of a running request
    def update_request(self):
        self.x_bot_state = 'moving'
        req = requests.put(url=QUEUE_ENDPOINT + str(self.activeRequest["id"]), headers=self.req_headers(), json={
            #update request attributes depending on the current request status
            'pickup_request': {
                'rstatus': 'in_progress'
            }
        })        
        #check status returned and perform action accordingly
        self.parse_request(req)

    #update the state of a running request
    def complete_request(self):
        self.x_bot_state = 'moving'
        req = requests.put(url=QUEUE_ENDPOINT + str(self.activeRequest["id"]) + "/completed", headers=self.req_headers())          
        #check status returned and perform action accordingly
        self.parse_request(req)

    #if the robot fails to get to its location, set to the erorr state and resume polling
    def set_as_error(self):
        self.x_bot_state = 'in_error'
        req = requests.put(url=QUEUE_ENDPOINT + str(self.activeRequest["id"]) + "/cancel", headers=self.req_headers())          
        #check status returned and perform action accordingly
        self.parse_request(req)

    #move
    def perform_request(self):
        self.x_bot_lat = float(self.activeRequest["x_coordinate"])
        self.x_bot_long = float(self.activeRequest["y_coordinate"])
        z = 0
        try:
            # Customize the following values so they are appropriate for your location
            position = {'x': self.x_bot_lat, 'y' : self.x_bot_long}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
            success = self.navigator.goto(position, quaternion)

            if success:
                rospy.loginfo("Hooray, reached the desired pose")
                #reached
                self.complete_request()
            else:
                rospy.loginfo("The base failed to reach the desired pose")
                #update to error state
                self.set_as_error()

            # Sleep to give the last log messages time to be sent
            #rospy.sleep(1)

        except rospy.ROSInterruptException:
            rospy.loginfo("Ctrl-C caught. Quitting")

    #parse the request from JSON to a dictionary
    def parse_request(self, req):
        if (int(req.status_code) == 200):
            #parse the request
            if (req.content == "null" or req.content == "{}"):
                #nothing to do
                rospy.loginfo("Nothing to do...")
                time.sleep(MAX_POLL_TIME)
                self.start_polling()
            else:
                rospy.loginfo("Request received..")
                req = json.loads(req.content.decode("utf-8") )
                self.activeRequest = req
                #perform move if not completed
                if (self.activeRequest["rstatus"] == "completed" or self.activeRequest["rstatus"] == "cancelled"):
                       #TODO: figure out how to get the coordinates from GPS from server
     #move to stationary and resume polling
                    rospy.loginfo("Request completed...")
                    self.start_polling()
                elif (self.activeRequest["rstatus"] == "in_progress"):
                    #start performing request
                    self.perform_request()
                else:
                    #mark as in progress
                    rospy.loginfo("Starting request...")
                    self.update_request()
        else:
            #an error :0
            rospy.loginfo("Error reaching server!")


#establish connection to server and start poll and navigating
#call with: python connect_to_server.py API_KEY_FROM_SERVER_HERE
def main():
    apikey = sys.argv[1]
    rospy.init_node('nav_test', anonymous=False)
    navnode = GoToPose()

    server = QueueServer(str(apikey), navnode)  
    server.start_polling()

if __name__ == '__main__':
    main()
