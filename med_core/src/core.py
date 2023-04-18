import sys
import datetime, time
import numpy as np

import rospy
from std_msgs.msg import Int64MultiArray
from move_base_msgs.msg import MoveBaseGoal

class state_machine:
    def __init__(self):
        self.detected_sub = rospy.Subscriber("/detected", Int64MultiArray, self.callback)
        self.data = []
        self.nav_pub = rospy.Publisher("/move_base", MoveBaseGoal)

    def callback(self, data):
        self.data = data.data

    def seek(self, MAX_TIME=3000):
        buffer_size = 10
        count = np.zeros(buffer_size)
        THRESHOLD = 0.5
        MIN_FRAMES = 10
        start_time = time.time()*1000
        elapsed_time = 0
        total = 0
        while elapsed_time < MAX_TIME:
            total += 1
            if self.data[0] > 0:
                count[:-1] = count[1:]
                count[-1] = 1
                start_time = time.time()*1000
                print("I see *someone*")
            else:
                count[:-1] = count[1:]
                count[-1] = 0
            if total > MIN_FRAMES and np.mean(count) > THRESHOLD:
                break
            elapsed_time = time.time()*1000 - start_time
        # did we find them?
        print("Found: ")
        print(np.mean(count) >= THRESHOLD)

    def start(self):
        while True:
            # WAITING AT BASE STATION
            # wait for input from user to simulate a
            # user's medication time
            print("Wating at base station...")
            input()

            # ACQUIRE MEDICATION
            now = datetime.datetime.now()
            print(now)
            print("### Monish Reddy Kotturu ###")
            print(" - 500mg Viagra")
            print()
            print("Press Enter When Medication Loaded")
            input()

            # SEEK
            print("Seeking...")
            position = [-3.991, -1.297, 0.000]
            orientation = [0.000, 0.000, 0.006, 1.000]

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = position[0]
            goal.target_pose.pose.position.y = position[1]
            goal.target_pose.pose.position.z = 0.0
            goal.target_pose.pose.orientation.x = orientation[0]
            goal.target_pose.pose.orientation.y = orientation[1]
            goal.target_pose.pose.orientation.z = orientation[2]
            goal.target_pose.pose.orientation.w = orientation[3]
            print(goal)

            self.nav_pub.publish(goal)
            print("hiakfaduajf")

            # DO ACTUAL STUFF
            # wait for /detected for x conesecutive frames
            self.seek()

            # DELIVER MEDICATION
            # do more stuff
            print("Handoff meds")

            # GO HOME
            # nav goal
            print("go hoe")

def main(args):
    rospy.init_node('core', anonymous=True)
    sm = state_machine()
    sm.start()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

