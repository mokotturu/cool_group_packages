import sys
import datetime, time
import numpy as np

import rospy
from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import PoseStamped, Twist
from actionlib_msgs.msg import GoalStatusArray

class state_machine:
    def __init__(self):
        self.detected_sub = rospy.Subscriber("/detected", Int64MultiArray, self.callback)
        self.data = []
        self.nav_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback)
        self.is_busy = False
        self.vel_publisher = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=1)

    def callback(self, data):
        self.data = data.data

    def status_callback(self, data):
        self.is_busy = len(data.status_list) > 0 and data.status_list[-1].status == 1

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
        return np.mean(count) >= THRESHOLD

    def navigate(self, position, orientation):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = position[0]
        goal.pose.position.z = 0.0
        goal.pose.position.y = position[1]
        goal.pose.orientation.x = orientation[0]
        goal.pose.orientation.y = orientation[1]
        goal.pose.orientation.z = orientation[2]
        goal.pose.orientation.w = orientation[3]
        # print(goal)

        self.nav_pub.publish(goal)

    def rotate(self, time_ms=1000):
        print('rotating...')
        start_time = time.time()
        vel = Twist()
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = 0.5
        dt = time.time() - start_time
        while dt < time_ms/1000:
            self.vel_publisher.publish(vel)
            dt = time.time() - start_time
            print(dt)
        vel.angular.z = 0.0
        self.vel_publisher.publish(vel)
        print('DONE rotating.')


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
            positions = [[-3.991, -1.297, 0.000], [-1.582, -0.825, 0.000], [0.584, -0.759, 0.000]]
            orientation = [0.000, 0.000, 0.006, 1.000]
            found = False
            for idx, p in enumerate(positions):
                print(f"going to position {idx}...")
                self.navigate(p, orientation)
                time.sleep(1)
                while self.is_busy:
                    time.sleep(1)
                # spin
                self.rotate(10000)
                if self.seek():
                    found = True
                    break

            if not found:
                # recovery behavior
                print("oh shit.. aint nobody here?")

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

