import sys
import datetime, time
import numpy as np

import rospy
from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import PoseStamped, Twist
from actionlib_msgs.msg import GoalStatusArray
from stretch_moveit_shim.srv import SetJoints, SetJointsRequest, SetBodyResponse
from stretch_moveit_shim.msg import Joint

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
        count = []
        THRESHOLD = 0.2
        start_time = time.time()*1000
        elapsed_time = 0
        while elapsed_time < MAX_TIME:
            count.append(1 if self.data[0] > 0 else 0) # detected
            elapsed_time = time.time()*1000 - start_time
        # did we find them?
        avg_detection = np.mean(count)
        found = avg_detection >= THRESHOLD
        print(f"Found: {found} - {avg_detection}")
        return found

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

    def rotate(self, angle, speed):
        angle = np.radians(angle)
        speed = np.radians(speed)
        time_s = angle/speed
        print('rotating...')
        start_time = time.time()
        vel = Twist()
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = speed
        dt = time.time() - start_time
        while dt < time_s:
            self.vel_publisher.publish(vel)
            dt = time.time() - start_time
        vel.angular.z = 0.0
        self.vel_publisher.publish(vel)
        print('DONE rotating.')

    def deliver(self):
        detected, x, y, w, h = self.data
        face_center = x+(w//2)
        img_width = 240
        img_center = img_width//2
        
        p = 0.05
        d = img_width*p

        vel = Twist()
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        
        speed = 0.1

        # center in frame
        while abs(img_center-face_center) > d:
            detected, x, y, w, h = self.data
            face_center = x+(w//2)
            if face_center < img_center-d:
                vel.angular.z = -1*speed
            else:
                vel.angular.z = 1*speed
            self.vel_publisher.publish(vel)
        vel.angular.z = 0.0
        self.vel_publisher.publish(vel)

        # approach

    def wiggle(self):
        try:
            service = rospy.ServiceProxy('/stretch_interface/set_joints', SetJoints) # setup a service client
            msg = Joint(joint_name='joint_head_tilt', val = -0.5) # create a joint with a name and value
            service([msg]) # pass an array of joints. This code will block
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        try:
            service = rospy.ServiceProxy('/stretch_interface/set_joints', SetJoints) # setup a service client
            msg = Joint(joint_name='joint_head_tilt', val = 0.5) # create a joint with a name and value
            service([msg]) # pass an array of joints. This code will block
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        try:
            service = rospy.ServiceProxy('/stretch_interface/set_joints', SetJoints) # setup a service client
            msg = Joint(joint_name='joint_head_tilt', val = 0.0) # create a joint with a name and value
            service([msg]) # pass an array of joints. This code will block
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

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
            positions = [[-1.582, -0.825, 0.000]]
            orientation = [0.000, 0.000, 0.006, 1.000]
            found = False
            for idx, p in enumerate(positions):
                print(f"going to position {idx}...")
                self.navigate(p, orientation)
                time.sleep(1)
                while self.is_busy:
                    time.sleep(1)
                # spin
                rotations = 7
                for _ in range(rotations):
                    self.rotate(360/rotations, 45)
                    if self.seek():
                        found = True
                        break
                if found:
                    break

            if found:
                # DELIVER MEDICATION
                # do more stuff
                print("Handing off meds")
                self.deliver()
                self.wiggle()
                time.sleep(5)
            else:
                print("I didn't find anyone! :(")

            # GO HOME
            # nav goal
            print("go hoe")
            self.navigate([-4.262, -3.064, 0.000], [-0.000, -0.000, 0.425, 0.905])

def main(args):
    rospy.init_node('core', anonymous=True)
    rospy.wait_for_service('/stretch_interface/set_joints') # check to make sure service client is available

    sm = state_machine()
    sm.start()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

