import sys

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class image_converter:
	def __init__(self):
		self.image_pub = rospy.Publisher("/detected", Image, queue_size=10)
		if self.image_pub == None:
			print("None")
		self.image_sub = rospy.Subscriber("/movie", Image, self.callback)
		if self.image_sub == None:
			print("None")
		
		self.bridge = CvBridge()

	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		# rotate cvimage 90 degrees clockwise
		cv_image = cv2.transpose(cv_image)

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)

def main(args):
	rospy.init_node('face_detection', anonymous=True)
	ic = image_converter()
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)