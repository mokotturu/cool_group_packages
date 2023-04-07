import sys
import os

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class image_converter:
	def __init__(self):
		self.image_pub = rospy.Publisher("/detected", Image, queue_size=10)
		if self.image_pub == None:
			print("None")
		self.image_sub = rospy.Subscriber("/webcam/image_raw", Image, self.callback)
		if self.image_sub == None:
			print("None")
		
		self.bridge = CvBridge()

	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		# rotate cvimage 90 degrees clockwise
		# cv_image = cv2.transpose(cv_image)

		# face detection (not recognition)
		dir_path = os.path.realpath(os.path.dirname(__file__))
		# Load the cascade
		face_cascade = cv2.CascadeClassifier(dir_path + '/../data/haarcascade_frontalface_default.xml')
		# Read the input image
		img = cv_image
		# Convert into grayscale
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		# Detect faces
		faces = face_cascade.detectMultiScale(gray, 1.1, 4)
		# Draw rectangle around the faces
		for (x, y, w, h) in faces:
			cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)

		cv_image = img

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