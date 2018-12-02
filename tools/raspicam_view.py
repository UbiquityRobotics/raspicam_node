#!/usr/bin/python

import rospy
import math
import struct
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt8MultiArray
from cv_bridge import CvBridge, CvBridgeError

# Motion vectors with SAD values above threshold will be ignored:
SAD_THRESHOLD = 650

bridge = CvBridge()
color_map = None
last_imv = None

def create_colormap(num_elements):
	size = ( num_elements, 1, 1 )
	im = np.zeros(size, dtype=np.uint8)

	for i in range(0, num_elements):
		im[i,0,0] = i

	return cv2.applyColorMap(im, cv2.COLORMAP_JET)

def draw_imv(img, imv):
	height, width, channels = img.shape
	mbx = int(math.ceil(width/16.0))
	mby = int(math.ceil(height/16.0))

	if len(imv.data) != (mbx + 1) * mby * 4:
		print('Error: Wrong IMV size!')
		return

	# Draw colored arrow per each macroblock:
	for j in range(0, mby):
		for i in range(0, mbx):
			idx = (i + (mbx + 1) * j) * 4
			buff = imv.data[idx:idx+4]
			dx, dy, sad = struct.unpack('<bbH', buff)

			x = i*16 + 8;
			y = j*16 + 8;

			if dx == 0 and dy == 0:
				continue

			if sad >= SAD_THRESHOLD:
				continue

			color = ( int(color_map[sad,0,0]),
					  int(color_map[sad,0,1]),
					  int(color_map[sad,0,2]) )
			cv2.arrowedLine(img, (x+dx, y+dy), (x, y), color, 1)

def img_callback(msg):
	try:
		img = bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
	except CvBridgeError as e:
		print(e)
	else:
		if last_imv is not None:
			draw_imv(img, last_imv)
		cv2.imshow('raspicam_node viewer', img)
		cv2.waitKey(25)

def imv_callback(msg):
	global last_imv
	last_imv = msg

def main():
	global color_map
	color_map = create_colormap(SAD_THRESHOLD)

	img_topic = '/raspicam_node/image/compressed'
	imv_topic = '/raspicam_node/motion_vectors'

	rospy.init_node('raspicam_view')
	rospy.Subscriber(img_topic, CompressedImage, img_callback)
	rospy.Subscriber(imv_topic, UInt8MultiArray, imv_callback)
	rospy.spin()

if __name__ == '__main__':
	main()
