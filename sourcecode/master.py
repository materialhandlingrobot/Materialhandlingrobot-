#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16

from pyzbar import pyzbar
# import datetime
import imutils
# import time
import cv2

distance = 0.0

def scanner(frame, width, height):
	barcodeCenter = 0
	# find the barcodes in the frame and decode each of the barcodes
	barcodes = pyzbar.decode(frame)
	# loop over the detected barcodes
	for barcode in barcodes:
		# extract the bounding box location of the barcode and draw
		# the bounding box surrounding the barcode on the image
		(x, y, w, h) = barcode.rect
		barcodeCenter = int(x+w/2)
		cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
		# draw point
		cv2.circle(frame, (int(x + w/2),int(y + h/2)), radius=10, color=(0, 0, 255), thickness=-1)
		cv2.circle(frame, (int(width/2),int(height/2)), radius=10, color=(0, 255, 0), thickness=-1)
		# the barcode data is a bytes object so if we want to draw it
		# on our output image we need to convert it to a string first
		barcodeData = barcode.data.decode("utf-8")
		barcodeType = barcode.type
		# draw the barcode data and barcode type on the image
		text = "{} ({})".format(barcodeData, barcodeType)
		cv2.putText(frame, text, (x, y - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        #cv2.waitKey(1)
	# print("barcode center is {}".format(barcodeCenter))
        return (frame, barcodeCenter)

def laserCallback(msg):
    global distance 
    distance = 0
    pointCloudSize = len(msg.ranges)
    # print("dist are {}, {}, {}, {} and {}".format(msg.ranges[0], msg.ranges[int(pointCloudSize/4)], msg.ranges[int(pointCloudSize/2)], msg.ranges[int(3*pointCloudSize/4)], msg.ranges[pointCloudSize -1]))
    for dist in msg.ranges[pointCloudSize/2-1:pointCloudSize/2+1]:
        distance += dist
    distance = msg.ranges[pointCloudSize/2]
    print('distance is {}'.format(distance))
    
def init():
    rospy.init_node('control')
    arduPub = rospy.Publisher('/blinkm', Int16, queue_size=10)
    laserSub = rospy.Subscriber('/scan', LaserScan, laserCallback)

    # initialize the video stream and allow the camera sensor to warm up
    print("[INFO] starting video stream...")
    vs = cv2.VideoCapture(0)
    rospy.sleep(2)
    width = vs.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = vs.get(cv2.CAP_PROP_FRAME_HEIGHT)
    while not rospy.is_shutdown():
        # grab the frame from the threaded video stream and resize it to
        # have a maximum width of 400 pixels
        rec, frame = vs.read()
        newFrame, barcodeCenter = scanner(frame, width, height)
        #print('Pixel diff is {}'.format(width/2 - barcodeCenter))
        # cmd = input("Enter Direction: \n")
        # arduPub.publish(int(cmd))
        # rospy.sleep(2)
	# arduPub.publish(5) 
	cv2.imshow("image", imutils.resize(newFrame, width= 400))
	cv2.waitKey(3)

    cv2.destroyAllWindows()
    vs.stop()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass