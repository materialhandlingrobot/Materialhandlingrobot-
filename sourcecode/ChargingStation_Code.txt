#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16

from pyzbar import pyzbar
import imutils
import cv2
import enum


class Cmd(enum.Enum):
    N = 2
    S = 8
    E = 6
    W = 4
    X = 5
    CW = 10
    CCW = 11

shouldMove = True
distance = 0.0
prevDiff = 1000.0
prevCmd = Cmd.N
arduPub = rospy.Publisher('/blinkm', Int16, queue_size=10)

def move(cmd):
    print(cmd)
    global arduPub, prevCmd
    prevCmd = cmd
    arduPub.publish(cmd.value)

def controller(diff):
    global distance, prevCmd, prevDiff, shouldMove
    if(distance > 1.02 and shouldMove and diff != 640):
        if(abs(diff) < 100):
            move(Cmd.N)
        elif(diff < -100):
            #if(prevCmd == Cmd.E and abs(prevDiff) < abs(diff)):
            move(Cmd.CW)
            rospy.sleep(0.5)
            move(Cmd.X)
            #else:
            #   move(Cmd.E)
        elif(diff > 100):
            #if(prevCmd == Cmd.W and abs(prevDiff) < abs(diff)):
            move(Cmd.CCW)
            rospy.sleep(0.5)
            move(Cmd.X)
            #else:
            #    move(Cmd.W)
        #elif(abs(diff) > 500):
        #    move(Cmd.X)
    elif(distance < 1.02):
        move(Cmd.X)
        shouldMove = False
    elif(diff == 640):
        move(Cmd.CW)
        rospy.sleep(0.2)
        move(Cmd.X)
        rospy.sleep(1)
    prevDiff = diff

def scanner(frame, width, height):
    # find the barcodes in the frame and decode each of the barcodes
    barcodeCenter = 0
    barcodes = pyzbar.decode(frame)
    # loop over the detected barcodes
    for barcode in barcodes:
        # extract the bounding box location of the barcode and draw
        # the bounding box surrounding the barcode on the image
        (x, y, w, h) = barcode.rect
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        # draw point
        cv2.circle(frame, (int(x + w/2),int(y + h/2)), radius=10, color=(0, 0, 255), thickness=-1)
        cv2.circle(frame, (int(width/2),int(height/2)), radius=10, color=(0, 255, 0), thickness=-1)
        barcodeCenter = int(x + w/2)
        # the barcode data is a bytes object so if we want to draw it
        # on our output image we need to convert it to a string first
        barcodeData = barcode.data.decode("utf-8")
        barcodeType = barcode.type
        # draw the barcode data and barcode type on the image
        text = "{} ({})".format(barcodeData, barcodeType)
        cv2.putText(frame, text, (x, y - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        # cv2.imshow("image", imutils.resize(frame, width=400))
        # cv2.waitKey(1)
    return (frame, barcodeCenter)

def laserCallback(msg):
    global distance
    pointCloudSize = len(msg.ranges)
    # for dist in msg.ranges[pointCloudSize-3:pointCloudSize+3]:
    #     distance += dist
    temp = msg.ranges[pointCloudSize/2]
    if(temp <= 10 and temp >= 0.2):
        distance = temp
    print('distance is {}'.format(distance))
    
def init():
    rospy.init_node('control')
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
        newF, barcodeCenter = scanner(frame, width, height)
        print('Pixel diff is {}'.format(width/2 - barcodeCenter))
        controller(width/2 - barcodeCenter)
        cv2.imshow("image", imutils.resize(newF, width=400))
        cv2.waitKey(1)
        # cmd = input("Enter Direction: \n")
        # arduPub.publish(int(cmd))
        # rospy.sleep(2)
	    # arduPub.publish(5)
    cv2.destroyAllWindows()
    vs.stop()
    move(Cmd.X)

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass