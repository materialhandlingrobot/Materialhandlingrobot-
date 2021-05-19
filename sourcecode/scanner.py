from pyzbar import pyzbar
import datetime
import imutils
import time
import cv2

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
vs = cv2.VideoCapture(1)
time.sleep(2.0)
width = vs.get(cv2.CAP_PROP_FRAME_WIDTH)
height = vs.get(cv2.CAP_PROP_FRAME_HEIGHT)
print("{} and {}".format(width, height))
# loop over the frames from the video stream
while True:
	# grab the frame from the threaded video stream and resize it to
	# have a maximum width of 400 pixels
	rec, frame = vs.read()
	# frame = imutils.resize(frame, width=400)
	# find the barcodes in the frame and decode each of the barcodes
	barcodes = pyzbar.decode(frame)
	
	# loop over the detected barcodes
	for barcode in barcodes:
		# extract the bounding box location of the barcode and draw
		# the bounding box surrounding the barcode on the image
		(x, y, w, h) = barcode.rect
                print(height/2 - y - h/2)
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
		cv2.putText(frame, text, (x, y - 10),
			cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
	# show the output frame
	frame = imutils.resize(frame, width=400)
	cv2.imshow("Barcode Scanner", frame)
	key = cv2.waitKey(1) & 0xFF
 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
cv2.destroyAllWindows()
vs.stop()