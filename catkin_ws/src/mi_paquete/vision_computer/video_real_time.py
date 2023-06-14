#!/usr/bin/env python3

import cv2 as cv

video_capture = cv.VideoCapture(0)
#video_capture = cv2.VideoCapture('video/ros.mp4')

while(True):
	ret, frame = video_capture.read()
	
	#frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
	#frame = cv.resize(frame, (0,0), fx=0.5,fy=0.5)
	cv.line(frame,(0,0),(511,511),(255,0,0),5)
	cv.imshow("Frame",frame)
	if cv.waitKey(1) & 0xFF == ord('q'):
		break

video_capture.release()
cv2.destroyAllWindows()