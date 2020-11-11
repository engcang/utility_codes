import cv2
print(cv2._version__)

'''using Gstreamer'''
#for NVargus
#camSet = 'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=3264, height=2464, framerate=21/1, format=NV12' ! nvvidconv flip-method=2 ! video/x-raw, width=800, height=600, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink' #flip-method=0 or 2

#wbmode : white balance, 0:off, 1:auto(default)
#tnr-mode : noise reduction mode, 0:off, 1:Fast(default), 2:High Quality
#ee-mode : edge enhancement mode, 0:off, 1:Fast(def), 2:High Qual
#contrast 0~2, brightness-1~1, saturation0~2
#camSet = 'nvarguscamerasrc sensor-id=0 tnr-mode=2 wbmode=3 ! video/x-raw(memory:NVMM), width=3264, height=2464, framerate=21/1, format=NV12' ! nvvidconv flip-method=2 ! video/x-raw, width=800, height=600, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! videobalance contrast=1.5 brightness=-.15 saturation=1.2 ! appsink' #flip-method=0 or 2

#cam = cv2.VideoCapture(camSet)

#for Webcam
#camSet = 'v4l2src device=/dev/video0 ! video/x-raw, width=800, height=600, framerate=24/1 ! videoconvert ! appsink'
#cam = cv2.VideoCapture(camSet)

#when using other codec
''' test-> v4l2-ctl -d /dev/video1 --list-formats-ext '''
# cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'H264'))
# cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
# cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920);
# cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080);
# cap.set(cv2.CAP_PROP_FPS, 30);

cam = cv2.VideoCapture('/dev/video0')
while True:
	_, frame = cam.read()
	cv2.imshow('pi Cam on Xavier-NX', frame)
	cv2.moveWindow('pi Cam on Xavier-NX', 0,0)
	if cv2.waitKey(1)==ord('q'):
		break
cam.release()
cv2.destroyAllWindows()