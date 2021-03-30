import cv2
import numpy as np
import time

cam = cv2.VideoCapture('/dev/video0')
print(cam.get(cv2.CAP_PROP_FORMAT))
print(cam.get(cv2.CAP_PROP_MODE))

cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
cam.set(cv2.CAP_PROP_CONVERT_RGB, False)

#cam.set(cv2.CAP_PROP_FORMAT,cv2.CV_16U)
#cam.set(cv2.CAP_PROP_CONVERT_RGB, False)
#cam.set(cv2.CAP_PROP_MODE, 3)  
print(cam.get(cv2.CAP_PROP_FORMAT))
print(cam.get(cv2.CAP_PROP_MODE))


time.sleep(1)
count = 0
while True:
    _, frame = cam.read()
    #print(np.shape(frame))
    count=count+1

    if count > 10 :
        #channel1=frame[:,:,0]
        #channel2=frame[:,:,1]
        #channel3=frame[:,:,2]
        #horizon_tmp = cv2.hconcat([channel1, channel2])
        #horizon = cv2.hconcat([horizon_tmp, channel3])
        minVal = np.amin(frame)
        maxVal = np.amax(frame)
        #print(frame[120:122,:])
        print(frame)
        img = frame[:120,:]
        #print(maxVal)
#        draw = cv2.convertScaleAbs(frame, alpha=255.0/(maxVal - minVal), beta=0)
        #draw = cv2.convertScaleAbs(frame, alpha=1, beta=0)
        draw = cv2.convertScaleAbs(img) # 8-bit to 16-bit converting
        #cv2.imshow('Thermal image URL', horizon)
        cv2.imshow('Thermal image URL', draw)

        #cv2.moveWindow('Thermal image URL',0,0)
        if cv2.waitKey(1)==ord('q'):
            break
cam.release()
cv2.destroyAllWindows()
