import cv2
import numpy as np
import time
import math


def img_to_8bit_img(bit_img):
    minVal = np.amin(bit_img); maxVal = np.amax(bit_img)
    it_img = bit_img - minVal
    it_img = it_img / float(maxVal-minVal) * 255.0
    return it_img.astype('uint8') 



#cam2 = cv2.VideoCapture('/dev/video1') 
cam = cv2.VideoCapture('/dev/video5')
#cam = cam2



#backtorgb = cv2.cvtColor(gray,cv2.COLOR_GRAY2RGB)



print(cam.get(cv2.CAP_PROP_FORMAT))
#print(cam.get(cv2.CAP_PROP_MODE))

cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
cam.set(cv2.CAP_PROP_CONVERT_RGB, False)

cam.set(cv2.CAP_PROP_FORMAT,cv2.CV_16U)
cam.set(cv2.CAP_PROP_CONVERT_RGB, False)
cam.set(cv2.CAP_PROP_MODE, 3)  
#print(cam.get(cv2.CAP_PROP_FORMAT))
#print(cam.get(cv2.CAP_PROP_MODE))





time.sleep(1)
count = 0

while True:
    _, frame = cam.read()
#    _, frame2 = cam2.read()

#    print(np.shape(frame))

    frame_120160 = frame[:120,:]

    count=count+1

    thermal_frame = frame[:120,:]
    color_temperature = frame[:120,:]

    thermal_calib = 0.0217*(thermal_frame-8192.0)+293.0-273.0

    gray_8bit = img_to_8bit_img(frame_120160)
    colored_img = cv2.applyColorMap(gray_8bit, cv2.COLORMAP_JET)

    cv2.imshow('Thermal', colored_img)

    if cv2.waitKey(1)==ord('q'):
        break
cam.release()
cv2.destroyAllWindows()
