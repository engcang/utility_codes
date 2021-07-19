import cv2
import numpy as np
import time
import math


def img_to_8bit_img(bit_img):
    minVal = np.amin(bit_img); maxVal = np.amax(bit_img)
    it_img = bit_img - minVal
    it_img = it_img / float(maxVal-minVal) * 255.0
    return it_img.astype('uint8') 



cam = cv2.VideoCapture('/dev/video5')

#print(cam.get(cv2.CAP_PROP_FORMAT))
#print(cam.get(cv2.CAP_PROP_MODE))

cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
#cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y','1','6',' '))

cam.set(cv2.CAP_PROP_CONVERT_RGB, False)
cam.set(cv2.CAP_PROP_FORMAT,cv2.CV_16U)
cam.set(cv2.CAP_PROP_CONVERT_RGB, False)
cam.set(cv2.CAP_PROP_MODE, 3)  
#print(cam.get(cv2.CAP_PROP_FORMAT))
#print(cam.get(cv2.CAP_PROP_MODE))

time.sleep(1)
count = 0
width = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cam.get(cv2.CAP_PROP_FRAME_HEIGHT)

while True:
    _, frame = cam.read()
    count=count+1
    if count>10:
    #    print(np.shape(frame))
        frame_120160 = frame[:120,:]

        thermal_calib = 0.0217*(frame_120160-8192.0)+293.0-273.0
        gray_8bit = img_to_8bit_img(frame_120160)
        colored_img = cv2.applyColorMap(gray_8bit, cv2.COLORMAP_JET)
        colored_img2 = cv2.resize(colored_img,(int(3*width),int(3*height)),interpolation = cv2.INTER_CUBIC)

        #hot_y = np.argmax(thermal_calib)//160
        #hot_x = np.argmax(thermal_calib)%160
        #hot_temp = np.amax(thermal_calib)
        
        cv2.imshow('Thermal', colored_img2)
        if cv2.waitKey(1)==ord('q'):
            break
cam.release()
cv2.destroyAllWindows()
