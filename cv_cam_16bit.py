import cv2
import numpy as np
import time

def img_to_8bit_img(bit_img):
    minVal = np.amin(bit_img); maxVal = np.amax(bit_img)
    it_img = bit_img - minVal
    it_img = it_img / float(maxVal-minVal) * 255.0 #### bing shin
    return it_img.astype('uint8') 
#   it_img = it_img / float(maxVal-minVal) #### bing shin
#   return it_img


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
        #horizon_tmp = cv2.hconcat([channel1, channel2]); horizon = cv2.hconcat([horizon_tmp, channel3])
        minVal = np.amin(frame); maxVal = np.amax(frame)
        
        img = frame[:120,:]

        thermal_calib = 0.0217*(frame_120160-8192.0)+293.0-273.0
        gray_8bit = img_to_8bit_img(img)
        colored_img = cv2.applyColorMap(gray_8bit, cv2.COLORMAP_JET)
        
        #draw = cv2.convertScaleAbs(frame, alpha=255.0/(maxVal - minVal), beta=0)
        #draw = cv2.convertScaleAbs(frame, alpha=1, beta=0)
        #draw = cv2.convertScaleAbs(img) # 16-bit to 8-bit converting
        #cv2.imshow('Thermal image URL', horizon)
        #cv2.imshow('Thermal image URL', draw)
        cv2.imshow('Thermal image URL', colored_img)

        if cv2.waitKey(1)==ord('q'):
            break
cam.release()
cv2.destroyAllWindows()
