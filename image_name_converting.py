import cv2
import os

def convert_images_from_folder(folder):
    for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder,filename))
        if img is not None:
#			cv2.imwrite(filename[5:])
#			cv2.imshow("test",img)
#			cv2.waitKey(1)
#			print(filename[5:])
			cv2.imwrite(filename[5:15]+'000.png',img,[cv2.IMWRITE_PNG_COMPRESSION,0])

convert_images_from_folder("/home/hcshim/Desktop/devkit/before_cam0")
