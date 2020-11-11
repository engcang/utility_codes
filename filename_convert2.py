import cv2
import os

def convert_images_from_folder(folder):
	f = open("/home/hcshim/Desktop/devkit/timestamp.txt",'w')
	tmp=[]
	for filename in os.listdir(folder):
		tmp.append(filename[:13]+'\n')
	tmp.sort()
	for i in range(len(tmp)):
		f.write(tmp[i])
	f.close()

convert_images_from_folder("/home/hcshim/Desktop/devkit/cam0")
