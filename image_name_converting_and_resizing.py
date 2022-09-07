import cv2
import os


def convert_images_from_folder(folder):
    count=0
    for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder,filename))
        if img is not None:
            resized = cv2.resize(img, dsize=(640,480), interpolation=cv2.INTER_LINEAR)
            cv2.imwrite(folder+'_resized/'+filename, resized)
            count = count+1

convert_images_from_folder("./train4")
