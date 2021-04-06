import cv2
import os

folder = '/home/cps/images/19/'
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4', fourcc, 30.0, (640,480))

files = os.listdir(folder)
files.sort()
for filename in files:
    img = cv2.imread(os.path.join(folder,filename))
    frame = cv2.resize(img, dsize=(640, 480), interpolation=cv2.INTER_AREA) #image should be in size same with out video

    out.write(frame)
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release everything if job is finished
out.release()
cv2.destroyAllWindows()
