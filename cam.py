import cv2
import time

cam = cv2.VideoCapture(0)
t
# Capture image every second
while True:
    ret, image = cam.read()

    cv2.imshow('Imagetest',image)
    cv2.imwrite('/home/tsc06/acrp-project/images/testimage.jpg', image)

    k = cv2.waitKey(1)
    if k != -1:
        break
    time.sleep(1)
cam.release()
cv2.destroyAllWindows()
