#!/usr/bin/env python
#import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
from pylab import *
import numpy as np
from matplotlib import pyplot as plt
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
from keras.models import load_model

print("2")
def detect(data):
    print("4")
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    print("3")

    model = load_model('model.h5')

    alphabets = ['P', 'S', 'L', 'U', 'B', 'O', 'G', 'N', 'C', 'Y', 'K', 'C', 'M', 'F', 'I', 'H', 'A', 'T']
    img = cv2.imread(image)
    im2 = img.copy()
    alpha = 1.4
    beta = 50
    new_img = np.zeros(img.shape, img.dtype)
    for y in range(img.shape[0]):
        for x in range(img.shape[1]):
            new_img[y, x] = np.clip(alpha * img[y, x] + beta, 0, 255)

    gray = cv2.cvtColor(new_img, cv2.COLOR_BGR2GRAY)

    ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU | cv2.THRESH_BINARY_INV)
    rect_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
    dilation = cv2.dilate(thresh, rect_kernel, iterations=5)
    contours, hierarchy = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        if h < 100 and w < 100:
            continue
        rect = cv2.rectangle(im2, x, y, x + w, y + h)
        cropped = im2[y:y + h, x:x + w]
        cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        # cv2_imshow(cropped)
        cropped = cv2.resize(cropped, (128, 64))
        imgs = np.array(cropped).reshape(64, 128)
        img1 = imgs[:, :64]
        img2 = imgs[:, 64:]

        img1 = img1.reshape(1, 64, 64, 1)
        img2 = img2.reshape(1, 64, 64, 1)

        # cv2_imshow(img1.reshape(64, 64))
        # cv2_imshow(img2.reshape(64, 64))

        pred1 = model.predict(img1)
        pred2 = model.predict(img2)

        l1 = print(alphabets[np.argmax(pred1[0])])
        l2 = print(alphabets[np.argmax(pred2[0])])

    letters = l1 + l2
    country1 = "PUKISTAN"   #1
    country2 = "NUNNIGA"    #2
    country3 = "BULLICHYA"  #3

    if country1.find(letters) != -1:
        return 1
    elif country2.find(letters) != -1:
        return 2
    elif country3.find(letters) != -1:
        return 3
    else:
        print("SOMETHING IS WRONG")

    cv2.imshow("Image",image)
    print("aagaya")
    k = cv2.waitKey(5) & 0xFF

if __name__ == '__main__':
    
	rospy.init_node('image_gazebo', anonymous=True)
	rospy.Subscriber("/mybot/mybot/camera1/image_raw", Image, detect)
    
	rospy.spin()





