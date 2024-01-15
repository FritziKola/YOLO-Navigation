#!/usr/bin/env python3
import cv2
import rospy
from rumex_detection import rumex_detection


rospy.init_node("test", anonymous=True)
detect = rumex_detection()
cv2.namedWindow("Camera", flags=cv2.WINDOW_GUI_NORMAL)

while not rospy.is_shutdown():
    try:
        cv2.imshow("Camera", detect.img_pred)
    except:
        pass
    cv2.waitKey(10)
cv2.destroyAllWindows()