#! /usr/bin/env python2

import cv2
from cv2 import aruco
import numpy as np
import rospy
import mavros
from mavros_msgs.msg import PositionTarget as PT



class MarkerDetector():
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.mtx = np.array([1.537822710187638449e+03, 0.0, 6.404980296570619203e+02, 0.0, 1.418298704599976645e+03, 2.406765310346528963e+02, 0.0, 0.0, 1.0]).reshape(3,3)
        self.dist = np.array([-3.479798240858279768e-02, 6.107234468717099851e-01, -1.615287344933384606e-02, -2.327112969541987163e-03,-2.305669941088070551e+00])
        self.dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.param = cv2.aruco.DetectorParameters_create()
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        #self.rate = rospy.Rate(100)
        self.aruco_marker_pos_pub = rospy.Publisher('/aruco_marker_pos', PT, queue_size=10)
        rospy.init_node('marker_detector')
        mavros.set_namespace('mavros')

    def marker_pose(self):
        # rospy.init_node("pub_pos", anonymous=True)
        # rate = rospy.Rate(10)
        self.cap.set(3, 1280)
        self.cap.set(4, 720)
        # set dictionary size depending on the aruco marker selected
        self.param.adaptiveThreshConstant = 7

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            # operations on the frame
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

            # lists of ids and the corners belonging to each id
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.dict, parameters=self.param)

            if np.all(ids is not None):
                ret1 = aruco.estimatePoseSingleMarkers(corners=corners, markerLength=0.1,
                                                       cameraMatrix=self.mtx, distCoeffs=self.dist)
                rvec, tvec = ret1[0][0, 0, :], ret1[1][0, 0, :]
                # -- Draw the detected marker and put a reference frame over it
                aruco.drawDetectedMarkers(frame, corners, ids)
                aruco.drawAxis(frame, self.mtx, self.dist, rvec, tvec, 0.1)
                str_position0 = "Marker Position in Camera frame: x=%f  y=%f  z=%f" % (tvec[0], tvec[1], tvec[2])
                cv2.putText(frame, str_position0, (0, 50), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                (rvec - tvec).any()  # get rid of that nasty numpy value array error

                # self.pub.publish(marker_position[0], marker_position[1], marker_position[2])
                marker_pos = PT()
                marker_pos.position.x = tvec[1]
                marker_pos.position.y = tvec[0]
                marker_pos.position.z = -tvec[2]
                self.aruco_marker_pos_pub.publish(marker_pos)

            cv2.imshow("frame", frame)
            cv2.waitKey(1)
            #self.rate.sleep()


if __name__ == '__main__':
    MD = MarkerDetector()
    try:
        MD.marker_pose()
    except rospy.ROSInterruptException:
        pass
