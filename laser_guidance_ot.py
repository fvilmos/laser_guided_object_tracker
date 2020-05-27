#! /usr/bin/python
import cv2
import numpy as np
import sys
import argparse
import time

# import the utility classes
from utils import clKalman
from utils import  ContourDetector
from utils import clPreProcessing
from utils import clMotorCmd

#define image dimensions
IMG_WIDTH = 640
IMG_HEIGHT = 480

# Camera ID
CAMID = 0

# go Home threshold limit
TH_LIMIT = 5

# create named window, set position
cv2.namedWindow('img',2)
cv2.moveWindow('img',0,0)

# main loop
if __name__ == "__main__":

    # process cmd line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-cam', type=int, required=True, metavar='camID', default=0, help='Camera ID, default 0')

    args = parser.parse_args()

    # create cam instance
    CAMID = args.cam
    cam0 = cv2.VideoCapture(CAMID)

    # resize, to spare CPU load
    cam0.set(3,IMG_WIDTH)
    cam0.set(4,IMG_HEIGHT)

    # used to detect contours
    cd = ContourDetector()

    # Kalman estimator for guidance
    kf = clKalman()

    # create preprocessing class
    objPP = clPreProcessing(False)

    # create motors control class
    mctrl = clMotorCmd('/dev/ttyUSB0')

    # used as simple frame counter
    count = 1

    # used to initialize the system
    process = False

    # threshold for laser pointer lost and go home
    waithThreshold = 0

    while True:

        # increment counter in every 10 cycles
        if (count % 10) == 0:
            waithThreshold +=1

        #grab a frame
        _, img0 = cam0.read()

        # test cam instance
        if cam0:

            # get laser pointer position
            img1 = objPP.processImg(img0,255,50,105,255,255,255)
            ret = cd.CotourFilter(img1,10.0)
            ret = np.array(ret)

            # get ball position
            imgb = objPP.processImg(img0,10,108,136,62,142,255)
            retb = cd.CotourFilter(imgb, 20.0)
            retb = np.array(retb)

            # Laser pointer detected
            if len(ret) > 0 :
                x = ret[0][5][0]
                y = ret[0][5][1]
                cv2.circle(img0, (x, y), 5, (0, 255, 0), -1)

                #record initial position, for laser homing
                if process == False:
                    mctrl.setHome()
                    kf.init (x,y)

                    # process once
                    process = True

                #reset threshold
                waithThreshold = 0

            else:
                # Laser pointer lost, go home if threshold reached
                if waithThreshold >=TH_LIMIT:

                    #no detections for a while go home
                    mctrl.goHome()
                    waithThreshold = 0

            # ball detected, laser pointer too
            if len(retb) >0 and  len(ret) >0:

                # ball position
                xb = retb[0][5][0]
                yb = retb[0][5][1]

                # show a rectangle on the detection
                cv2.rectangle(img0,(xb-10,yb-10),(xb+10,yb+10),(0,255,0),1)

                # predict new position with Kalman
                lp, cp = kf.predictAndUpdate(xb, yb, True)

                # compute offset between Laser pointer and the ball, use as velocity
                vx = int(cp[0]-x)
                vy = int(cp[1]-y)

                mctrl.guideMotor(vx,vy)


            cv2.imshow('img', img0)

        # quit on keypress
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
          break

        # motor manual control, use to set initial position
        if k == ord('a'):
            mctrl.moveMotor(0,0,5)
        if k == ord('d'):
            mctrl.moveMotor(0,1,5)
        if k == ord('s'):
            mctrl.moveMotor(1,0,5)
        if k == ord('w'):
            mctrl.moveMotor(1,1,5)
        if k == ord('h'):
            mctrl.goHome()

        count +=1

    # release cam
    cam0.release()
    cv2.destroyAllWindows()
