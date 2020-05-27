#! /usr/bin/python

import cv2
import numpy as np
import serial
import time


class clMotorCmd():
    '''
    Controls the 2 motors over serial
    '''

    def __init__(self, port ='/dev/ttyUSB0', baud=9600):

        self.ser = serial.Serial(port, baudrate=baud, timeout=1)

    def setHome(self):
        '''
        send set home command
        :return:
        '''
        self.ser.write(b"sh,0" + b'\n')
        self.ser.write(b"sh,1" + b'\n')

    def goHome(self):
        '''
        send go home command
        :return:
        '''
        self.ser.write(b"gh,0" + b'\n')
        self.ser.write(b"gh,1" + b'\n')

    def guideMotor(self,velx,vely):
        '''
        Control the motors over the 2 axix.
        :param velx:
        :param vely:
        :return:
        '''
        if velx < 0:
            self.ser.write(b"m0p," + bytes(str(abs(int(velx / 10))), 'utf8') + b'\n')
        else:
            self.ser.write(b"m0m," + bytes(str(abs(int(velx / 10))), 'utf8') + b'\n')

        if vely > 0:
            self.ser.write(b"m1p," + bytes(str(abs(int(vely / 10))), 'utf8') + b'\n')
        else:
            self.ser.write(b"m1m," + bytes(str(abs(int(vely / 10))), 'utf8') + b'\n')

    def moveMotor(self, motor=0, dir=0, vel=5):
        '''
        Move selected motor
        :param motor: [0,1]
        :param dir: [0,1]
        :param vel: default 5
        :return:
        '''

        # test motor and direction
        if motor == 0:
            if dir == 0:
                self.ser.write(b"m0p," + bytes(str(abs(int(vel))), 'utf8') + b'\n')
            else:
                self.ser.write(b"m0m," + bytes(str(abs(int(vel))), 'utf8') + b'\n')
        else:
            if dir == 0:
                self.ser.write(b"m1p," + bytes(str(abs(int(vel))), 'utf8') + b'\n')
            else:
                self.ser.write(b"m1m," + bytes(str(abs(int(vel))), 'utf8') + b'\n')


class clKalman():
    def __init__(self):
        '''
        Init local variables and Kalman filter
        '''

        # 2d Kalman
        self.kalman = cv2.KalmanFilter(4, 2)

        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0],[0, 1, 0, 1],[0, 0, 1, 0], [0, 0, 0, 1]],np.float32)

        self.kalman.processNoiseCov = np.array([[1, 0, 0 ,0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]],np.float32) * 0.1

        # state variables
        self.last_measurement = np.array((2, 1), np.float32)
        self.current_measurement = np.array((2, 1), np.float32)
        self.last_prediction = np.zeros((2, 1), np.float32)
        self.current_prediction = np.zeros((2, 1), np.float32)

        # correction => take from measurement, add to correction
        self.xi = 0
        self.yi = 0


    def predictAndUpdate(self,x,y,correct=True):
        '''
        Makes the update and correction phase from Kalman
        :param x: first parameter to estimeate in a 2d space
        :param y: secound parameter to estimate in a 2d space
        :param correct: if active, measurement correction take place, otherwise just predict.
        Usefull if it is lost the measurement, then we can realy only on estimation.
        :return: last estimation, current estimate
        '''
        self.last_prediction = self.current_prediction
        self.last_measurement = self.current_measurement
        self.current_measurement = np.array([[np.float32(x-self.xi)], [np.float32(y-self.yi)]])

        # correct and predict, if is the case
        if correct:
            self.kalman.correct(self.current_measurement)
        self.current_prediction = self.kalman.predict()

        self.current_prediction = [self.current_prediction[0]+self.xi, self.current_prediction[1]+self.yi]

        return self.last_prediction, self.current_prediction

    def getStateVariables(self):
        '''
        Helper to return internal variables
        :return: last/current measurement values; last / current prediction values; array of (2,1)
        '''
        return self.last_measurement, self.current_measurement, self.last_prediction, self.current_prediction

    def init(self,x,y):
        '''
        State initialization
        :param x: init x value
        :param y: init y value
        :return:
        '''
        self.xi = x
        self.yi = y

class ContourDetector():
    def __init__(self):
        self.gimg = []
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.items = []
        self.count = 0

    def CotourFilter(self,img, area=1000.0):
        '''
        Used to filter the contours
        :param img: input image for detecting contours
        :param area: minimum aria to detect, skipp smaller ones
        :return: list of detected countours (ID, coordinates)
        '''

        self.items = []
        self.gimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        contours, hierarchy = cv2.findContours(self.gimg, 1, 2)

        id = 1
        for cnt in contours:
            larea = cv2.contourArea(cnt)
            if larea > area:

                x, y, w, h = cv2.boundingRect(cnt)

                center = (int(x+(w/2)), int(y+(h/2)))

                self.items.append(np.array([id, x, y, w, h, center]))

                id +=1
        return self.items

    def DrawDetections(self, img, detections, offset =20, objCenter=True,objRectangle=True, label=[0,0]):
        '''
        Draw detected contours on image
        :param img: input image
        :param detections: list of detected contours (ID, coordinates)
        :param offset: offset to increase detected area
        :param objCenter: draw object center
        :param objRectangle: draw object contour
        :param label: label name
        :return: labeled image
        '''
        # copy image
        limg = cv2.copyMakeBorder(img, 0, 0, 0, 0, cv2.BORDER_REPLICATE)

        # check if there are detections
        if len(detections) > 0:
            for ii,ll in zip(detections,label):
                id, x, y, w, h, center = ii
                if objCenter == True:
                    cv2.circle(limg, center, 2, (0, 255, 0), -1)
                if objRectangle == True:
                    cv2.rectangle(limg, (x-offset, y-offset), (x+w+offset, y+h+offset), (0, 255, 0), 1)
                    if ll[0] == id:
                        cv2.putText(limg, str(ll[1]), (x-offset+5, y-offset+15), self.font, 0.4, (0, 255, 0), 1, cv2.LINE_AA)

        return limg

    def GetRoiForDetections(self, img, detections, offset=20, roi_size=(96, 96)):
        '''
        Get patches from an image
        :param img: input image
        :param detections: list of detected ROIs
        :param offset: detection offset
        :param roi_size: ROI size
        :return: return image patches
        '''

        rois = []
        #check if there are detections
        if len(detections) > 0:
            for ii in detections:
                id, x, y, w, h, center = ii

                try:
                    detRoi = img[y - offset:y + h + offset, x - offset:x + w + offset]
                    detRoi = cv2.normalize(detRoi, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
                    detRoi = cv2.resize(detRoi, roi_size)

                    rois.append(detRoi)
                except:
                    pass
        return rois

    def ShowRoisOnImage(self, img, rois, roi_size=(96, 96)):
        '''
        Helper function, draw  ROIs on blank image
        :param img: imput image
        :param rois: ROIs
        :param roi_size: size
        :return: show roi on image
        '''

        if len(rois) > 0:
            offs = 0
            try:
                for ii in rois:
                    ii = np.uint8((ii + 1) * 255 / 2)
                    img[0:roi_size[0], 0 + offs: 0 + roi_size[0] + offs] = ii
                    offs += roi_size[0]
            except:
                pass


        return img

    def SaveImages(self,img, rois, path=".", initnumber=0, usetime=True,saveframe=False,prefix=""):
        '''
        Used to create training sets, save the detected ROIs, as image patches
        :param img: canera inage
        :param rois: detections
        :param path: location to save the patches
        :param initnumber: append a specific number to the image name
        :param usetime: append to image name the current time
        :return:
        '''
        time = datetime.now().strftime("%H%M%S")

        if len(rois) > 0:
            for ii in rois:
                ii = (ii + 1) * 255 / 2
                if usetime == True:
                    cv2.imwrite(path + "img_" + prefix + str(time) + ".png", ii)
                else:
                    cv2.imwrite(path + "img_" + prefix +str(self.count + initnumber) + ".png", ii)
                self.count +=1
        else:
            cv2.imwrite(path + "img_"+ prefix + str(time) + ".png", img)


###########################################################
# With help of this class the correct HSV values can be
# selected from a picture color space
# this values can be further used i.e. for skin detection.
# Track-bars will appear if debug mode is activated!
###########################################################
class clPreProcessing():
    def __init__(self,debug=True, h=122,s=22,v=0,hm=0, sm=0, vm=0):
        self.img = []
        self.debug = debug
        self.h = h
        self.s = s
        self.v = v
        self.hm = hm
        self.sm = sm
        self.vm = vm

        if self.debug == True:
            # create trackbars for color change
            cv2.createTrackbar('h', 'img', 0, 255, self.nothing)
            cv2.createTrackbar('s', 'img', 0, 255, self.nothing)
            cv2.createTrackbar('v', 'img', 0, 255, self.nothing)
            cv2.createTrackbar('hm', 'img', 0, 255, self.nothing)
            cv2.createTrackbar('sm', 'img', 0, 255, self.nothing)
            cv2.createTrackbar('vm', 'img', 0, 255, self.nothing)

    def nothing(self,x):
        pass

    ##################################
    # Make color space transformation
    ##################################
    def processImg(self, img,h=0,s=0,v=0,hm=0,sm=0,vm=0):

        if self.debug == True:
            self.h = cv2.getTrackbarPos('h', 'img')
            self.s = cv2.getTrackbarPos('s', 'img')
            self.v = cv2.getTrackbarPos('v', 'img')
            self.hm = cv2.getTrackbarPos('hm', 'img')
            self.sm = cv2.getTrackbarPos('sm', 'img')
            self.vm = cv2.getTrackbarPos('vm', 'img')

        # snoth the image
        self.img = cv2.medianBlur(img, 7)
        self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2YCR_CB)

        # set color space domain
        lower_color = np.array([h, s, v])
        upper_color = np.array([hm, sm, vm])

        # filter HSV image
        mask = cv2.inRange(self.img, lower_color, upper_color)

        # apply mask on the original image
        self.img = cv2.bitwise_and(img, img, mask=mask)

        return self.img

class clPIDController():
    '''
    PID controller
    '''

    def __init__(self, Kp, Ki, Kd, limitOut=None):
        '''
        Init parameters
        :param Kp: constat for proportional part
        :param Ki: constant for integrative part
        :param Kd: constant for derivative part
        :param limitOut: use output limitation (will reduce output range to [-limitOut,limitOut])
        '''

        self.time = 0
        self.prevTime = 0
        self.prevError = 0

        # PID Gains
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # cummulative error
        self.cummError = 0.0

        # limit pid value
        self.limitOut = limitOut

    def Correct(self, target, current, dtin=None):
        '''
        Implement PID corrections
        :param target: Set Value to be reached
        :param current: Sendor value
        :param dtin: use constant or calculated time
        :return: cirrected PID value
        '''

        error = target - current

        now = time.time()

        # compute delta t
        if dtin == None:
            # update time
            dt = now - self.prevTime
        else:
            dt = dtin

        # avoid division by zero
        if dt != 0:
            # integration term
            self.cummError += error * dt

            # derivativ term
            de = (error - self.prevError) / dt

        # update previous values
        self.prevError = error
        if dtin is None:
            self.prevTime = now
        else:
            self.prevTime = dtin

        out = self.Kp * error + self.Ki * self.cummError + self.Kd * de

        # use output limitation
        if self.limitOut is not None:
            # solve for negative values
            if out > 0:
                out = max(min(self.limitOut, out), 0)
            else:
                out = - max(min(self.limitOut, abs(out)), 0)

        # compute output
        return out

