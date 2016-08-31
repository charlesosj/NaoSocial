# !/usr/bin/env python
import rospy
import cv2
import sys
import roslib
from naoqi import *
# from rospy.exceptions import ROSException

import actionlib
import threading
import os
import numpy
import time
from std_msgs.msg import String,Bool, Float32MultiArray
from random import randint


class NaoSocial:
    def __init__(self):

        self.habituation = 0
        self.speechCount =0
        self.undetectedcount=0
        self.detected = False
        self.ini = False
        self.searching = False
        self.state ='nothing'
      #  self.motionProxy = ALProxy("ALMotion", '10.18.12.56', 9559)
  
        #publisher
        self.behaviorpub = rospy.Publisher('/nao_behavior/add/nonblocking', String, queue_size=5, latch =True)

        self.enableDiagpub = rospy.Publisher('/nao_behavior/enable_Diag', String,latch=True ,queue_size=5)
        self.disableDiagpub = rospy.Publisher('/nao_behavior/reset_Diag', String,latch=True ,queue_size=5)

        #subscriber
        self.tracking = rospy.Subscriber("/nao_behavior/tracking", Bool, self.trackingC)


        #enable tracking
    	#start awarenss
        self.behaviorpub.publish("aware")
        self.look_away_timer = 30

    def run(self):
        while True and not rospy.is_shutdown():
           
            print self.habituation ,self.undetectedcount
            if self.habituation == 0 and self.detected == True:
                self.undetectedcount = 0
                self.hello()
                #enable diag
                self.enableDiagpub.publish("sdsd")

            if self.detected == True:
                # if we have a person detected
                self.habituation += 1
                self.undetectedcount =0

                # if we have been looking at a person for a while
                #unregister them
                if self.habituation % self.look_away_timer == 0:
                    print 'Changing Target'
                    self.behaviorpub.publish('changetarget')
					#put habituationto 1 so we dont wave at the next person
                    self.habituation =1
            else:
                self.undetectedcount +=1

            #if we havevent detected a person in a while exit and reset
            if self.undetectedcount >500:
                print 'reset'

                self.habituation = 0
                self.speechCount = 0
                self.undetectedcount= 0
                #disable dialogs
                self.disableDiagpub.publish("msg")
                self.ini = False
                return    
            time.sleep(0.5)
    def trackingC(self, data):
        self.detected = data.data

        if self.ini == False and data.data == True:
            self.ini =True
            t1 = threading.Thread(target=self.run)
            t1.start()
            #self.run()
    def hello(self):
        str = 'System/animations/Stand/Gestures/Hey_1'
        #self.behaviorpub.publish(str)
        str = 'say Hello'
        #self.behaviorpub.publish(str)


rospy.init_node('NaoSocial', anonymous=True)
app = NaoSocial()
#app.run()

rospy.spin()




