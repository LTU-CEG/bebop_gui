#!/usr/bin/python
import cv2
import sys
from PyQt5.QtWidgets import  QWidget, QLabel, QApplication, QPushButton, QListWidget, QProgressBar, QStatusBar, QTabWidget, QTableWidget, QTableWidgetItem, QVBoxLayout, QMainWindow, QAbstractItemView, QLineEdit, QShortcut, QMessageBox
from PyQt5.QtCore import QThread, Qt, pyqtSignal, pyqtSlot, QObject
from PyQt5.QtGui import QImage, QPixmap, QKeySequence, QDoubleValidator
import rospy, rostopic
from sensor_msgs.msg import CompressedImage, Image, LaserScan
import numpy as np
from numpy import inf
from cv_bridge import CvBridge
import logging
from geometry_msgs.msg import Vector3Stamped, TwistStamped, Twist
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import Imu, Temperature
from mav_msgs.msg import Status, RollPitchYawrateThrust
from math import *
import subprocess, shlex
import signal
import os
import message_filters
import rosbag
from std_msgs.msg import Int32, String, Empty
import unicodedata
import time
from nav_msgs.msg import Odometry
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged
import threading
##################################################################

class App(QWidget):
    changeQImage = pyqtSignal(QImage) #Creating a signal
    changeLog = pyqtSignal(Log) #Creating a signal
    changeBatteryBar = pyqtSignal(CommonCommonStateBatteryStateChanged) #Creating a siganl

    checkHz = pyqtSignal(list)
    def __init__(self):
        super(QWidget, self).__init__()
        self.velocityPub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=10)
        self.takeoffPub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        self.landPub = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        self.goFwrd_vel = Twist()
        self.goBwrd_vel = Twist()
        self.recording = False #creating variable that keeps record if we're recording
        self.logCount = 0 #count for adding right amount of rows to Log
        self.count5 = 0 #count so we only print one ready/error takeoff msg per takeoff
        #subscriber to battery, Image, Log and LaserScan
        self.batterySub = rospy.Subscriber("/bebop/states/common/CommonState/BatteryStateChanged", CommonCommonStateBatteryStateChanged, self.batteryCallback, queue_size=10)
        self.imgSub = rospy.Subscriber("/bebop/image_raw", Image, self.imageCallback, queue_size = 10)
        self.logSub = rospy.Subscriber("/rosout_agg", Log, self.logCallback, queue_size = 10)
        self.recording = False
        threading.Timer(0.1, self.hzFunk).start()
        self.initUI()
##-------------------------------------------------------------------------------------------------------
    @pyqtSlot(QImage)
    def setImage(self, image):
        self.imageLabel.setPixmap(QPixmap.fromImage(image)) #Setting QImage in imageLabel

    @pyqtSlot(Log)
    def setLog(self, log):
        #Setting table
        self.logger.setItem(self.logCount,0, QTableWidgetItem(log.msg)) #printing msg to table
        self.logger.setItem(self.logCount,1, QTableWidgetItem(str(log.level))) #printing error lever table
        self.logger.setItem(self.logCount,2, QTableWidgetItem(log.name)) #printing node-name to table
        self.logger.setItem(self.logCount,3, QTableWidgetItem(str(log.header.stamp))) #printing timestamp to table
        self.logger.setRowCount(4+self.logCount) #adding new row
        self.logCount+=1

    @pyqtSlot(CommonCommonStateBatteryStateChanged)
    def setBatteyBar(self, status):
        self.batteryBar.setValue(status.percent) # setting battery percent

    @pyqtSlot(list)
    def getHz(self, list):
        self.hzLogger.setItem(0,0, QTableWidgetItem(str(list[0])))
        self.hzLogger.setItem(1,0, QTableWidgetItem(str(list[1])))
        self.hzLogger.setItem(2,0, QTableWidgetItem(str(list[2])))
        try:
            if  float(self.hzLogger.item(0,0).text()) < float(self.hzLogger.item(0,1).text()):
                self.hzLogger.item(0,0).setBackground(QColor(250,0,0))
        except:
            pass
        try:
            if  float(self.hzLogger.item(1,0).text()) < float(self.hzLogger.item(1,1).text()):
                self.hzLogger.item(1,0).setBackground(QColor(250,0,0))
        except:
            pass
        try:
            if  float(self.hzLogger.item(2,0).text()) < float(self.hzLogger.item(2,1).text()):
                self.hzLogger.item(2,0).setBackground(QColor(250,0,0))
        except:
            pass
        #Callback for QImage
    def imageCallback(self, data):
        bridge = CvBridge()
        rgbImage = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough") #converting rosimage to cvimage
        h, w, ch = rgbImage.shape
        bytesPerLine = ch * w
        convertToQtFormat = QImage(rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888) #converting cvimage to QImage
        p = convertToQtFormat.scaled(640, 480, Qt.KeepAspectRatio)
        self.changeQImage.emit(p) #emitting QImage through changeQImage-signal to setImage
        #Callback for Log
    def logCallback(self, log):
        self.changeLog.emit(log) #emitting rosout_agg-info through changeLog-signal to setLog
        #Callback for battery
    def batteryCallback(self, status):
        self.changeBatteryBar.emit(status) #emitting battarystatus through changeBatteryBar-signal to setBatteyBar
##------------------------------------------------------------------------------------------------------
    def takeOff(self):
        ret = self.checkRecStatus()
        if ret == 1048576 or ret == None: #number for ignore-action
            self.takeoffPub.publish()
        else:
            pass

    def land(self):
        self.landPub.publish()

    def goFwrd(self):
        self.goFwrd_vel.linear.x = +1
        self.velocityPub.publish(self.goFwrd_vel)

    def goBwrd(self):
        self.goBwrd_vel.linear.x = -1
        self.velocityPub.publish(self.goBwrd_vel)

    def turnRight(self):
        self.turnRight = Twist()
        self.turnRight.angular.z = -0.5
        self.velocityPub.publish(self.turnRight)

    def turnLeft(self):
        self.turnLeft = Twist()
        self.turnLeft.angular.z = +0.5
        self.velocityPub.publish(self.turnLeft)

    def hover(self):
        self.hover = Twist()
        self.hover.linear.x = 0
        self.hover.linear.y = 0
        self.hover.linear.z = 0
        self.hover.angular.z = 0
        self.velocityPub.publish(self.hover)

    def hzFunk(self):
        a = rostopic.ROSTopicHz(-1)
        b = rostopic.ROSTopicHz(-1)
        c = rostopic.ROSTopicHz(-1)
        rospy.Subscriber("/bebop/cmd_vel", Twist, a.callback_hz)
        rospy.Subscriber("/bebop/odom", Odometry, b.callback_hz)
        rospy.Subscriber("/bebop/image_raw", Image, c.callback_hz)
        time.sleep(1.0)
        list = []
        try:
            list.append(a.get_hz()[0])
        except:
            list.append("No message")
        try:
            list.append(b.get_hz()[0])
        except:
            list.append("No message")
        try:
            list.append(c.get_hz()[0])
        except:
            list.append("No message")
        self.checkHz.emit(list)

    def checkRecStatus(self):
        if self.recording == False:
            ret = self.recMessageBox("You are not recording!")
            return ret
        else:
            pass
##------------------------------------------------------------------------------------------------------------
    #FUNCTIONS FOR RECORDING
    def rec_funk(self):
        #if we're not already recording--> changing look of rec btn, setting rec to true and
        #setting selected topics to list of all selected topics in topicLogger.
        #Else --> end recording
        if self.recording == False:
            self.rec_btn.setStyleSheet("border-radius :40; background-color: green; border : 2px solid darkgreen;font-size: 30px;font-family: Arial")
            self.recording = True
            selectedTopics = self.topicLogger.selectedItems()
            # if at least one topic is selected--> record trem. Else --> record all topics.
            if len(selectedTopics)>0:
                command = ""
                for i in selectedTopics:
                    command = command + " " + i.text()
                command = shlex.split("rosbag record " + command)
                self.rosbag_record = subprocess.Popen(command)
            else:
                command = shlex.split("rosbag record -a")
                self.rosbag_record = subprocess.Popen(command)
        else:
            list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
            list_output = list_cmd.stdout.read()
            retcode = list_cmd.wait()
            assert retcode == 0, "List command returned %d" % retcode
            for string in list_output.split("\n"):
                if (string.startswith("/record")):
                    os.system("rosnode kill " + string)
            self.rec_btn.setStyleSheet("border-radius :40; background-color: red; border : 2px solid darkred;font-size: 30px;font-family: Arial")
            self.topicLogger.clearSelection()
            self.recording = False
##-----------------------------------------------------------------------------------------------------------
    def initUI(self):
        #create mainwindow
        self.setWindowTitle("ROS usb_cam")
        self.setGeometry(250, 100, 1100, 687)
        #creating edge label2
        self.edgeLabel = QLabel(self)
        self.edgeLabel.setStyleSheet("border: 1px solid grey; background-color: #CCCCCC")
        self.edgeLabel.move(650, 65)
        self.edgeLabel.resize(445, 358)
        #create a Image Label
        self.imageLabel = QLabel(self)
        self.imageLabel.move(4, 4)
        self.imageLabel.resize(640, 480)
        #create QTableWidget for logger
        self.logger =QTableWidget(self)
        self.logger.resize(780,180)
        self.logger.setRowCount(6)
        self.logger.setColumnCount(4)
        self.logger.setColumnWidth(0, 400)
        self.logger.setColumnWidth(1, 80)
        self.logger.setColumnWidth(2, 158)
        self.logger.setHorizontalHeaderLabels(("Message;Severity;Node;Timestamp").split(";"))
        #create QTableWidget for topics
        self.topicLogger =QTableWidget(self)
        self.topicLogger.resize(780,180)
        self.topicLogger.setRowCount(45)
        self.topicLogger.setColumnCount(2)
        self.topicLogger.setColumnWidth(0, 400)
        self.topicLogger.setColumnWidth(1, 380)
        self.topicLogger.setHorizontalHeaderLabels(("Topic;Msgs").split(";"))
        self.topicLogger.setSelectionMode(QAbstractItemView.MultiSelection)
        topics = rospy.get_published_topics(namespace='/')
        count2 = 0
        for i in topics:
            self.topicLogger.setItem(count2,0, QTableWidgetItem(i[0]))
            self.topicLogger.setItem(count2,1, QTableWidgetItem(i[1]))
            count2 +=1
        #creating hz-sensor logger
        self.hzLogger =QTableWidget(self)
        self.hzLogger.resize(780,180)
        self.hzLogger.setRowCount(3)
        self.hzLogger.setColumnCount(2)
        self.hzLogger.setColumnWidth(0, 500)
        self.hzLogger.setVerticalHeaderLabels(("Velocity commands;Odometry;Image").split(";"))
        self.hzLogger.setHorizontalHeaderLabels(("Frequencies;Preferd min").split(";"))
        #creating tabWidget to insert logger osv
        self.tabWidget = QTabWidget(self)
        self.tabWidget.move(310, 440)
        self.tabWidget.resize(780,240)
        self.tabWidget.addTab(self.logger, "rosout_agg info")
        self.tabWidget.addTab(self.topicLogger, "Topics")
        self.tabWidget.addTab(self.hzLogger, "Topic frequencies")
        #create push button - deactivate
        self.takeoff_btn = QPushButton('Takeoff', self)
        self.takeoff_btn.move(700,100)
        self.takeoff_btn.clicked.connect(self.takeOff)
        #create push button - activate
        self.land_btn = QPushButton('Land', self)
        self.land_btn.move(800,100)
        self.land_btn.clicked.connect(self.land)
        #create push button - go forward
        self.goFwrd_btn = QPushButton('Go forward', self)
        self.goFwrd_btn.move(830,200)
        self.goFwrd_btn.clicked.connect(self.goFwrd)
        #create push button - go backward
        self.goBwrd_btn = QPushButton('Go backward', self)
        self.goBwrd_btn.move(830,300)
        self.goBwrd_btn.clicked.connect(self.goBwrd)
        #create push button - turn right
        self.turnRight_btn = QPushButton('Turn right', self)
        self.turnRight_btn.move(910,250)
        self.turnRight_btn.clicked.connect(self.turnRight)
        #create push button - turn left
        self.turnLeft_btn = QPushButton('Turn left', self)
        self.turnLeft_btn.move(750,250)
        self.turnLeft_btn.clicked.connect(self.turnLeft)
        #create push button - hover
        self.hover_btn = QPushButton('HOVER', self)
        self.hover_btn.move(700,140)
        self.hover_btn.clicked.connect(self.hover)
        #create push button - recording
        self.rec_btn = QPushButton('REC', self)
        self.rec_btn.setGeometry(1000, 85, 80, 80)
        self.rec_btn.setStyleSheet("border-radius :40; background-color: red; border : 2px solid darkred;font-size: 30px;font-family: Arial")
        self.rec_btn.clicked.connect(self.rec_funk)
        #creating LTU logo label
        self.label_ltu_image = QLabel(self)
        pixmap_ltu_image = QPixmap("./image/LTU.png")
        pixmap_ltu_image = pixmap_ltu_image.scaled(200, 200, Qt.KeepAspectRatio, Qt.FastTransformation)
        self.label_ltu_image.setPixmap(pixmap_ltu_image)
        self.label_ltu_image.move(4, 450)
        #creating robotics team logo label
        self.robteamImage = QLabel(self)
        robteamPixmap = QPixmap("./image/robteam.png")
        robteamPixmap = robteamPixmap.scaled(300, 300, Qt.KeepAspectRatio, Qt.FastTransformation)
        self.robteamImage.setPixmap(robteamPixmap)
        self.robteamImage.move(4, 600)
        # #Creating battery label and bar
        self.batteryLabel = QLabel("Battery", self)
        self.batteryLabel.move(5, 0)
        self.batteryLabel.resize(50, 20)
        self.batteryBar = QProgressBar(self)
        self.batteryBar.move(5, 25)
        self.batteryBar.resize(150, 30)
        self.batteryBar.setMaximum(100)
        self.show() #showing GUI
        #creating different keybord shortcuts
        self.shortcut_fwrd = QShortcut(QKeySequence("w"), self)
        self.shortcut_fwrd.activated.connect(self.goFwrd)
        self.shortcut_bwrd = QShortcut(QKeySequence("s"), self)
        self.shortcut_bwrd.activated.connect(self.goBwrd)
        self.shortcut_turnRight = QShortcut(QKeySequence("d"), self)
        self.shortcut_turnRight.activated.connect(self.turnRight)
        self.shortcut_turnLeft = QShortcut(QKeySequence("a"), self)
        self.shortcut_turnLeft.activated.connect(self.turnLeft)
        self.shortcut_takeoff = QShortcut(QKeySequence("k"), self)
        self.shortcut_takeoff.activated.connect(self.takeOff)
        self.shortcut_land = QShortcut(QKeySequence("l"), self)
        self.shortcut_land.activated.connect(self.land)

        self.changeQImage.connect(self.setImage) #connecting signal to slot
        self.changeLog.connect(self.setLog) #connecting signal to slot
        self.checkHz.connect(self.getHz) #connecting signal to slot
        self.changeBatteryBar.connect(self.setBatteyBar) #connecting signal to slot

        #creating warning-messagebox
    def messageBox(self, text):
        self.rec_msgBox = QMessageBox()
        self.rec_msgBox.setWindowModality(False)
        self.rec_msgBox.setIcon(QMessageBox.Warning)
        self.rec_msgBox.setWindowTitle("Warning")
        self.rec_msgBox.setText(text)
        self.rec_msgBox.show()
        #creating warning-message for recording
    def recMessageBox(self, text):
        self.rec_msgBox = QMessageBox()
        self.rec_msgBox.setIcon(QMessageBox.Warning)
        self.rec_msgBox.setWindowTitle("Warning")
        self.rec_msgBox.setText(text)
        self.rec_msgBox.setStandardButtons(QMessageBox.Ignore | QMessageBox.Abort)
        ret = self.rec_msgBox.exec_()
        if ret == QMessageBox.Abort:
            return ret
        else:
            return ret
##---------------------------------------------------------------------------------------------
if __name__ == '__main__':
    app = QApplication(sys.argv)
    rospy.init_node('image_read', anonymous=True)
    ex = App()
    sys.exit(app.exec_())
