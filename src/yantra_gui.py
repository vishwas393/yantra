#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'yantra_gui.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!

import sys
from PyQt5 import QtCore, QtGui, QtWidgets
import rospy
import time as pytime
from yantra.srv import *
from yantra.msg import *

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1110, 751)
        self.label = QtWidgets.QLabel(Form)
        self.label.setGeometry(QtCore.QRect(30, 90, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setUnderline(True)
        font.setWeight(75)
        font.setKerning(False)
        self.label.setFont(font)
        self.label.setTextFormat(QtCore.Qt.PlainText)
        self.label.setObjectName("label")
        self.horizontalLayoutWidget_4 = QtWidgets.QWidget(Form)
        self.horizontalLayoutWidget_4.setGeometry(QtCore.QRect(30, 130, 1061, 201))
        self.horizontalLayoutWidget_4.setObjectName("horizontalLayoutWidget_4")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_4)
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label_7 = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        font.setUnderline(True)
        font.setWeight(50)
        font.setKerning(False)
        self.label_7.setFont(font)
        self.label_7.setTextFormat(QtCore.Qt.PlainText)
        self.label_7.setObjectName("label_7")
        self.verticalLayout_4.addWidget(self.label_7)
        self.horizontalLayout_11 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_11.setObjectName("horizontalLayout_11")
        self.label_22 = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        font.setUnderline(False)
        font.setWeight(50)
        font.setKerning(False)
        self.label_22.setFont(font)
        self.label_22.setTextFormat(QtCore.Qt.PlainText)
        self.label_22.setObjectName("label_22")
        self.horizontalLayout_11.addWidget(self.label_22)
        self.pos1x_4 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_4)
        self.pos1x_4.setObjectName("pos1x_4")
        self.horizontalLayout_11.addWidget(self.pos1x_4)
        self.verticalLayout_4.addLayout(self.horizontalLayout_11)
        self.horizontalLayout_12 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_12.setObjectName("horizontalLayout_12")
        self.label_4 = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        font.setUnderline(False)
        font.setWeight(50)
        font.setKerning(False)
        self.label_4.setFont(font)
        self.label_4.setTextFormat(QtCore.Qt.PlainText)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_12.addWidget(self.label_4)
        self.pos1y_4 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_4)
        self.pos1y_4.setObjectName("pos1y_4")
        self.horizontalLayout_12.addWidget(self.pos1y_4)
        self.verticalLayout_4.addLayout(self.horizontalLayout_12)
        self.horizontalLayout_13 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_13.setObjectName("horizontalLayout_13")
        self.label_23 = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        font.setUnderline(False)
        font.setWeight(50)
        font.setKerning(False)
        self.label_23.setFont(font)
        self.label_23.setTextFormat(QtCore.Qt.PlainText)
        self.label_23.setObjectName("label_23")
        self.horizontalLayout_13.addWidget(self.label_23)
        self.pos2z_4 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_4)
        self.pos2z_4.setObjectName("pos2z_4")
        self.horizontalLayout_13.addWidget(self.pos2z_4)
        self.verticalLayout_4.addLayout(self.horizontalLayout_13)
        self.horizontalLayout_4.addLayout(self.verticalLayout_4)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.label_6 = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        font.setUnderline(True)
        font.setWeight(50)
        font.setKerning(False)
        self.label_6.setFont(font)
        self.label_6.setTextFormat(QtCore.Qt.PlainText)
        self.label_6.setObjectName("label_6")
        self.verticalLayout_3.addWidget(self.label_6)
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.label_20 = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        font.setUnderline(False)
        font.setWeight(50)
        font.setKerning(False)
        self.label_20.setFont(font)
        self.label_20.setTextFormat(QtCore.Qt.PlainText)
        self.label_20.setObjectName("label_20")
        self.horizontalLayout_8.addWidget(self.label_20)
        self.pos1x_3 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_4)
        self.pos1x_3.setObjectName("pos1x_3")
        self.horizontalLayout_8.addWidget(self.pos1x_3)
        self.verticalLayout_3.addLayout(self.horizontalLayout_8)
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.label_3 = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        font.setUnderline(False)
        font.setWeight(50)
        font.setKerning(False)
        self.label_3.setFont(font)
        self.label_3.setTextFormat(QtCore.Qt.PlainText)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_9.addWidget(self.label_3)
        self.pos1y_3 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_4)
        self.pos1y_3.setObjectName("pos1y_3")
        self.horizontalLayout_9.addWidget(self.pos1y_3)
        self.verticalLayout_3.addLayout(self.horizontalLayout_9)
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.label_21 = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        font.setUnderline(False)
        font.setWeight(50)
        font.setKerning(False)
        self.label_21.setFont(font)
        self.label_21.setTextFormat(QtCore.Qt.PlainText)
        self.label_21.setObjectName("label_21")
        self.horizontalLayout_10.addWidget(self.label_21)
        self.pos2z_3 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_4)
        self.pos2z_3.setObjectName("pos2z_3")
        self.horizontalLayout_10.addWidget(self.pos2z_3)
        self.verticalLayout_3.addLayout(self.horizontalLayout_10)
        self.horizontalLayout_4.addLayout(self.verticalLayout_3)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.label_5 = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        font.setUnderline(True)
        font.setWeight(50)
        font.setKerning(False)
        self.label_5.setFont(font)
        self.label_5.setTextFormat(QtCore.Qt.PlainText)
        self.label_5.setObjectName("label_5")
        self.verticalLayout.addWidget(self.label_5)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_15 = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        font.setUnderline(False)
        font.setWeight(50)
        font.setKerning(False)
        self.label_15.setFont(font)
        self.label_15.setTextFormat(QtCore.Qt.PlainText)
        self.label_15.setObjectName("label_15")
        self.horizontalLayout_3.addWidget(self.label_15)
        self.pos1x = QtWidgets.QLineEdit(self.horizontalLayoutWidget_4)
        self.pos1x.setObjectName("pos1x")
        self.horizontalLayout_3.addWidget(self.pos1x)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_2 = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        font.setUnderline(False)
        font.setWeight(50)
        font.setKerning(False)
        self.label_2.setFont(font)
        self.label_2.setTextFormat(QtCore.Qt.PlainText)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout.addWidget(self.label_2)
        self.pos1y = QtWidgets.QLineEdit(self.horizontalLayoutWidget_4)
        self.pos1y.setObjectName("pos1y")
        self.horizontalLayout.addWidget(self.pos1y)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_14 = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        font.setUnderline(False)
        font.setWeight(50)
        font.setKerning(False)
        self.label_14.setFont(font)
        self.label_14.setTextFormat(QtCore.Qt.PlainText)
        self.label_14.setObjectName("label_14")
        self.horizontalLayout_2.addWidget(self.label_14)
        self.pos2z = QtWidgets.QLineEdit(self.horizontalLayoutWidget_4)
        self.pos2z.setObjectName("pos2z")
        self.horizontalLayout_2.addWidget(self.pos2z)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_4.addLayout(self.verticalLayout)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_16 = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        font.setUnderline(True)
        font.setWeight(50)
        font.setKerning(False)
        self.label_16.setFont(font)
        self.label_16.setTextFormat(QtCore.Qt.PlainText)
        self.label_16.setObjectName("label_16")
        self.verticalLayout_2.addWidget(self.label_16)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label_17 = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        font.setUnderline(False)
        font.setWeight(50)
        font.setKerning(False)
        self.label_17.setFont(font)
        self.label_17.setTextFormat(QtCore.Qt.PlainText)
        self.label_17.setObjectName("label_17")
        self.horizontalLayout_5.addWidget(self.label_17)
        self.pos1x_2 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_4)
        self.pos1x_2.setObjectName("pos1x_2")
        self.horizontalLayout_5.addWidget(self.pos1x_2)
        self.verticalLayout_2.addLayout(self.horizontalLayout_5)
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.label_18 = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        font.setUnderline(False)
        font.setWeight(50)
        font.setKerning(False)
        self.label_18.setFont(font)
        self.label_18.setTextFormat(QtCore.Qt.PlainText)
        self.label_18.setObjectName("label_18")
        self.horizontalLayout_6.addWidget(self.label_18)
        self.pos1y_2 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_4)
        self.pos1y_2.setObjectName("pos1y_2")
        self.horizontalLayout_6.addWidget(self.pos1y_2)
        self.verticalLayout_2.addLayout(self.horizontalLayout_6)
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.label_19 = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(12)
        font.setBold(False)
        font.setUnderline(False)
        font.setWeight(50)
        font.setKerning(False)
        self.label_19.setFont(font)
        self.label_19.setTextFormat(QtCore.Qt.PlainText)
        self.label_19.setObjectName("label_19")
        self.horizontalLayout_7.addWidget(self.label_19)
        self.pos2z_2 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_4)
        self.pos2z_2.setObjectName("pos2z_2")
        self.horizontalLayout_7.addWidget(self.pos2z_2)
        self.verticalLayout_2.addLayout(self.horizontalLayout_7)
        self.horizontalLayout_4.addLayout(self.verticalLayout_2)
        self.pushButton = QtWidgets.QPushButton(Form)
        self.pushButton.setGeometry(QtCore.QRect(830, 660, 261, 71))
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton.setFont(font)
        self.pushButton.setObjectName("pushButton")
        self.pushButton.clicked.connect(lambda: self.send_button_pressed_callback())
        self.label_8 = QtWidgets.QLabel(Form)
        self.label_8.setGeometry(QtCore.QRect(490, 20, 151, 41))
        font = QtGui.QFont()
        font.setPointSize(23)
        font.setBold(True)
        font.setWeight(75)
        font.setStrikeOut(False)
        self.label_8.setFont(font)
        self.label_8.setObjectName("label_8")
        self.verticalLayoutWidget_6 = QtWidgets.QWidget(Form)
        self.verticalLayoutWidget_6.setGeometry(QtCore.QRect(30, 490, 271, 201))
        self.verticalLayoutWidget_6.setObjectName("verticalLayoutWidget_6")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_6)
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.horizontalLayout_14 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_14.setObjectName("horizontalLayout_14")
        self.label_9 = QtWidgets.QLabel(self.verticalLayoutWidget_6)
        self.label_9.setObjectName("label_9")
        self.horizontalLayout_14.addWidget(self.label_9)
        self.time1 = QtWidgets.QLineEdit(self.verticalLayoutWidget_6)
        self.time1.setObjectName("time1")
        self.horizontalLayout_14.addWidget(self.time1)
        self.verticalLayout_5.addLayout(self.horizontalLayout_14)
        self.horizontalLayout_17 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_17.setObjectName("horizontalLayout_17")
        self.label_12 = QtWidgets.QLabel(self.verticalLayoutWidget_6)
        self.label_12.setObjectName("label_12")
        self.horizontalLayout_17.addWidget(self.label_12)
        self.time2 = QtWidgets.QLineEdit(self.verticalLayoutWidget_6)
        self.time2.setObjectName("time2")
        self.horizontalLayout_17.addWidget(self.time2)
        self.verticalLayout_5.addLayout(self.horizontalLayout_17)
        self.horizontalLayout_18 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_18.setObjectName("horizontalLayout_18")
        self.label_13 = QtWidgets.QLabel(self.verticalLayoutWidget_6)
        self.label_13.setObjectName("label_13")
        self.horizontalLayout_18.addWidget(self.label_13)
        self.time3 = QtWidgets.QLineEdit(self.verticalLayoutWidget_6)
        self.time3.setObjectName("time3")
        self.horizontalLayout_18.addWidget(self.time3)
        self.verticalLayout_5.addLayout(self.horizontalLayout_18)
        self.horizontalLayout_15 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_15.setObjectName("horizontalLayout_15")
        self.label_10 = QtWidgets.QLabel(self.verticalLayoutWidget_6)
        self.label_10.setObjectName("label_10")
        self.horizontalLayout_15.addWidget(self.label_10)
        self.time4 = QtWidgets.QLineEdit(self.verticalLayoutWidget_6)
        self.time4.setObjectName("time4")
        self.horizontalLayout_15.addWidget(self.time4)
        self.verticalLayout_5.addLayout(self.horizontalLayout_15)
        self.horizontalLayout_16 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_16.setObjectName("horizontalLayout_16")
        self.label_11 = QtWidgets.QLabel(self.verticalLayoutWidget_6)
        self.label_11.setObjectName("label_11")
        self.horizontalLayout_16.addWidget(self.label_11)
        self.time5 = QtWidgets.QLineEdit(self.verticalLayoutWidget_6)
        self.time5.setObjectName("time5")
        self.horizontalLayout_16.addWidget(self.time5)
        self.verticalLayout_5.addLayout(self.horizontalLayout_16)
        self.label_24 = QtWidgets.QLabel(Form)
        self.label_24.setGeometry(QtCore.QRect(30, 440, 111, 31))
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setUnderline(True)
        font.setWeight(75)
        font.setKerning(False)
        self.label_24.setFont(font)
        self.label_24.setTextFormat(QtCore.Qt.PlainText)
        self.label_24.setObjectName("label_24")
        self.label_25 = QtWidgets.QLabel(Form)
        self.label_25.setGeometry(QtCore.QRect(30, 340, 211, 17))
        self.label_25.setObjectName("label_25")
        self.label_26 = QtWidgets.QLabel(Form)
        self.label_26.setGeometry(QtCore.QRect(30, 700, 211, 50))
        self.label_26.setObjectName("label_26")

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.label.setText(_translate("Form", "LOCATION :"))
        self.label_7.setText(_translate("Form", "Point 1 : "))
        self.label_22.setText(_translate("Form", "X :"))
        self.pos1x_4.setText(_translate("Form", "300"))
        self.label_4.setText(_translate("Form", "Y :"))
        self.pos1y_4.setText(_translate("Form", "0"))
        self.label_23.setText(_translate("Form", "Z :"))
        self.pos2z_4.setText(_translate("Form", "300"))
        self.label_6.setText(_translate("Form", "Point 2 : "))
        self.label_20.setText(_translate("Form", "X :"))
        self.pos1x_3.setText(_translate("Form", "200"))
        self.label_3.setText(_translate("Form", "Y :"))
        self.pos1y_3.setText(_translate("Form", "200"))
        self.label_21.setText(_translate("Form", "Z :"))
        self.pos2z_3.setText(_translate("Form", "450"))
        self.label_5.setText(_translate("Form", "Point 3 : "))
        self.label_15.setText(_translate("Form", "X :"))
        self.pos1x.setText(_translate("Form", "0"))
        self.label_2.setText(_translate("Form", "Y :"))
        self.pos1y.setText(_translate("Form", "200"))
        self.label_14.setText(_translate("Form", "Z :"))
        self.pos2z.setText(_translate("Form", "500"))
        self.label_16.setText(_translate("Form", "Point 4 : "))
        self.label_17.setText(_translate("Form", "X :"))
        self.pos1x_2.setText(_translate("Form", "-200"))
        self.label_18.setText(_translate("Form", "Y :"))
        self.pos1y_2.setText(_translate("Form", "500"))
        self.label_19.setText(_translate("Form", "Z :"))
        self.pos2z_2.setText(_translate("Form", "300"))
        self.pushButton.setText(_translate("Form", "SEND "))
        self.label_8.setText(_translate("Form", "YANTRA"))
        self.label_9.setText(_translate("Form", "Path Point 1:"))
        self.time1.setText(_translate("Form", "2"))
        self.label_12.setText(_translate("Form", "Path Point 2:"))
        self.time2.setText(_translate("Form", "4"))
        self.label_13.setText(_translate("Form", "Path Point 3:"))
        self.time3.setText(_translate("Form", "6"))
        self.label_10.setText(_translate("Form", "Path Point 4:"))
        self.time4.setText(_translate("Form", "8"))
        self.label_11.setText(_translate("Form", "Path Point 5:"))
        self.time5.setText(_translate("Form", "10"))
        self.label_24.setText(_translate("Form", "TIME :"))
        self.label_25.setText(_translate("Form", "* All values are in milimeter."))
        self.label_26.setText(_translate("Form", "* All values are in seconds. Time for Path Point 5 is ignored for now."))
        
    def send_button_pressed_callback(self):
        self.send_button_pressed = 1
        self.pushButton.setEnabled(False)
    

    def service_callback(self, req):
        self.pushButton.setEnabled(True)
        while (self.send_button_pressed != 1):
            pytime.sleep(0.5)
        
        # Naming for PyQt objects and path points are mismatched. For doubt, look up PyQt file in QtDesigner app.
        p1 = Position()
        p1.pos = [float(self.pos1x_4.text()), float(self.pos1y_4.text()), float(self.pos2z_4.text())]

        p2 = Position()
        p2.pos = [float(self.pos1x_3.text()), float(self.pos1y_3.text()), float(self.pos2z_3.text())]

        p3 = Position()
        p3.pos = [float(self.pos1x.text()), float(self.pos1y.text()), float(self.pos2z.text())]

        p4 = Position()
        p4.pos = [float(self.pos1x_2.text()), float(self.pos1y_2.text()), float(self.pos2z_2.text())]

        time = [float(self.time1.text()), float(self.time2.text()), float(self.time3.text()), float(self.time4.text())]

        resp = PathPointsGuiResponse()
        resp.pp = [p1, p2, p3, p4]
        resp.pp_time = time

        return resp



    def __init__(self):
        rospy.init_node("yantra_gui")
        self.srvc = rospy.Service('/yantra_gui', PathPointsGui, self.service_callback)
        self.send_button_pressed = 0
        app = QtWidgets.QApplication(sys.argv)
        MainWindow = QtWidgets.QMainWindow()
        self.setupUi(MainWindow)
        MainWindow.show()
        sys.exit(app.exec_())
        rospy.spin()


if __name__ == "__main__":
    ui = Ui_Form()
