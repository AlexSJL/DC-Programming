#!/usr/bin/env python
# coding: utf-8

# In[1]:


# Import Packages
from port_ctrl1 import Rigol_DCPort
from CAENDesktopHighVoltagePowerSupply1 import CAENDesktopHighVoltagePowerSupply, OneCAENChannel
import pyvisa as visa
import time
import numpy as np
import serial
import serial.tools.list_ports 
from matplotlib import pyplot as plt
from DC_Scan_class import *
from random import randint

import sys
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg


# In[2]:


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(2500, 1500)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        
        # Connecting CAEN device, Button
        self.sendbutton = QtWidgets.QPushButton(self.centralwidget)
        self.sendbutton.setGeometry(QtCore.QRect(230, 280, 200, 50)) # upper left xy wh
        font = QtGui.QFont()
        font.setPointSize(12)
        self.sendbutton.setFont(font)
        self.sendbutton.setObjectName("sendbutton")
        
        # Configuration devices, Button
        self.confbutton = QtWidgets.QPushButton(self.centralwidget)
        self.confbutton.setGeometry(QtCore.QRect(125, 370, 150, 50)) # upper left xy wh
        font = QtGui.QFont()
        font.setPointSize(12)
        self.confbutton.setFont(font)
        self.confbutton.setObjectName("confbutton")
        
        # Cancel Configuration, Button
        self.cancelbutton = QtWidgets.QPushButton(self.centralwidget)
        self.cancelbutton.setGeometry(QtCore.QRect(285, 370, 150, 50)) # upper left xy wh
        font = QtGui.QFont()
        font.setPointSize(12)
        self.cancelbutton.setFont(font)
        self.cancelbutton.setObjectName("cancelbutton")
        self.cancelbutton.setEnabled(False)
        
        # Port Selection, Selection List
        self.sendcom = QtWidgets.QComboBox(self.centralwidget)
        self.sendcom.setGeometry(QtCore.QRect(280, 100, 150, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.sendcom.setFont(font)
        self.sendcom.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.sendcom.setObjectName("sendcom")
        self.sendcom.addItem("")
        self.sendcom.setItemText(0, "")
        
        # Baud Rate Selection, Selection List
        self.sendbot = QtWidgets.QComboBox(self.centralwidget)
        self.sendbot.setGeometry(QtCore.QRect(280, 190, 150, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.sendbot.setFont(font)
        self.sendbot.setObjectName("sendbot")
        self.sendbot.addItem("")
        self.sendbot.addItem("")
        
        # Port, Label
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(125, 100, 70, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label.setFont(font)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        
        # Baud Rate, Label
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(60, 190, 200, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_2.setFont(font)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        
        # AQ Graph window
        self.plotwindow = pg.PlotWidget(self.centralwidget)
        self.plotwindow.setGeometry(QtCore.QRect(61, 551, 1668, 898))
        self.plotwindow.setBackground("w")
        self.plotwindow.plotItem.setLabel("bottom",text="RF Amplitude (V)")
        self.plotwindow.plotItem.setLabel("left",text="DC Voltage (V)")
        self.plotwindow.setMouseEnabled(x=False, y=False)
        self.plotwindow.addLegend()
        self.plotwindow.showGrid(x=True, y=True)
        self.plotwindow.setObjectName("plotwindow")
        
        # AQ Graph, Label
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(60, 470, 160, 90))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_3.setFont(font)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        
        # AQ Graph Rectangle
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setGeometry(QtCore.QRect(60, 550, 1670, 900))
        self.label_4.setFrameShape(QtWidgets.QFrame.Box)
        self.label_4.setFrameShadow(QtWidgets.QFrame.Plain)
        font = QtGui.QFont()
        self.label_4.setStyleSheet("background-color: White")
        font.setPointSize(12)
        self.label_4.setFont(font)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.label_4.lower()
        
        # Message Display window
        self.messagewindow = QtWidgets.QTextEdit(self.centralwidget)
        self.messagewindow.setGeometry(QtCore.QRect(1770, 550, 600, 900))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.messagewindow.setFont(font)
        self.messagewindow.setReadOnly(True)
        self.messagewindow.setObjectName("messagewindow")
        
        # Messages, Label
        self.messagelabel = QtWidgets.QLabel(self.centralwidget)
        self.messagelabel.setGeometry(QtCore.QRect(1770, 470, 160, 90))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.messagelabel.setFont(font)
        self.messagelabel.setAlignment(QtCore.Qt.AlignCenter)
        self.messagelabel.setObjectName("messagelabel")
        
        # Input 1:  RF starting amplitude, text input window
        self.Aminputdata = QtWidgets.QLineEdit(self.centralwidget)
        self.Aminputdata.setGeometry(QtCore.QRect(970, 100, 200, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.Aminputdata.setPlaceholderText("0 - 6 V")
        self.Aminputdata.setFont(font)
        self.Aminputdata.setObjectName("Aminputdata")
        self.Aminputdata.setReadOnly(True)
        reg1 = QtCore.QRegExp('[0-5]?\.\d{4}|6?')
        validator1 = QtGui.QRegExpValidator()
        validator1.setRegExp(reg1)
        self.Aminputdata.setValidator(validator1)
        
        # Input 1 Label
        self.Aminputlabel = QtWidgets.QLabel(self.centralwidget)
        self.Aminputlabel.setGeometry(QtCore.QRect(590, 100, 350, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.Aminputlabel.setFont(font)
        self.Aminputlabel.setAlignment(QtCore.Qt.AlignCenter)
        self.Aminputlabel.setObjectName("Aminputlabel")
        
        # Input 2: RF ending amplitude, text input window
        self.AMinputdata = QtWidgets.QLineEdit(self.centralwidget)
        self.AMinputdata.setGeometry(QtCore.QRect(970, 190, 200, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.AMinputdata.setPlaceholderText("0 - 6 V")
        self.AMinputdata.setFont(font)
        self.AMinputdata.setObjectName("AMinputdata")
        self.AMinputdata.setReadOnly(True)
        self.AMinputdata.setValidator(validator1)
        
        # Input 2 Label
        self.AMinputlabel = QtWidgets.QLabel(self.centralwidget)
        self.AMinputlabel.setGeometry(QtCore.QRect(590, 190, 350, 50)) 
        font = QtGui.QFont()
        font.setPointSize(12)
        self.AMinputlabel.setFont(font)
        self.AMinputlabel.setAlignment(QtCore.Qt.AlignCenter)
        self.AMinputlabel.setObjectName("AMinputlabel")
        
        # Input 3: DC lowest Volt, text input window
        self.Qminputdata = QtWidgets.QLineEdit(self.centralwidget)
        self.Qminputdata.setGeometry(QtCore.QRect(1570, 100, 200, 50)) 
        font = QtGui.QFont()
        font.setPointSize(12)
        self.Qminputdata.setPlaceholderText("0 - 3000 V")
        self.Qminputdata.setFont(font)
        self.Qminputdata.setObjectName("Qminputdata")
        self.Qminputdata.setReadOnly(True)
        reg2 = QtCore.QRegExp('\d?(\.\d{2})|[1-9]\d(\.\d{2})|[1-9]\d{2}(\.\d{2})|[1-2]\d{3}(\.\d{2})|3000?')
        validator2 = QtGui.QRegExpValidator()
        validator2.setRegExp(reg2)
        self.Qminputdata.setValidator(validator2)
        
        # Input 3 Label
        self.Qminputlabel = QtWidgets.QLabel(self.centralwidget)
        self.Qminputlabel.setGeometry(QtCore.QRect(1180, 100, 370, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.Qminputlabel.setFont(font)
        self.Qminputlabel.setAlignment(QtCore.Qt.AlignCenter)
        self.Qminputlabel.setObjectName("Qminputlabel")
        
        # Input 4: DC greatest Volt, text input window
        self.QMinputdata = QtWidgets.QLineEdit(self.centralwidget)
        self.QMinputdata.setGeometry(QtCore.QRect(1570, 190, 200, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.QMinputdata.setPlaceholderText("0 - 3000 V")
        self.QMinputdata.setFont(font)
        self.QMinputdata.setObjectName("QMinputdata")
        self.QMinputdata.setReadOnly(True)
        self.QMinputdata.setValidator(validator2)
        
        # Input 4 Label
        self.QMinputlabel = QtWidgets.QLabel(self.centralwidget)
        self.QMinputlabel.setGeometry(QtCore.QRect(1180, 200, 370, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.QMinputlabel.setFont(font)
        self.QMinputlabel.setAlignment(QtCore.Qt.AlignCenter)
        self.QMinputlabel.setObjectName("QMinputlabel")
        
        # Input 5: Hstep
        self.Hstep = QtWidgets.QLineEdit(self.centralwidget)
        self.Hstep.setGeometry(QtCore.QRect(1570, 370, 200, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.Hstep.setPlaceholderText("sep by ','")
        self.Hstep.setFont(font)
        self.Hstep.setObjectName("Hstep")
        self.Hstep.setReadOnly(True)
        self.Hstep.setValidator(validator2)
        reg4 = QtCore.QRegExp("^(((0(\.\d{1,2})?)|([1-9](\.\d{0,2})?)|([1-9]\d(\.\d{0,2})?)|([1-9]\d\d(\.\d{0,2})?)|([1-2]\d\d\d(\.\d{0,2})?)|(3000)))(,\s*((0(\.\d{1,2})?)|([1-9](\.\d{0,2})?)|([1-9]\d(\.\d{0,2})?)|([1-9]\d\d(\.\d{0,2})?)|([1-2]\d\d\d(\.\d{0,2})?)|(3000)))*$")
        validator4 = QtGui.QRegExpValidator()
        validator4.setRegExp(reg4)
        self.Hstep.setValidator(validator4)
        
        # Input 5 Hstep Label
        self.Hsteplabel = QtWidgets.QLabel(self.centralwidget)
        self.Hsteplabel.setGeometry(QtCore.QRect(1210, 370, 300, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.Hsteplabel.setFont(font)
        self.Hsteplabel.setAlignment(QtCore.Qt.AlignCenter)
        self.Hsteplabel.setObjectName("Hsteplabel")
        
        # Input 6: Astep
        self.Astep = QtWidgets.QLineEdit(self.centralwidget)
        self.Astep.setGeometry(QtCore.QRect(970, 370, 200, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.Astep.setPlaceholderText("sep by ','")
        self.Astep.setFont(font)
        self.Astep.setObjectName("Astep")
        self.Astep.setReadOnly(True)
        reg3 = QtCore.QRegExp("^(((0(\.\d{1,4})?)|([1-5](\.\d{0,4})?))|(6))(,\s*((0(\.\d{1,4})?)|([1-5](\.\d{0,4})?))|(6))*$")
        validator3 = QtGui.QRegExpValidator()
        validator3.setRegExp(reg3)
        self.Astep.setValidator(validator3)
        
        # Input 6 Astep Label
        self.Asteplabel = QtWidgets.QLabel(self.centralwidget)
        self.Asteplabel.setGeometry(QtCore.QRect(620, 370, 305, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.Asteplabel.setFont(font)
        self.Asteplabel.setAlignment(QtCore.Qt.AlignCenter)
        self.Asteplabel.setObjectName("Asteplabel")
        
        # Input 5: Channel for RF
        self.AChannel = QtWidgets.QComboBox(self.centralwidget)
        self.AChannel.setGeometry(QtCore.QRect(2160, 100, 200, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.AChannel.setFont(font)
        self.AChannel.setObjectName("AChannel")
        self.AChannel.addItem("")
        self.AChannel.addItem("")
        self.AChannel.addItem("")
        self.AChannel.setEnabled(False)
        
        # Input 5 Label
        self.AChannellabel = QtWidgets.QLabel(self.centralwidget)
        self.AChannellabel.setGeometry(QtCore.QRect(1800, 100, 300, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.AChannellabel.setFont(font)
        self.AChannellabel.setAlignment(QtCore.Qt.AlignCenter)
        self.AChannellabel.setObjectName("AChannellabel")
        
        # Input 6: Channel1 for DC
        self.QChannel = QtWidgets.QComboBox(self.centralwidget)
        self.QChannel.setGeometry(QtCore.QRect(2160, 190, 200, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.QChannel.setFont(font)
        self.QChannel.setObjectName("QChannel")
        self.QChannel.addItem("")
        self.QChannel.addItem("")
        self.QChannel.addItem("")
        self.QChannel.addItem("")
        self.QChannel.setEnabled(False)
        
        # Input 6 Label
        self.QChannellabel = QtWidgets.QLabel(self.centralwidget)
        self.QChannellabel.setGeometry(QtCore.QRect(1800, 190, 305, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.QChannellabel.setFont(font)
        self.QChannellabel.setAlignment(QtCore.Qt.AlignCenter)
        self.QChannellabel.setObjectName("QChannellabel")
        
        # Input 6: Channel2 for DC
        self.QChannel1 = QtWidgets.QComboBox(self.centralwidget)
        self.QChannel1.setGeometry(QtCore.QRect(2160, 280, 200, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.QChannel1.setFont(font)
        self.QChannel1.setObjectName("QChannel")
        self.QChannel1.addItem("")
        self.QChannel1.addItem("")
        self.QChannel1.addItem("")
        self.QChannel1.addItem("")
        self.QChannel1.setEnabled(False)
        
        # Input 6 Label Channel 2
        self.QChannel1label = QtWidgets.QLabel(self.centralwidget)
        self.QChannel1label.setGeometry(QtCore.QRect(1800, 280, 305, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.QChannel1label.setFont(font)
        self.QChannel1label.setAlignment(QtCore.Qt.AlignCenter)
        self.QChannel1label.setObjectName("QChannel1label")
        
        # Input 7: Number of Sampling Points
        self.Sample = QtWidgets.QLineEdit(self.centralwidget)
        self.Sample.setGeometry(QtCore.QRect(970, 280, 200, 50)) #1570, 300, 190, 50
        font = QtGui.QFont()
        font.setPointSize(12)
        self.Sample.setFont(font)
        self.Sample.setObjectName("Sample")
        self.Sample.setReadOnly(True)
        reg5 = QtCore.QRegExp("^[1-9]\d*$")
        validator5 = QtGui.QRegExpValidator()
        validator5.setRegExp(reg5)
        self.Sample.setValidator(validator5)
        
        # Input 7 Label
        self.Samplelabel = QtWidgets.QLabel(self.centralwidget)
        self.Samplelabel.setGeometry(QtCore.QRect(620, 280, 300, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.Samplelabel.setFont(font)
        self.Samplelabel.setAlignment(QtCore.Qt.AlignCenter)
        self.Samplelabel.setObjectName("Samplelabel")
        
        # Input 8: Duration
        self.Duration = QtWidgets.QLineEdit(self.centralwidget)
        self.Duration.setGeometry(QtCore.QRect(1570, 280, 200, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.Duration.setFont(font)
        self.Duration.setObjectName("Sample")
        self.Duration.setReadOnly(True)
        reg6 = QtCore.QRegExp("^((0\.\d{1,3})|([1-9]\d*(\.\d{1,3})?))(,\s*((0\.\d{1,3})|([1-9]\d*(\.\d{1,3})?)))*$")
        validator6 = QtGui.QRegExpValidator(reg6)
        validator6.setRegExp(reg6)
        self.Duration.setValidator(validator6)
        self.QMinputdata.setPlaceholderText("Up to 3 deci")
        
        # Input 8 Label
        self.Durationlabel = QtWidgets.QLabel(self.centralwidget)
        self.Durationlabel.setGeometry(QtCore.QRect(1210, 280, 300, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.Durationlabel.setFont(font)
        self.Durationlabel.setAlignment(QtCore.Qt.AlignCenter)
        self.Durationlabel.setObjectName("Durationlabel")
        
        # Rectangle for Communication Ports
        self.Com_portal = QtWidgets.QLabel(self.centralwidget)
        self.Com_portal.setGeometry(QtCore.QRect(60, 90, 400, 250))
        self.Com_portal.setFrameShape(QtWidgets.QFrame.Box)
        self.Com_portal.setFrameShadow(QtWidgets.QFrame.Plain)
        self.Com_portal.setAlignment(QtCore.Qt.AlignCenter)
        self.Com_portal.lower()
        
        # Label for Communication Ports
        self.Com_portallabel = QtWidgets.QLabel(self.centralwidget)
        self.Com_portallabel.setGeometry(QtCore.QRect(60, 50, 300, 50))
        self.Com_portallabel.setAlignment(QtCore.Qt.AlignCenter)
        
        # Rectangle for Parameter Settings
        self.Par_set = QtWidgets.QLabel(self.centralwidget)
        self.Par_set.setGeometry(QtCore.QRect(570, 90, 1800, 350))
        self.Par_set.setFrameShape(QtWidgets.QFrame.Box)
        self.Par_set.setFrameShadow(QtWidgets.QFrame.Plain)
        self.Par_set.setAlignment(QtCore.Qt.AlignCenter)
        self.Par_set.lower()
        
        # Label for Parameter Settings
        self.Par_setlabel = QtWidgets.QLabel(self.centralwidget)
        self.Par_setlabel.setGeometry(QtCore.QRect(570, 50, 350, 50))
        self.Par_setlabel.setAlignment(QtCore.Qt.AlignCenter)
        
        # Mode Selection
        self.custom = QtWidgets.QCheckBox(self.centralwidget)
        self.custom.setGeometry(QtCore.QRect(2100, 370, 220, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.custom.setFont(font)
        self.custom.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.custom.setObjectName("custom")
        
        self.auto = QtWidgets.QCheckBox(self.centralwidget)
        self.auto.setGeometry(QtCore.QRect(1850, 370, 210, 50))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.auto.setFont(font)
        self.auto.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.auto.setObjectName("auto")
        
        self.custom.stateChanged.connect(self.mode_select)
        self.auto.stateChanged.connect(self.mode_select)
        
        # Display all designs above in the window
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 775, 18))
        self.menubar.setObjectName("menubar")
        
        # Headline Menu
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        
        # Enable buttons and Plots in UI
        self.retranslateUi(MainWindow)
        # Connecting port with preset baudrate and port
        #self.sendbutton.clicked.connect(MainWindow.open_com)
        #self.sendcom.activated['QString'].connect(MainWindow.port_changed)
        #self.sendbot.activated['QString'].connect(MainWindow.baud_changed)
        #
        #self.confbutton.clicked.connect(MainWindow.config)
        #self.cancelbutton.clicked.connect(MainWindow.cancel)
        #QtCore.QMetaObject.connectSlotsByName(MainWindow)    


# In[3]:


class Ui_MainWindow(Ui_MainWindow):    
    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Amplitude Scan Control Panel"))
        self.sendbutton.setStatusTip(_translate("MainWindow", "Click to Connect"))
        self.sendbutton.setText(_translate("MainWindow", "Port Conn"))
        self.sendbot.setItemText(0, _translate("MainWindow", "9600"))
        self.sendbot.setItemText(1, _translate("MainWindow", "115200"))
        
        #self.AChannel.setItemText(0, _translate("MainWindow", ""))
        self.AChannel.setItemText(0, _translate("MainWindow", "CH1"))
        self.AChannel.setItemText(1, _translate("MainWindow", "CH2"))
        self.AChannel.setItemText(2, _translate("MainWindow", "CH3"))
        
        #self.QChannel.setItemText(0, _translate("MainWindow", ""))
        self.QChannel.setItemText(0, _translate("MainWindow", "CH0"))
        self.QChannel.setItemText(1, _translate("MainWindow", "CH1"))
        self.QChannel.setItemText(2, _translate("MainWindow", "CH2"))
        self.QChannel.setItemText(3, _translate("MainWindow", "CH3"))
        
        #self.QChannel1.setItemText(0, _translate("MainWindow", ""))
        self.QChannel1.setItemText(0, _translate("MainWindow", "CH4"))
        self.QChannel1.setItemText(1, _translate("MainWindow", "CH5"))
        self.QChannel1.setItemText(2, _translate("MainWindow", "CH6"))
        self.QChannel1.setItemText(3, _translate("MainWindow", "CH7"))
        
        self.label.setText(_translate("MainWindow", "Port"))
        self.label_2.setText(_translate("MainWindow", "Baud Rate"))
        self.label_3.setText(_translate("MainWindow", "A-Q Graph"))
        
        self.Aminputlabel.setText(_translate("MainWindow", "Start Amplitude (V):"))
        self.AMinputlabel.setText(_translate("MainWindow", "End Amplitude (V):"))
        self.Qminputlabel.setText(_translate("MainWindow", "Start DC Voltage (V):"))
        self.QMinputlabel.setText(_translate("MainWindow", "End DC Voltage (V):"))
        self.AChannellabel.setText(_translate("MainWindow", "Channel for RF:"))
        self.QChannellabel.setText(_translate("MainWindow", "Channel 1 for DC:"))
        self.QChannel1label.setText(_translate("MainWindow", "Channel 2 for DC:"))

        self.Asteplabel.setText(_translate("MainWindow", "RF Step (V):"))
        self.Hsteplabel.setText(_translate("MainWindow", "DC Step (V):"))
        
        self.Samplelabel.setText(_translate("MainWindow", "Number of Samples:"))
        self.Com_portallabel.setText(_translate("MainWindow", "Communication Settings"))
        self.Par_setlabel.setText(_translate("MainWindow", "Scanning Parameter Settings"))
        self.confbutton.setText(_translate("MainWindow", "Configure"))
        self.cancelbutton.setText(_translate("MainWindow", "Cancel"))
        self.messagelabel.setText(_translate("MainWindow", "Messages"))
        self.Durationlabel.setText(_translate("MainWindow", "Duration (s):"))
        
        self.custom.setText(_translate("MainWindow", "Custom Mode"))
        self.auto.setText(_translate("MainWindow", "Auto Mode"))


# In[4]:


class Ui_MainWindow(Ui_MainWindow):
    def mode_select(self, state):
        # If either check box is checked, disable the other
        if state == Qt.Checked:
            if self.sender() == self.custom:
                self.auto.setChecked(False)
                self.AChannel.setEnabled(True)
                self.QChannel.setEnabled(True)
                self.QChannel1.setEnabled(True)
                self.Aminputdata.setReadOnly(False)
                self.AMinputdata.setReadOnly(False)
                self.Qminputdata.setReadOnly(False)
                self.QMinputdata.setReadOnly(False)
                self.Duration.setReadOnly(False)
                self.Astep.setReadOnly(False)
                self.Hstep.setReadOnly(False)
                self.Sample.setReadOnly(True)
            
            if self.sender() == self.auto:
                self.custom.setChecked(False)
                self.Aminputdata.setReadOnly(False)
                self.AMinputdata.setReadOnly(False)
                self.Qminputdata.setReadOnly(False)
                self.QMinputdata.setReadOnly(False)
                self.Sample.setReadOnly(False)
                self.Duration.setReadOnly(False)
                self.AChannel.setEnabled(True)
                self.QChannel.setEnabled(True)
                self.QChannel1.setEnabled(True)
                self.Astep.setReadOnly(True)
                self.Hstep.setReadOnly(True)
        else:
            self.AChannel.setEnabled(False)
            self.QChannel.setEnabled(False)
            self.QChannel1.setEnabled(False)
            self.Aminputdata.setReadOnly(True)
            self.AMinputdata.setReadOnly(True)
            self.Qminputdata.setReadOnly(True)
            self.QMinputdata.setReadOnly(True)
            self.Duration.setReadOnly(True)
            self.Astep.setReadOnly(True)
            self.Hstep.setReadOnly(True)
            self.Sample.setReadOnly(True)

