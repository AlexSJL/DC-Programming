#!/usr/bin/env python
# coding: utf-8

# In[1]:


from port_ctrl1 import Rigol_DCPort
from CAENDesktopHighVoltagePowerSupply1 import CAENDesktopHighVoltagePowerSupply, OneCAENChannel
import pyvisa as visa
import time
import numpy as np
import serial
import serial.tools.list_ports 
from matplotlib import pyplot as plt


# In[2]:


# Constrain RF data in the range between 0-6 V
class DC_Scan:
    def __init__(self,ser,Rigol_address="USB0::0x1AB1::0x0E11::DP8B242401816::INSTR",Hrange=None,Arange=None,Achannel=None,Hchannel=None, sampling=None, Astep=None, Hstep=None, intv=None, mode=None):
        self.RF = Rigol_DCPort(Rigol_address)
        self.DC = CAENDesktopHighVoltagePowerSupply(ser)
        self.Hrange=Hrange
        self.Arange=Arange
        self.Achannel=Achannel
        self.Hchannel=Hchannel
        self.sampling=sampling
        self.Astep=Astep
        self.Hstep=Hstep
        self.intv=intv
        self.working_message = None
        self.error_message = None
        self.error = False
        self.complete = False
        self.mode=mode
        
    def RF_validity(self):
        if self.Arange is None:
            self.error_message = "No RF Input"
            return False
        for vol in range(len(self.Arange)):
            if self.Arange[vol] < 0 or self.Arange[vol] > 6:
                self.error_message = "Invalid RF Amplitude Range"
                return False
            else:
                self.Arange[vol] = float(self.Arange[vol])
        return True

    def DC_validity(self):
        if self.Hrange is None:
            self.error_message = "No DC Input"
            return False
        for vol in range(len(self.Hrange)):
            if vol < 0 or vol > 3000:
                self.error_message = "Invalid DC Voltage Range"
                return False
            else:
                self.Hrange[vol] = float(self.Hrange[vol])
        return True
    
    def sampling_validity(self):
        curr_q = np.linspace(Hrange[0], Hrange[1], sampling)
        if np.abs(curr_q[1] - curr_q[0]) > 40:
            self.sampling = int(np.ceil(np.abs(Hrange[1] - Hrange[0]) / 40))
    
    def duration_check(self):
        if self.mode == 0:
            if self.intv is None or (type(self.intv) is int and  self.intv < 3):
                self.interval = 3 * np.ones(sampling)
            elif type(self.intv) is list and len(self.intv) != len(self.curr_a):
                self.error_message=f'Number of data in duration does not meet the number of data to be scanned.'
                return False
            elif type(self.intv) is list and len(self.intv) == len(self.curr_a):
                for i in range(len(self.intv)):
                    if self.intv[i] < 3:
                        self.intv[i] = 3
                self.interval = self.intv
        elif self.mode == 1:
            if self.intv is None or (type(self.intv) is int and  self.intv < 30):
                self.interval = 30 * np.ones(sampling)
            elif type(self.intv) is list and len(self.intv) != len(self.curr_a):
                self.error_message=f'Number of data in duration does not meet the number of data to be scanned.'
                return False
            elif type(self.intv) is list and len(self.intv) == len(self.curr_a):
                for i in range(len(self.intv)):
                    if self.intv[i] < 30:
                        self.intv[i] = 30
                self.interval = self.intv
        return True
    
    def cancel_configuration(self):
        self.RF.reset()
        self.RF.ChannelOff(self.Achannel)
        self.DC.query(CMD="SET", PAR="VSET", CH=Hchannel[-1], VAL=0)
        self.DC.query(CMD="SET", PAR="ISET", CH=Hchannel[-1], VAL=0)
        time.sleep(0.5)
        self.DC.query(CMD="SET", PAR="OFF", CH=Hchannel[-1])
        
    def RF_set(self, VAL):
        self.RF.set_dc_fix_value(voltage = VAL, current=200, channel=self.Achannel)
        
    def DC_set(self, VAL):
        
        self.DC.query(CMD="SET", PAR="VSET", CH=self.Hchannel[-1], VAL=VAL)

    def DC_scan(self, Arange: list, Hrange: list, Astep=None, Hstep=None, intv=None, samp=None, Achannel:str = None, Hchannel:str = None):
            '''
            Input
            Arange    param: Org and Dest of RF, a list
            Hrange    param: Org and Dest for H, a list
            Astep     param: a list of specified value (Not include org and dest, order insensitive) or just a constant
            Hstep     param: a list of specified value (Not include org and dest, order insensitive) or just a constant
            intv      param: time constant, integer or list of integers
            samp      param: # of data points, when step of each one is specified,
            Achannel  param: device channel
            Hchannel  param: device channel
            '''
            
            # Check Valid Input Working Mode
            self.working_message = "Working Mode Checking..."
            
            self.working_message = "Working Mode Configuration Complete."
            time.sleep(0.1)
            
            self.working_message = "Working Mode Configuration Complete."
            
            # Check Input Channel
            if Achannel is None or Hchannel is None:
                self.error = True
                raise ValueError(f'Please specify the output channel of both power supply.')
                
            curr_v = [] # monitored value of RF amplitude
            curr_u = [] # monitored valur of DC voltage
            
            # Turn on the output port for supply
            self.DC.query(CMD="SET", PAR="ON", CH=Hchannel[-1])
            time.sleep(5)
            self.DC.query(CMD="SET", PAR="ISET", CH=Hchannel[-1], VAL=300)
            self.DC.query(CMD="SET", PAR="VSET", CH=Hchannel[-1], VAL=curr_q[0])
            if curr_q[0] <= 3000:
                time.sleep(30) # Waiting for CAEN to be set to the starting value
            else:
                time.sleep(60)
            self.RF.ChannelOn(channel=Achannel)
            #plt.ion()
            
            for i in range(sampling):
                self.RF.set_dc_fix_value(curr_a[i], 2, channel=Achannel)
                self.DC.query(CMD="SET", PAR="VSET", CH=Hchannel[-1], VAL=curr_q[i])
                #H.send_command(CMD="SET", PAR="ON", CH=Hchannel[-1])
                #time.sleep(30)
                
                for _ in range(int(interval[i])):
                    # Do not try to measure output with Power supply
                    Hmon = H.get_single_channel_parameter("VMON", Hchannel[-1])
                    if type(Hmon) != float:
                        continue
                        
                    curr_v.append(RF.read_dc_v(channel = Achannel))    
                    time.sleep(1)
                    
            # After the entire process, close the output of both device
            self.DC.query(CMD="SET", PAR="VSET", CH=0, VAL=0)
            self.DC.query(CMD="SET", PAR="ISET", CH=0, VAL=0)
            time.sleep(0.5)
            self.DC.query(CMD="SET", PAR="OFF", CH=Hchannel[-1])
            self.RF.ChannelOff(channel=Achannel)
            self.RF.reset()
            self.complete = True

