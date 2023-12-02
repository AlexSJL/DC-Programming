import pyvisa as visa
import time
import numpy as np
from matplotlib import pyplot as plt

class Rigol_DCPort():
    def __init__(self, Rigol_address):
        self.Rigol_address = Rigol_address
        rm = visa.ResourceManager()
        try:
            self.RigolPort = rm.open_resource(self.Rigol_address)
            
            if self.RigolPort.query("*IDN?"):  # /*查询电源ID字符串以检测远程通信是否正常*/
                print("Reference DC power successfully initialized! The address is {}".format(self.Rigol_address))
        except Exception as e:
            print(repr(e))
            
    #------------------------------------------- Following Content Are for Rigol Control Only-----------------------------------------#
    def channel_change(self, channel: str = 'CH1', ):
        self.RigolPort.write(':APPLy {}'.format(channel))

    def set_dc_fix_value(self, voltage, current, channel: str = 'CH1'):
        self.RigolPort.write(':APPLy {}, {},{}'.format(channel, voltage, current))  # 设置电流和电压

    def ChannelOn(self, channel="Ch1"):
        if channel == "CH1":
            self.RigolPort.write(':OUTP CH1,ON ')  # /*打开 CH1 的输出*/
        elif channel == "CH2":
            self.RigolPort.write(':OUTP CH2,ON ')  # /*打开 CH2 的输出*/
        elif channel == "CH3":
            self.RigolPort.write(':OUTP CH3,ON ')  # /*打开 CH2 的输出*/
            
    def ChannelOff(self, channel="Ch1"):
        if channel == "CH1":
            self.RigolPort.write(':OUTP CH1,OFF ')  # /*Turn off Output of CH1*/
        elif channel == "CH2":
            self.RigolPort.write(':OUTP CH2,OFF ')  # /*Turn off Output of CH2*/
        elif channel == "CH3":
            self.RigolPort.write(':OUTP CH3,OFF ')  # /*Turn off Output of CH2*/
    
    # Reading current output voltage
    def read_dc_v(self, channel = "CH1"):
        read_data = self.RigolPort.query(':APPL? {}, VOLT'.format(channel))
        return read_data
    
    def timer_linear_scan_dc(self, org=0, dest=6, sampling=60, interval=5, channel="CH1"):
        '''
        :param org: minimum output voltage, unit: volt
        :param dest: maximum output voltage, unit: volt
        :param sampling: number of points in scan
        :param interval: time duration at each voltage, unit:second
        :param channel: channel to output
        '''
        # Overall Initialization
        curr_v = org
        curr_t = 0
        v = []
        t = []
        
        step = (dest - org) / sampling # Voltage Increment step
        if channel == "CH1":
            self.RigolPort.write(':INST CH1')  # /*Choosing channel 1*/
        elif channel == "CH2":
            self.RigolPort.write(':INST CH2')  # /*Choosing channel 2*/
        elif channel == "CH3":
            self.RigolPort.write(':INST CH3')  # /*Choosing channel 3*/

        # Turn on Output, Start amplitude scan
        if channel == "CH1":
            self.RigolPort.write(':OUTP CH1,ON ')  # /*Turn on Output of CH1*/
        elif channel == "CH2":
            self.RigolPort.write(':OUTP CH2,ON ')  # /*Turn on Output of CH2*/
        elif channel == "CH3":
            self.RigolPort.write(':OUTP CH3,ON ')  # /*Turn on Output of CH3*/
        
        plt.ion()
        
        # Temporarily Here
        for i in range(sampling):
            self.set_dc_fix_value(curr_v, 0.2, channel)
            v.append(curr_v)
            t.append(curr_t**2.)
            
            # Update for plot
            plt.clf()
            plt.title("DC Voltage vs RF Amplitude")
            plt.plot(v,t)
            plt.xlabel("RF Freq Q (kV)")
            plt.ylabel("DC Voltage U (kV)")
            plt.grid()
            plt.pause(0.001)
            plt.ioff()
            curr_v += step
            curr_t += interval
            time.sleep(interval)
        plt.show()
        
        # Turn off output, Terminate Scan
        if channel == "CH1":
            self.RigolPort.write(':OUTP CH1,OFF ')  # /*Turn on Output of CH1*/
        elif channel == "CH2":
            self.RigolPort.write(':OUTP CH2,OFF ')  # /*Turn on Output of CH2*/
        elif channel == "CH3":
            self.RigolPort.write(':OUTP CH3,OFF ')  # /*Turn on Output of CH2*/

    def timer_stop(self):
        self.RigolPort.write(':TIME OFF')
        self.RigolPort.write(':OUTPut OFF')

    def sin_signal(self):
        self.RigolPort.write(':INST CH1')  # /*选择通道 CH1*/
        self.RigolPort.write(':TIME:GROUP 1')  # /*设置输出组数：25*/
        self.RigolPort.write(':TIME:CYCLE N,1')  # /*设置循环数：20*/
        self.RigolPort.write(':TIME:ENDS OFF ')  # /*设置终止状态：最后一组*/
        self.RigolPort.write(':TIME:TEMP:SEL SINE')  # /*选择模板：Sine*/
        self.RigolPort.write(':TIME:TEMP:OBJ V,0.2')  # /*选择编辑对象为电压，并将电流设置为 2A*/
        self.RigolPort.write(':TIME:TEMP:MAXV 8')  # /*设置最大值：8V*/
        self.RigolPort.write(':TIME:TEMP:MINV 0')  # /*设置最小值：0V*/
        self.RigolPort.write(':TIME:TEMP:POINT 10')  # /*设置总点数：25*/
        self.RigolPort.write(':TIME:TEMP:INTE 1')  # /*设置时间间隔：5s*/
        self.RigolPort.write(':TIME:TEMP:INVE ON')  # /*打开反相*/
        self.RigolPort.write(':TIME:TEMP:CONST')  # /*构建定时参数*/
        self.RigolPort.write(':MEM:STOR RTF,1')  # /*将已编辑的定时参数保存在内部存储器中*/
        self.RigolPort.write(':OUTP CH1,ON')  # /*打开 CH1 的输出*/
        self.RigolPort.write(':TIME ON')  # /*打开定时输出*/

    def reset(self):
        self.RigolPort.write('*RST;*CLS')  # 仪器复位

    def test(self, channel='CH1'):
        self.RigolPort.query(':MEAS? CH1')

    def voltage_acquire(self, channel='CH1'):
        vol = self.RigolPort.query(':MEASure:VOLTage:DC? {}'.format(channel))
        return vol

    def cur_acquire(self, channel='CH1'):
        current = self.RigolPort.query(':MEASure:CURRent:DC? {}'.format(channel))
        return current

    def power_acquire(self, channel='CH1'):
        power = self.RigolPort.query(':MEASure:POWEr:DC? {}'.format(channel))
        return power

    def channel_acquire(self, channel='CH1'):
        voltage, current, power = self.RigolPort.query(':MEASure:ALL:DC? {}'.format(channel))
        return voltage, current, power

    def wav_display(self):
        self.RigolPort.write(":DISP:MODE WAVE")  # panel display mode，NORMAL、WAVE、DIAL or CLASSIC。

    def normal_display(self):
        self.RigolPort.write(":DISP:MODE NORMAL")
    #--------------------------------------------------End Of Rigol Control-------------------------------------------------------------#