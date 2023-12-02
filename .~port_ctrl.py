import pyvisa as visa
import time

class DCPort():
    def __init__(self, address):
        self.address = address
        rm = visa.ResourceManager()
        try:
            self.RigolPort = rm.open_resource(self.address)
            if self.RigolPort.query("*IDN?"):  # /*查询电源ID字符串以检测远程通信是否正常*/
                print("DC power successfully initialized! The address is {}".format(self.address))
        except Exception as e:
            print(repr(e))


    def channel_change(self, channel:str = 'CH1',):
        self.RigolPort.write(':APPLy {}'.format(channel))


    def set_dc_fix_value(self, voltage, current, channel:str = 'CH1',):
        self.RigolPort.write(':APPLy {}, {},{}'.format(channel, voltage, current))  # 设置电流和电压
    
    def ChannelOn(self,channel = "Ch1"):
        if channel == "CH1":
            self.RigolPort.write(':OUTP CH1,ON ')  # /*打开 CH1 的输出*/
        elif channel == "CH2":
            self.RigolPort.write(':OUTP CH2,ON ')  # /*打开 CH2 的输出*/


    def linear_scan_dc(self, org=0, dest=10, sampling=10, interval=1, channel="CH1"):
        self.RigolPort.write('*RST;*CLS')  # 仪器复位
        if channel == "CH1":
            self.RigolPort.write(':INST CH1') #/*选择通道 CH1*/
        elif channel == "CH2":
            self.RigolPort.write(':INST CH2') #/*选择通道 CH1*/
        self.RigolPort.write(':DELAY:CYCLE N,1')  # /*设置循环数：1*/
        self.RigolPort.write(':DELAY:GROUP 1')  # /*设置输出组数：1，默认值为1*/
        self.RigolPort.write(':TIME:TEMP:SEL UP')  # /*选择模板：UP 连续上升*/
        self.RigolPort.write(':TIME:TEMP:OBJ V, 2000mA')  # /*选择编辑对象为电压，并将电流设置为 200mA*/
        self.RigolPort.write(':TIME:TEMP:MAXV {}'.format(dest))   #  /*设置扫描终点*/
        self.RigolPort.write(':TIME:TEMP:MINV {}'.format(org))   #/*设置扫描起点：0V*/
        self.RigolPort.write(':TIME:TEMP:POINT {}'.format(sampling))  #  /*设置扫描点数*/
        self.RigolPort.write(':TIME:TEMP:INTE {}'.format(interval))  # /*设置时间间隔*/
        self.RigolPort.write(':TIME:TEMP:CONST')  # /*构建定时参数*/
        self.RigolPort.write(':MEM:STOR RTF,1 ')  # /*将已编辑的定时参数保存在内部存储器中，位置为1*/
        if channel == "CH1":
            self.RigolPort.write(':OUTP CH1,ON ')  # /*打开 CH1 的输出*/
        elif channel == "CH2":
            self.RigolPort.write(':OUTP CH2,ON ')  # /*打开 CH2 的输出*/
        self.RigolPort.write(':TIME:ENDS OFF')  # /*设置终止状态：关闭输出*/
        self.RigolPort.write(':TIME ON')  # /*打开定时输出*/
    
    def timer_stop(self):
        self.RigolPort.write(':TIME OFF')  
        self.RigolPort.write(':OUTPut OFF')  

    def sin_signal(self):
        self.RigolPort.write(':INST CH1') #/*选择通道 CH1*/
        self.RigolPort.write(':TIME:GROUP 1') #/*设置输出组数：25*/
        self.RigolPort.write(':TIME:CYCLE N,1') #/*设置循环数：20*/
        self.RigolPort.write(':TIME:ENDS OFF ') #/*设置终止状态：最后一组*/
        self.RigolPort.write(':TIME:TEMP:SEL SINE') #/*选择模板：Sine*/
        self.RigolPort.write(':TIME:TEMP:OBJ V,0.2') #/*选择编辑对象为电压，并将电流设置为 2A*/
        self.RigolPort.write(':TIME:TEMP:MAXV 8') #/*设置最大值：8V*/
        self.RigolPort.write(':TIME:TEMP:MINV 0') #/*设置最小值：0V*/
        self.RigolPort.write(':TIME:TEMP:POINT 10') #/*设置总点数：25*/
        self.RigolPort.write(':TIME:TEMP:INTE 1') #/*设置时间间隔：5s*/
        self.RigolPort.write(':TIME:TEMP:INVE ON') #/*打开反相*/
        self.RigolPort.write(':TIME:TEMP:CONST') #/*构建定时参数*/
        self.RigolPort.write(':MEM:STOR RTF,1') #/*将已编辑的定时参数保存在内部存储器中*/
        self.RigolPort.write(':OUTP CH1,ON') #/*打开 CH1 的输出*/
        self.RigolPort.write(':TIME ON') #/*打开定时输出*/

    def reset(self):
        self.RigolPort.write('*RST;*CLS') #仪器复位

    def test(self,channel='CH1'):
        self.RigolPort.write(':MEAS? CH1')


    def voltage_acquire(self,channel='CH1'):
        vol = self.RigolPort.write(':MEASure:VOLTage:DC? {}'.format(channel))
        return vol

    def cur_acquire(self,channel='CH1'):
        current = self.RigolPort.write(':MEASure:CURRent:DC? {}'.format(channel))
        return current    
    
    def power_acquire(self,channel='CH1'):
        power = self.RigolPort.write(':MEASure:POWEr:DC? {}'.format(channel))
        return power    

    def channel_acquire(self, channel='CH1'):
        voltage, current, power = self.RigolPort.write(':MEASure:ALL:DC? {}'.format(channel))
        return voltage, current, power
    
    def wav_display(self):
        self.RigolPort.write(":DISP:MODE WAVE") #panel display mode，NORMAL、WAVE、DIAL or CLASSIC。

    def normal_display(self):
        self.RigolPort.write(":DISP:MODE NORMAL")


        


