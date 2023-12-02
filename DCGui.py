from port_ctrl import DCPort
import pyvisa as visa
import time



if __name__ == "__main__":

    #programmable DCpower usb address for visa
    address1 = "USB0::0x1AB1::0x0E11::DP8B242401816::INSTR" #DP832A
    address2 = "USB0::0x1AB1::0x0E11::DP8E234100254::INSTR" #DP821A
    address3 = "USB0::0x1AB1::0x0E11::DP8E244300633::INSTR"
    address4 = ""



    # DCport1 = DCPort(address1)
    # DCport2 = DCPort(address2)
    DCport1 = DCPort(address1)

    # Turn off all output and timer output at initialization
    DCport1.timer_stop()

    DCport1.set_dc_fix_value(10, 0.1, channel="CH1")

    # DCport3.set_dc_fix_value(0.6, 1.5, channel="CH2")
    DCport1.ChannelOn(channel="CH1")
    # DCport3.normal_display()
    #DCport1.linear_scan_dc(org=0.5, dest=6, interval=1, sampling=25, channel="CH1")

    '''
    DCport3.wav_display()
    DCport3.channel_change()
    voltage3 = DCport3.voltage_acquire("CH1")
    print(voltage3)
    '''
    #DCport3.reset()

    port1_voltage = DCport1.voltage_acquire("CH1")
    print(port1_voltage)

