'''
Last update: 5 May 2020
Adapted from 3d3a lab motor.py

Provides calibration and command of the Tic T500 Multi-Interface Stepper Motor Controller.

@author: Sydney Hsu
Created in Spring 2020
'''

# -----------------------------------------------------------------------------
    # Import useful packages
# -----------------------------------------------------------------------------
import serial
from serial.tools import list_ports
import time
import numpy as np
import os

# attempt to locate motor port address
def find_motor():
    '''
    Function to find the motor device address in case we don't know what it is
    and/or the default value doesn't work.
    '''
    all_ports = list_ports.comports()
    port_descriptions = [port.description for port in all_ports]

    try:
        motor_index = port_descriptions.index('Pololu Tic T500 USB Multi-Interface Stepper Motor Controller')
        motor_address = all_ports[motor_index].device

    except ValueError:
        print('Could not find motor.')
        motor_address = None

    return(motor_address)

# -----------------------------------------------------------------------------
    # Motor calibration
# -----------------------------------------------------------------------------
class Calibration:
    def __init__(self,voltage_min,position_min,voltage_max,position_max):
        self.min = voltage_min
        self.max = voltage_max
        self.x_min = position_min
        self.x_max = position_max
        self.distance=position_max-position_min

    @classmethod
    def from_file(cls,fname):
        current_directory = os.path.abspath(os.path.curdir)
        if os.path.exists(current_directory + '/' + fname):
            with open(current_directory + '/' + fname,'r') as f:
                v_min,x_min = list(map(float,f.readline().split(',')))
                v_max,x_max = list(map(float,f.readline().split(',')))

            return cls(v_min,x_min,v_max,x_max)
        else:
            return None

    def save(self,fname):
        current_directory = os.path.abspath(os.path.curdir)
        with open(current_directory + '/' + fname,'w') as f:
            f.writelines(['{},{}\n'.format(self.min,self.x_min),'{},{}\n'.format(self.max,self.x_max)])

# -----------------------------------------------------------------------------
    # The motor object with useful functions.
    # If None is given as an argument for port (not
    # the default value), then find_motor is used to attempt to find it.
# -----------------------------------------------------------------------------
class motor:
    def __init__(self,port='/dev/ttyACM0',calibration_file='motor_calibration.dat'):

        if port == None or port == '':
            port = find_motor()
        try:
            self.usb = serial.Serial(port,9600)
        except serial.SerialException:
            self.usb = None
            raise serial.SerialException('Could not connect to motor.')
        self.start_time = time.time()
        self.calibration = Calibration.from_file(calibration_file)

    def __del__(self):
        if self.usb is not None:
            self.stop()
            self.usb.close()

    # must be calibrated to define its position
    @property
    def position(self):
        if self.calibration is not None:
            return (self.get_analog_input(2) - self.calibration.min)/(self.calibration.max - self.calibration.min)
        else:
            return None

    # serial commands encoded in hexadecimal
    def send_cmd(self,*cmds):
        '''
        Convenience method for sending tuple of hex values to motor controller.
        '''
        cmd_out = ''
        for cmd in cmds:
            cmd_out+=chr(cmd)

        self.usb.write(bytes(cmd_out,'latin-1'))

    def safe_start_off(self):
        '''
        Must send this prior to trying to move the motor for reasons.
        '''
        self.send_cmd(0x83)

    # energize stepper motor coils
    def fwd(self):
        self.send_cmd(0x85)

    # de-energize stepper motor coils
    def rev(self):
        self.send_cmd(0x86)

    def set_speed(self,speed=0xC8):
        '''
        Sets max. starting speed at which acceleration/deceleration is allowed. Default value from Tic is 200 steps per second.
        Speed may be 0-500,000,000 steps per 10,000 seconds (0-50kHz)
        '''
        # convert from Hz to steps per 10,000 sec
        speed = speed * 10000
        if (speed > 500000000 | speed < 0): # don't let speed exceed max
            speed = 50000000 # default for Lin 211-18-01 at 1200 RPM
        # encode speed into bytes
        self.send_cmd(0xE5,(((speed >>  7) & 1) | ((speed >> 14) & 2) |
            ((speed >> 21) & 4) | ((speed >> 28) & 8),
            speed >> 0 & 0x7F,
            speed >> 8 & 0x7F,
            speed >> 16 & 0x7F,
            speed >> 24 & 0x7F))

    def stop(self,braking=32):
        '''
        Stops the motor using the brake command. Braking value assumed max
        unless set.
        Braking may be between 0-32.
        '''
        self.send_cmd(0x92,braking)

    def go_home(self):
        # 7-bit write command 0x97 0x00
        '''
        Homing in the reverse direction, away from the cathode.
        '''
        homing_speed = 50000000 # 5000 Hz; change if needed
        self.send_cmd(0x61,(((homing_speed >>  7) & 1) | ((homing_speed >> 14) & 2) |
            ((homing_speed >> 21) & 4) | ((homing_speed >> 28) & 8),
            homing_speed >> 0 & 0x7F,
            homing_speed >> 8 & 0x7F,
            homing_speed >> 16 & 0x7F,
            homing_speed >> 24 & 0x7F))
        self.send_cmd(0x97,0x00)

    def fwd_step(self,k):
        '''
        Move forward by a specified number of steps.
        '''
        for i in range(0,k):
            if(self.position < self.calibration.max):
                self.fwd()
            else:
                self.stop()
                break

    def rev_step(self,k):
        '''
        Move backwards by a specified number of steps.
        '''
        for i in range(0,k):
            if(self.position > self.calibration.min):
                self.rev()
            else:
                self.stop()
                break

    def get_analog_input(self,channel=1):
        '''
        Gets current value of analog input for the specified channel (1 or 2).
        '''
        self.send_cmd(0xA1,12 if channel == 1 else 16)
        x = (self.usb.read(),self.usb.read())
        out = int.from_bytes(x[0],'big')+256*int.from_bytes(x[1],'big')
        if out == 0xFFFF:
            print('Input channel is disconnected.')
        else:
            out *= 3.3/4095
        return(out)

    # not currently implemented into the GUI
    def go(self,relative_distance,tol=0.01,dwell=1E-4):
        '''
        Moves a specified distance in the units of position used
        in motor calibration.  Returns the end position.
        '''
        start_point = self.position
        end_point = start_point + relative_distance
        err = relative_distance

        while abs(err)>tol:
            err = self.position - end_point
            if err < 0:
                self.fwd()
            elif err > 0:
                self.rev()
            self.wait(dwell)
        self.stop()
        return(self.position)

    # not currently implemented into the GUI
    def goto(self,target,tol=0.01,dwell=1E-4,averaging=False):
        '''
        Moves to the specified target with the given error tolerance.
        '''
        #Check whether target point is inside calibration
        if target > self.calibration.x_max or target < self.calibration.x_min:
            print('Target outside motor range.')
            return self.position

        if averaging:
            start_point = np.mean([self.position for i in range(averaging)])
        else:
            start_point = self.position

        end_point = target
        err = end_point - start_point

        while abs(err)>tol:
            if averaging:
                err = np.mean([self.position for i in range(averaging)]) - end_point
            else:
                err = self.position - end_point

            if err < 0:
                self.fwd()
            elif err > 0:
                self.rev()
            self.wait(dwell)
        return(self.position)

    def wait(self,wait_time=1E-1):
        '''
        Convenience method to ensure pauses between movements to assist in
        acceleration compensation.
        '''
        time.sleep(wait_time)

    def calibrate(self,x_min=0,x_max=1,fname='motor_calibration.dat'):
        '''
        Method to generate motor voltage-position calibration.  Currently
        assumes a linear calibration curve and uses only the two limit switch
        end points.
        '''
        #Move to minimum position
        self.safe_start_off()
        self.rev()
        dv = 100
        while dv>1E-4:
            v_old = self.get_analog_input(2)
            self.wait(.5)
            dv = abs(self.get_analog_input(2)-v_old)

        v_min = sum([self.get_analog_input(2) for i in range(1000)])/1000

        #Move to maximum position
        self.safe_start_off()
        self.fwd()
        dv=100
        while dv>1E-4:
            v_old = self.get_analog_input(2)
            self.wait(.5)
            dv = abs(self.get_analog_input(2)-v_old)

        v_max = sum([self.get_analog_input(2) for i in range(1000)])/1000

        #Generate calibration and save to file
        self.calibration = Calibration(v_min,x_min,v_max,x_max)
        self.calibration.save(fname)

if __name__ == '__main__':
    #Start timer and motor
    m = motor()
    m.safe_start_off()
    m.calibrate()
