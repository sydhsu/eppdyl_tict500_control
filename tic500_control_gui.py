"""
Last update: 5 May 2020
Adapted from 3d3a lab motor_control_gui.py in 2018

User-friendly control interface for the Tic T500 Stepper Motor Controller
for the EPPDyL.

@author: Sydney Hsu
Created in Spring 2020
"""

# -----------------------------------------------------------------------------
    # Import useful packages
# -----------------------------------------------------------------------------
from tkinter import Tk,Toplevel,Menu,PhotoImage,StringVar,N,S,E,W,FALSE,filedialog
from tkinter import ttk
import numpy as np
import motor

DEFAULT_PORT = motor.find_motor()

CONNECTION_ERROR_MSG = 'Motor not connected.'


# -----------------------------------------------------------------------------
    # Class to more easily manage ttk.Label objects with associated
    # StringVars.
    # Can set associated variable by setting the TrackedLabel.value property,
    # or using call syntax.
# -----------------------------------------------------------------------------
class TrackedLabel(ttk.Label):

    def __init__(self,*args,**kwargs):
        self._tracked_var = StringVar()
        self._tracked_var.set(kwargs['text'])
        kwargs['textvariable']=self._tracked_var
        ttk.Label.__init__(self,*args,**kwargs)

    @property
    def value(self):
        return self._tracked_var.get()

    @value.setter
    def value(self,val):
        self._tracked_var.set(val)

    def __call__(self,val):
        self.value = val

class MotorControl(Tk):
    '''
    Tk root object, with controls for motor.
    Opens probe controls.
    '''
    def __init__(self):
        Tk.__init__(self)
        self.option_add('*tearOff',FALSE)
        self.title('Plasma Diagnostics Probe Control')

        #create variable to hold motor object once instantiated
        self.m = None

        # GUI frame
        self.gui_frame = ttk.Frame(self,padding=10)
        self.gui_frame.grid(row=0,column=0)

        # Row 0: Motor address and connection status
        self.motor_port_label = ttk.Label(self.gui_frame,text='Port address:')
        self.motor_port = StringVar()
        self.motor_port.set(DEFAULT_PORT)
        self.motor_port_entry = ttk.Entry(self.gui_frame,textvariable=self.motor_port,width=7)
        self.connect_button = ttk.Button(self.gui_frame,text='Connect',command=self.attempt_connection,width=10)
        self.connect_status = TrackedLabel(self.gui_frame,text='No connection',width=17)

        # Manual controls section
        self.motor_control_label = ttk.Label(self.gui_frame,text='Manual Motor Controls')

        # number of steps entry field
        self.step_label = ttk.Label(self.gui_frame,text='Steps: ')
        self.stepnumber = StringVar()
        self.stepnumber.set('0')
        self.step_entry = ttk.Entry(self.gui_frame,textvariable=self.stepnumber,width=10)

        # speed entry field
        self.speed_label = ttk.Label(self.gui_frame,text='Speed: ')
        self.speednumber = StringVar()
        self.speednumber.set('5000')
        self.speed_entry = ttk.Entry(self.gui_frame,textvariable=self.speednumber,width=10)
        self.set_speed_button = ttk.Button(self.gui_frame,text='Set speed',command=self.motor_set_speed,width=10)

        # buttons
        self.fwd_button = ttk.Button(self.gui_frame,text='Forward',command=self.motor_fwd_step,width=10)
        self.rev_button = ttk.Button(self.gui_frame,text='Reverse',command=self.motor_rev_step,width=10)
        self.stop_button = ttk.Button(self.gui_frame,text='Stop',command=self.motor_stop,width=10)
        self.homing_button = ttk.Button(self.gui_frame,text='Go home',command=self.motor_go_home,width=10)
        self.calibrate_button = ttk.Button(self.gui_frame,text='Calibrate',command=self.motor_calibrate,width=10)

        # Live position readout
        self.position_label = ttk.Label(self.gui_frame,text='Current position:')
        self.current_position = TrackedLabel(self.gui_frame,text='(?)')

        # Assemble GUI
        # Motor port
        self.motor_port_label.grid(row=0,column=0,sticky=E)
        self.motor_port_entry.grid(row=0,column=1,columnspan=4,sticky=W+E)
        self.connect_button.grid(row=0,column=5,sticky=W+E)
        self.connect_status.grid(row=0,column=6,columnspan=2,sticky=W)

        # Manual control section
        self.motor_control_label.grid(row=1,column=2,columnspan=3)
        self.step_label.grid(row=3,column=0,sticky=E)
        self.step_entry.grid(row=3,column=1,sticky=W+E)

        self.speed_label.grid(row=4,column=0,sticky=E)
        self.speed_entry.grid(row=4,column=1,sticky=W+E)
        self.set_speed_button.grid(row=4,column=2,sticky=W+E)

        self.fwd_button.grid(row=3,column=2,sticky=W+E)
        self.rev_button.grid(row=3,column=3,sticky=W+E)
        self.stop_button.grid(row=3,column=4,sticky=W+E)
        self.homing_button.grid(row=4,column=3,sticky=W+E)
        self.calibrate_button.grid(row=4,column=4,sticky=W+E)

        # Current position readout
        self.position_label.grid(row=5,column=6,sticky=E)
        self.current_position.grid(row=5,column=7)

        #bindings, hotkeys, etc.
        self.bind('<Control-w>',self.close)
        self.after(100,self.update_position)

# -----------------------------------------------------------------------------
    # Direct serial commands to motor via motor.py class
# -----------------------------------------------------------------------------

    # stopping
    def motor_stop(self):
        if self.m is not None:
            self.m.stop()
        else:
            print(CONNECTION_ERROR_MSG)
        print('stopping.')

    # calibrating
    def motor_calibrate(self):
        if self.m is not None:
            self.m.calibrate(fname='3d3a_stage_calibration.dat')
        else:
            print(CONNECTION_ERROR_MSG)
        print('calibrating.')

    # connect to port
    def attempt_connection(self):
        if self.m is not None:
            del(self.m)
            self.m = None
        try:
            self.connect_status('Connecting...')
            self.m = motor.motor(port=self.motor_port.get(),calibration_file='3d3a_stage_calibration.dat')
            self.connect_status('Connected')
            self.m.safe_start_off()

        except (motor.serial.SerialException, AttributeError) as e:
            print('Could not connect to motor.')
            self.m = None
            self.connect_status('Could not connect')

    # go forward by k steps
    def motor_fwd_step(self):
        if self.m is not None:
        # steps
            self.m.fwd_step(int(self.stepnumber.get()))
        else:
            print(CONNECTION_ERROR_MSG)
        print("forward by {} steps.".format(str(self.stepnumber.get())))

    # go in reverse by k steps
    def motor_rev_step(self):
        if self.m is not None:
            self.m.rev_step(int(self.stepnumber.get()))
        else:
            print(CONNECTION_ERROR_MSG)
        print("reverse by {} steps.".format(str(self.stepnumber.get())))

    # homing
    def motor_go_home(self):
        if self.m is not None:
            self.m.go_home()
        else:
            print(CONNECTION_ERROR_MSG)
        print("going home.")

    # set speed
    def motor_set_speed(self):
        if self.m is not None:
            self.m.set_speed(int(self.speednumber.get()))
        else:
            print(CONNECTION_ERROR_MSG)
        print("speed set to {} Hz".format(str(self.speednumber.get())))

    # position ranges from 0 to 100, calibrated against voltage
    def update_position(self,repeat=True):
        if self.m is not None and self.m.position is not None:
            position_vals = [self.m.position for i in range(100)]
            position = sum(position_vals)/len(position_vals)
            self.current_position('{:4.2f} cm'.format(position*100))
        else:
            self.current_position('(?)')

        if repeat:
            self.after(100,self.update_position)

    def close(self,*args):
        if self.m is not None:
            del(self.m)
        self.destroy()

if __name__=='__main__':
    root = MotorControl()
    root.mainloop()
