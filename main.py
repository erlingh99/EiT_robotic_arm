import threading
import time
import numpy as np

from model.arm import EiT_arm
from model.controller import Controller

from gui.gui import GUI

from Arduino.Driver import Driver
from xbox_controller import XboxController

##PROGRAM FLAGS
SIMULATE = True #if True then no interfacing with hardware, and motion is simulated
USE_XBX_CTR = False

##SIM PARMS
dt = 0.1 # controller time steps, how often new qd is calculated
update_dt = dt/4 # how often plots are updated

##INIT ARM MODEL
init_pos = np.array([0, 45, -90, 0, -45])*np.pi / 180
arm = EiT_arm(q0=init_pos)  # initial pose

## CONTROL
ctr = Controller(arm, arm.fkine(init_pos).A, dt) #arm controller when in end-position mode
ctr.start()

## XBOX
xbxCtrl = None
if USE_XBX_CTR:
    xbxCtrl = XboxController()

#GUI setup
gui = GUI(arm, ctr, xbxCtrl, update_dt=update_dt)


## HARDWARE
if not SIMULATE:
    driver = Driver(arm, xbxCtrl, dt, ctr) ##assign to variable to avoid garbage collection?
else: 
    ## init simulation
    def q_change():  # only for simulation
        while True:
            if ctr.enabled:
                arm.q = np.clip(arm.q + update_dt*arm.qd, arm.q_lims[0, :], arm.q_lims[1, :]) #Encoder values. Here simulated by forward euler integration
            else:
                arm.q = arm.qr
            time.sleep(update_dt)

    t = threading.Thread(target=q_change, daemon=True)
    t.start()

gui.start()