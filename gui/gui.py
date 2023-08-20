from PyQt6 import uic
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QTimer

import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox.backends import PyPlot

from gui.end_pos import EndPosition
from model.arm import EiT_arm
from xbox_controller import XboxController
from model.controller import Controller

from dataclasses import dataclass

@dataclass
class GUI:
    arm: EiT_arm
    ctr: Controller
    xbox_ctrl: XboxController = None
    update_dt: float = 0.05
    inc: float = 0.02 #increments for movement on button press
    inc_a: float =  5*np.pi/180 #angular increments
    inc_analog: float = 0.01 #increments for joystick

    def __post_init__(self):
        Form, Window = uic.loadUiType("gui/view.ui")
        self.app = QApplication([])
        self.window = Window()
        self.form = Form()
        self.form.setupUi(self.window)

        ##init environment
        figure = plt.figure() # Create figure with subplots
        self.form.simulationLayout.addWidget(figure.canvas)

        env = PyPlot.PyPlot()
        env.launch(name="EiT environment", fig=figure)  # lauches a second plot for some reason
        env.add(self.arm, options={'jointaxislength':0.1})
        plt.close()  # closes second plot
        env.ax.set_xlim(-0.4, 0.4)
        env.ax.set_ylim(-0.4, 0.4)
        env.ax.set_zlim(0, 0.8)

        self.ep = EndPosition(self.arm.fkine(self.arm.q).A, env.ax, reach=self.arm.length) #the end position axes in the plot
        self.env = env

        self.initialize_view()

        # Initialize QTimer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(int(self.update_dt*1000)) #how often to update (in ms, dt is in s)


    def start(self):
        self.window.show()
        self.app.exec()


    def update(self): # update plot periodically
        self.env.robots[0].draw()
        self.ep.draw()

        if self.form.follow.isChecked():
            self.ctr.set_position(self.ep.get_pos())

    def rotateViewElev(self, x):
        self.env.ax.elev += 5*x

    def rotateViewAzim(self, x):
        self.env.ax.azim += 5*x

    def set_sliders(self):
        ang = self.arm.q_degrees().astype(int)
        self.form.q1_slider.setValue(ang[0])
        self.form.q2_slider.setValue(ang[1])
        self.form.q3_slider.setValue(ang[2])
        self.form.q4_slider.setValue(ang[3])
        self.form.q5_slider.setValue(ang[4])


    def slider_change(self):
        arr = [self.form.q1_slider.value(),
                self.form.q2_slider.value(),
                self.form.q3_slider.value(),
                self.form.q4_slider.value(),
                self.form.q5_slider.value()]

        self.form.q1.setText(str(arr[0]))
        self.form.q2.setText(str(arr[1]))
        self.form.q3.setText(str(arr[2]))
        self.form.q4.setText(str(arr[3]))
        self.form.q5.setText(str(arr[4]))

        qs = np.asarray(arr)*np.pi/180
        self.arm.qr = qs  #Sets reference q


    def tab_change(self, tabIndex):
        if self.xbox_ctrl is not None:
            self.register_xbx_funcs(tabIndex) #change what xbox controller does

        if tabIndex == 0:
            self.ctr.disable()
            self.ep.disable()
            self.set_sliders()
        elif tabIndex == 1:
            self.ep.set_pos(self.arm.fkine(self.arm.q).A)
            self.ctr.set_position(self.ep.get_pos())
            self.ep.enable()
            self.ctr.enable()


    def initialize_view(self):
        # Set up angle slider 1
        self.form.q1_slider.setMinimum(-135)
        self.form.q1_slider.setMaximum(135)
        self.form.q1_slider.valueChanged.connect(self.slider_change)

        # Set up angle slider 2
        self.form.q2_slider.setMinimum(-135)
        self.form.q2_slider.setMaximum(135)
        self.form.q2_slider.valueChanged.connect(self.slider_change)

        # Set up angle slider 3
        self.form.q3_slider.setMinimum(-135)
        self.form.q3_slider.setMaximum(135)
        self.form.q3_slider.valueChanged.connect(self.slider_change)

        # Set up angle slider 4
        self.form.q4_slider.setMinimum(-135)
        self.form.q4_slider.setMaximum(135)
        self.form.q4_slider.valueChanged.connect(self.slider_change)

        # Set up angle slider 5
        self.form.q5_slider.setMinimum(-135)
        self.form.q5_slider.setMaximum(135)
        self.form.q5_slider.valueChanged.connect(self.slider_change)
        
        self.arm.qr = self.arm.q #set reference to current position

        self.set_sliders()
        # Add modes to mode_select
        self.form.mode_select.addItems(["Auto", "Position", "Optimization"])
        self.form.mode_select.setCurrentIndex(0)
        # add callbacks to buttons
        self.form.x_up.clicked.connect(lambda: self.ep.translate(self.inc, 0, 0))
        self.form.y_up.clicked.connect(lambda: self.ep.translate(0, self.inc, 0))
        self.form.z_up.clicked.connect(lambda: self.ep.translate(0, 0, self.inc))
        self.form.x_cc.clicked.connect(lambda: self.ep.rotate(self.inc_a, 0, 0))
        self.form.y_cc.clicked.connect(lambda: self.ep.rotate(0, self.inc_a, 0))
        self.form.z_cc.clicked.connect(lambda: self.ep.rotate(0, 0, self.inc_a))
        self.form.x_down.clicked.connect(lambda: self.ep.translate(-self.inc, 0, 0))
        self.form.y_down.clicked.connect(lambda: self.ep.translate(0, -self.inc, 0))
        self.form.z_down.clicked.connect(lambda: self.ep.translate(0, 0, -self.inc))
        self.form.x_c.clicked.connect(lambda: self.ep.rotate(-self.inc_a, 0, 0))
        self.form.y_c.clicked.connect(lambda: self.ep.rotate(0, -self.inc_a, 0))
        self.form.z_c.clicked.connect(lambda: self.ep.rotate(0, 0, -self.inc_a))

        self.form.tabWidget.currentChanged.connect(self.tab_change) #tabs
        self.form.mode_select.currentIndexChanged.connect(lambda: self.ctr.change_mode(self.form.mode_select.currentText())) #rullgardin

        self.form.set_goal.clicked.connect(lambda: self.ctr.set_position(self.ep.get_pos())) #set-goal button
        self.form.follow.stateChanged.connect(lambda: self.form.set_goal.setEnabled(not self.form.follow.isChecked())) #enable/disable button on check

        self.form.set_goal.setEnabled(False)
        self.form.follow.setChecked(True) #have follow mode enabled as default

        self.form.tabWidget.setCurrentIndex(0) #set to tab 0 as default


    def inc_arm_ref(self, i, x):
        if abs(x) < 0.15:
            return
        
        sliders = [self.form.q1_slider, self.form.q2_slider, self.form.q3_slider, self.form.q4_slider, self.form.q5_slider]
        labels = [self.form.q1, self.form.q2, self.form.q3, self.form.q4, self.form.q5]


        new_qr = self.arm.qr[i] + self.inc_analog*x
        q_deg = (new_qr*180/np.pi + 0.5).astype(int)

        
        if abs(q_deg) < 135:
            self.arm.qr[i] = new_qr 
            sliders[i].setSliderPosition(q_deg)
            labels[i].setText(str(q_deg))


    def register_xbx_funcs(self, mode):
        for code in ['ABS_X', 'ABS_Y', 'ABS_RX', 'ABS_RY', 'BTN_TR', 'BTN_TL', 'BTN_NORTH', 'BTN_SOUTH', 'BTN_WEST', 'BTN_EAST', 'BTN_START']:
            self.xbxCtrl.unregister_event_function(code)

        if mode == 0:
            self.xbxCtrl.register_event_function('BTN_TL', lambda x: self.inc_arm_ref(0, 5*x)) #bumper behind, 3 to give greater effect
            self.xbxCtrl.register_event_function('BTN_TR', lambda x: self.inc_arm_ref(0, -5*x)) #bumper behind
            self.xbxCtrl.register_event_function('ABS_Y', lambda x: self.inc_arm_ref(1, x)) #left joystick up/down
            self.xbxCtrl.register_event_function('ABS_X', lambda x: self.inc_arm_ref(2, x)) #left joystick left/right
            self.xbxCtrl.register_event_function('ABS_RY', lambda x: self.inc_arm_ref(3, x)) #right joystick up/down
            self.xbxCtrl.register_event_function('ABS_RX', lambda x: self.inc_arm_ref(4, x)) #right joystick left/right

        elif mode == 1: #follow end position
            self.xbxCtrl.register_event_function('ABS_Y', lambda x: self.ep.translate(self.inc_analog*x, 0, 0)) #left joystick up/down
            self.xbxCtrl.register_event_function('ABS_X', lambda x: self.ep.translate(0, self.inc_analog*x, 0)) #left joystick left/right
            self.xbxCtrl.register_event_function('ABS_RY', lambda x: self.ep.translate(0, 0, self.inc_analog*x)) #right joystick up/down

            self.xbxCtrl.register_event_function('BTN_WEST', lambda _: self.ep.rotate(self.inc_a, 0, 0)) #X-button, rotate X ccw
            self.xbxCtrl.register_event_function('BTN_NORTH', lambda _: self.ep.rotate(0, self.inc_a, 0)) #Y-Button, rotate Y ccw
            self.xbxCtrl.register_event_function('BTN_TR', lambda _: self.ep.rotate(0, 0, self.inc_a))  #right bumper, rotate Z ccw

            self.xbxCtrl.register_event_function('BTN_SOUTH', lambda _: self.ep.rotate(-self.inc_a, 0, 0)) #A-button, rotate X cw
            self.xbxCtrl.register_event_function('BTN_EAST', lambda _: self.ep.rotate(0, -self.inc_a, 0)) #B-Button, rotate Y cw
            self.xbxCtrl.register_event_function('BTN_TL', lambda _: self.ep.rotate(0, 0, -self.inc_a))  #left bumper, rotate Z cw
            self.xbxCtrl.register_event_function('BTN_START', lambda v: self.form.mode_select.setCurrentIndex(
                                                                (self.form.mode_select.currentIndex()+1)%3) if v == 1 else None) #start button, set goal to current position
        else:
            raise ValueError(f"Invalid mode encountered in register_xbx_funcs. MODE: {mode}")