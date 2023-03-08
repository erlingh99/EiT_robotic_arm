from arm_sim.init_robot import EiT_arm
from xbox_controller import XboxController
import threading
import serial
import serial.tools.list_ports
import time
import re
import numpy as np


class Controller(XboxController):
    def __init__(self, form):
        
        self.arm = EiT_arm()
        self.form = form
        ports = serial.tools.list_ports.comports()

        if (len(ports) == 0):
            print("No serial ports found")
            exit()
        
        for port in ports:
            if ("Arduino Uno" in port.description):
                print("Arduino Uno found on port: " + port.device)
                comport = port.device
        self.ser = serial.Serial(comport, 9600, timeout=0.5)

        x = threading.Thread(target=self._readDataThread, daemon=True)
        x.start()   

        y = threading.Thread(target=self._sendDataThread, daemon=True)
        y.start()  

        # form.onButton.clicked.connect(lambda: ser.write(str.encode('<Servo1: ' + servo6  + "; " + 'Servo2: ' + servo2 + "; " + 'Servo3: ' + servo3 + "; " + 'Servo4: ' + servo4 + "; " + 'Servo5: ' + servo5 + "; " + 'Servo6: ' + servo6 + ";>")))
        # form.offButton.clicked.connect(lambda: ser.write(str.encode('<Servo1: 90; Servo2: 60; Servo3: 90; Servo4: 90; Servo5: 45; Servo6: 73;>')))

        super().__init__()
        time.sleep(2)


    def _readDataThread(self):        
        while True:
            if self.ser.in_waiting > 0:                
                line = self.ser.readline()
                if line:
                    curr_degs = self.arm.q_degrees()
                    claw_deg = self.arm.claw_deg()

                    string = line.decode()
                    if "<Ready>" in string:
                        print("Connection established")
                    if "ServoPos1" in string:
                        curr_degs[0] = re.findall(r'\d+', string)[1]
                        #print("ServoPos1:", curr_degs[0])
                    if "ServoPos2" in string:
                        curr_degs[1] = re.findall(r'\d+', string)[1]        
                        #print("ServoPos2:", curr_degs[1])
                    if "ServoPos3" in string:
                        curr_degs[2] = re.findall(r'\d+', string)[1]                        
                        #print("ServoPos3:", curr_degs[2])
                    if "ServoPos4" in string:
                        curr_degs[3] = re.findall(r'\d+', string)[1]                        
                        #print("ServoPos4:", curr_degs[3])
                    if "ServoPos5" in string:
                        curr_degs[4] = re.findall(r'\d+', string)[1]                        
                        #print("ServoPos5:", curr_degs[4])
                    if "ServoPos6" in string:
                        claw_deg = re.findall(r'\d+', string)[1]                        
                        #print("ServoPos6:", claw_deg)

                    self.arm.q_degrees(curr_degs)
                    self.arm.claw_deg(claw_deg)
                
                    

    def _sendDataThread(self):

        prevServo1 = 0
        prevServo2 = 0
        prevServo3 = 0
        prevServo4 = 0
        prevServo5 = 0
        prevServo6 = 0

        while True:
            q_degrees = np.rint(self.arm.q_degrees()).astype(int)
            # claw_deg = np.rint(self.arm.claw_deg()).astype(int)

            controller_state = self.read()
            
            servo1 = str(int(np.interp(controller_state[2],[-1,1],[0,180])))
            servo2 = str(q_degrees[1])
            servo3 = str(q_degrees[2])
            servo4 = str(q_degrees[3])
            servo5 = str(q_degrees[4])
            servo6 = str(int(np.interp(controller_state[5],[0,1],[10,73])))

            

            if prevServo1 != servo1 or prevServo2 != servo2 or prevServo3 != servo3 or prevServo4 != servo4 or prevServo5 != servo5 or prevServo6 != servo6:
                print(str.encode("<" + servo1 + ", " + servo2 + ", " + servo3 + ", " + servo4 + ", " + servo5 + ", " + servo6 + ">"))
                self.ser.write(str.encode("<" + servo1 + ", " + servo2 + ", " + servo3 + ", " + servo4 + ", " + servo5 + ", " + servo6 + ">"))
                prevServo1 = servo1
                prevServo2 = servo2
                prevServo3 = servo3
                prevServo4 = servo4
                prevServo5 = servo5
                prevServo6 = servo6
                
            # b'<90,45,180,180,90,10>'

            #time.sleep(0.1)