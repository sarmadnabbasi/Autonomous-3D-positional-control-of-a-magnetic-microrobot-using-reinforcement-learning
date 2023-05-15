import serial
from time import sleep

class EAS_current_control:
    def __init__(self,  port, baudRate, debug_log=True):
        self.port = port
        self.baudRate = baudRate
        self.debug_log = debug_log
        self.connected = False
        self.connection = None
        try:
             sleep(2)
             self.connection = serial.Serial(self.port, self.baudRate)
             sleep(2)
             if debug_log and self.connection.is_open:
                  self.connected=True
                  print("Connection Established")
        except:
             if debug_log:
               print("Connection with EduMAG Failed")


    def sendPWM(self, pwm):
        self.connection.write(pwm.encode('UTF-8'))
        self.connection.flush()

    def sendStop(self):
        pwm = "0,0,0,0,0,0,0,0\n"
        self.connection.write(pwm.encode('UTF-8'))
        self.connection.flush()

    def send(self, val):
        self.connection.write(val)
        self.connection.flush()

    def getCurrent(self):
        return self.connection.readline()





"""
def connectArduino(port=None, baudRate=None):
    ser = serial.Serial(port, baudRate)
    
def sendData(ser, pwm=None):
    ser.write(pwm.encode('UTF-8'))


def readData(ser):
    return ser.readline();
"""
"""

ser = serial.Serial('COM5', 115200) # Establish the connection on a specific port
counter = 32 # Below 32 everything in ASCII is gibberish
data = "got 1"
data ="100,100,100,100,0"
while True:
     counter +=1
     ser.write(data.encode('UTF-8')) # Convert the decimal number to ASCII then send it to the Arduino
     print (ser.readline()) # Read the newest output from the Arduino
     sleep(1) # Delay for one tenth of a second
     if counter == 255:
          counter = 32
"""