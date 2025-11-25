
"""
Created on Wed Nov 19 15:00:31 2025

@author: Hector Quijada

ATI Net Delta Force Torce Sensor Data communications and file handling


"""

import rospy
import socket
import struct
import time
from datetime import datetime
import numpy as np

class DeltaFtSensor():
    
    def __init__(self, ip_address = "192.168.1.21"):
        self.ip_address = ip_address
        self.port = 49152 #UDP Port for Connection
        
        #Create Socker Instance
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        #Try to connect to ATI Sensor
        try:
            self.sock.connect((self.ip_address, self.port))
            self._send_command(65)
        except Exception as e:
            rospy.logerror("An unexpected error occurred while connecting to ATI sensor: %s",e)
            raise RuntimeError("An unexpected error occurred while connecting to ATI sensor: %s",e)
    
    #Close socket instance in destructor
    def __del__(self):
        self.sock.close()
    
    #Verify connection and try to reconnect
    def reconnect(self):
        
        try:
            self._send_command(65)
            return 0
        except:
            # recreate the socket and reconnect
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.connect((self.ip_address, self.port))
            self._send_command(65)
            return 1
    
    #Send Commands 
    def _send_command(self, cmd):
        
        #0 = Stop Streaming
        #2 = Start high-speed real-time streaming
        #65 = Reset Condition Latch
        #66 = Set Software Bias  
        
        try:
            header = 0x1234
            message = struct.pack('!HHI', header, cmd, 0)
            self.sock.sendto(message, (self.ip_address, self.port))
        except:
            rospy.logerror("Command send but connection not available, please check connection status")
            raise RuntimeError("Command send but connection not available, please check connection status")
    
    def get_data(self):
        rawdata = self.sock.recv(36)
        data = struct.unpack('!IIIiiiiii', rawdata)[3:]
        data = [(data[i]/1e6) for i in range(6)]
        return data
        
    def start_measuring(self):
        
        #Set bias (force measurement zero)
        self._send_command(66)
        #Start Streaming (Start sending bytes to the port)
        self._send_command(2)
        
    def stop_measuring(self):
        
        #Stop Streaming
        self._send_command(0)



#Example on data adquisition
if __name__ == "__main__":
    
    testing_time = 20.0
    
    delta = DeltaFtSensor()
    
    delta.start_measuring()
    
    ft_samples = np.zeros((1,6))
    
    global_time = 0
    i=0
    
    while global_time < testing_time:
        #Obtain data and arrange in a Numpy Array
        start_time = time.perf_counter()
        data = np.array(delta.get_data())
        if i == 0:
            print("Connection established, ft data sampling ongoing")
        ft_samples = np.vstack((ft_samples,data))
        end_time = time.perf_counter()
        i += 1
        
        global_time += (end_time - start_time)
    
    delta.stop_measuring()
    #Save all data to a csv file
    current_datetime = datetime.now()
    ATI_net_path = ("samples_%s.csv",current_datetime)
    np.savetxt(ATI_net_path, ft_samples, delimiter=',', fmt='%f')