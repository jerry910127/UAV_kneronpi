"""
Created on Wed Jan 15 16:50:23 2020

@author: obsidian
"""

import numpy as np
import serial
import time
import re

#%%
def printBytes(bts):
    t=''
    for x in bts:
        t=t+format(x, '02x')+' '
    print(t)
    
#%%    
MAXBUFFER = 4096 
INFORMATIONREGOFFSET = 0x1000

CONTROLREGOFFSET = 0x2000

ADCENABLE = 1#   write 0 to disable, anything else enable
                              
ACCUMULATEFRAMES = 3#        write anything to it to start collecting background frames

AUTOGAIN = 8#
COLORMAPENABLE = 9#
COLORMAPINDEX = 10#

SAVEDEFAULT = 13#

AUTOSHUTTER = 15#
AUTOSHUTTERMAX = 16#
SHUTTERDELAY = 17#


REBOOT = 53#        //write 0 to bit 8 to power down PV and PN
ManualSHUTTERCAL = 54#


CMAP_CONTROL = 0X2B000000
TM_CONTROL = 0x2C000000

#%%
#SAMPLE COMMAND READ/WRITE

class genicam():
    
    def __init__(self, comport="COM5",baud=115200):
        self.ser=serial.Serial(
        	port=comport,
        	baudrate=baud,
        	parity=serial.PARITY_NONE,
        	stopbits=serial.STOPBITS_ONE,
        	bytesize=serial.EIGHTBITS,
            timeout=200
        )
        if(self.ser.is_open!=True):
            raise Exception("cannot open "+ comport)
        self.requestID=0
        
        t=self.readRegister(4,64)
        #print("len==%i"%(len(t)))
        if len(t)>=64:
            #print("st=")
            st=t.decode('utf-8')
            print(st)
            if st.find("OBSIDIAN SENSORS INC.")>=0:
                print("GenICam opened successfully.")
        else:
            self.ser.close()
            raise Exception("not a obsidian genicam port")
        
    def waitBytes(self, bytes2Read, timeoutSeconds=2):
        counter=0
        t=self.ser.read(bytes2Read)
        while(len(t)<bytes2Read and counter < timeoutSeconds ):
           t=t+self.ser.read_all()
           time.sleep(1)
           counter=counter+1
        if counter < timeoutSeconds:
           return t
        else:
           raise Exception("read time out %i after seconds", timeoutSeconds)
           
    def readRegister(self,address, length):
        readCMD=bytearray(20)
        #CCD
        readCMD[0] = 0x00
        readCMD[1] = 0x40  #flags, bit 14 =1 requires an Ack 
        readCMD[2] = 0x00
        readCMD[3] = 0x08  #read, table 7, command identifier

        readCMD[4] = 0x0C
        readCMD[5] = 0x00  #//SCD is 12 bytes long, table 8, ReadMem SCD-fields

        readCMD[6] = self.requestID & 0x00FF
        readCMD[7] = (self.requestID >> 8) & 0xFF  #request_id 
        self.requestID =self.requestID+1
        
        #//SCD
        readCMD[8:16]=address.to_bytes(8,'little')        #//8 bytes address
        readCMD[16] = 0
        readCMD[17] = 0 #two bytes reserved

        readCMD[18] = length & 0xFF
        readCMD[19] = (length >> 8) & 0xFF    #//length to read

        printBytes(readCMD)

        readBuffer=None
        try:
            if not self.ser.isOpen():
                self.ser.open()
            self.ser.read_all()
            self.ser.write(readCMD)
            readBuffer=self.waitBytes( length+8, 20 )
            
        except:
            print("read wrong")
        
        
        if readBuffer[0] == 3 and readBuffer[1] == 0x80 :
            raise Exception("Wrong address to read")
        if readBuffer[0] != 0 or readBuffer[1] != 0:
            raise Exception("read ack flags are not 0")
        if readBuffer[2] != 0x01 or readBuffer[3] != 0x08 :
            raise Exception("read ack is not READMEM_ACK")
        return readBuffer[8:]
    
    
    def writeRegister(self,address, data):
        writeCMD = bytearray(8 + 8 + len(data))

        #//CCD
        writeCMD[0] = 0x00
        writeCMD[1] = 0x40  #//flags, bit 14 =1 requires an Ack 

        writeCMD[2] = 0x02
        writeCMD[3] = 0x08  #//write, table 7, command identifier

        l = len(data)+ 8
        writeCMD[4] = 0xff & l
        writeCMD[5] = 0xff & (l>>8)  #//SCD is 64+data length, table 10, WriteMem SCD-fields

        writeCMD[6] = self.requestID & 0x00FF
        writeCMD[7] = (self.requestID >> 8) & 0xFF  #request_id 
        self.requestID =self.requestID+1
        
        #//SCD        
        writeCMD[8:16]=address.to_bytes(8,'little')        #//8 bytes address
        writeCMD[16:]=data
            
        try:
            self.ser.read_all()
            self.ser.write(writeCMD)
            readBuffer=self.waitBytes( 12, 20 )
        except:
            print("no response")


        if readBuffer[0] != 0 or readBuffer[1] != 0:
            raise Exception("write ack flags are not 0")
        if readBuffer[2] != 0x03 or readBuffer[3] != 0x08:
            raise Exception("write ack is not WRITEMEM_ACK")
        lengthWritten = int.from_bytes(readBuffer[10:], 'little')
        return lengthWritten
    
    
    def reboot(self):
        self.writeRegister(CONTROLREGOFFSET+REBOOT, bytearray([1]))
        
    def stopCapture(self):
        self.writeRegister(CONTROLREGOFFSET+ADCENABLE, bytearray([1]))
    
    def startCapture(self):
        self.writeRegister(CONTROLREGOFFSET+ADCENABLE, bytearray([0]))
    

    def enableAutoGain(self):
        self.writeRegister(CONTROLREGOFFSET+AUTOGAIN, bytearray([1]))
    
    def disableAutoGain(self):
        self.writeRegister(CONTROLREGOFFSET+AUTOGAIN, bytearray([0]))
        self.resetGain()
    
    def setColormap(self,index=0):
        self.writeRegister(CONTROLREGOFFSET+COLORMAPINDEX, bytearray([index]))
    
    def enableColormap(self):
        self.writeRegister(CONTROLREGOFFSET+COLORMAPENABLE, bytearray([1]))
            
    def disableColormap(self):
        self.writeRegister(CONTROLREGOFFSET+COLORMAPENABLE, bytearray([0]))
    
    def saveDefault(self):
        self.writeRegister(CONTROLREGOFFSET+SAVEDEFAULT, bytearray([1]))
    
    def enableAutoShutter(self):
        self.writeRegister(CONTROLREGOFFSET+AUTOSHUTTER, bytearray([1]))
        
    def disableAutoShutter(self):
        self.writeRegister(CONTROLREGOFFSET+AUTOSHUTTER, bytearray([0]))
            
    def setBkgdFrames(self,numFrames=4):
        self.writeRegister(CONTROLREGOFFSET+ACCUMULATEFRAMES, bytearray([numFrames]))
        
    def setShutterDelay(self, delay=4):
        self.writeRegister(CONTROLREGOFFSET+SHUTTERDELAY, bytearray([delay]))
    
    def setAutoShutterGapMax(self, gap=3):
        self.writeRegister(CONTROLREGOFFSET+AUTOSHUTTERMAX, bytearray([gap]))
        
    def getBoardTemp(self):
        return int(self.readRegister(0x1000 + 0, 1)[-1])

    def controlColormap(self):
        t = self.readRegister(CMAP_CONTROL, 1)
        self.writeRegister(CMAP_CONTROL, bytearray([1, 0, 0, 0]))

    def uncontrolColormap(self):
        self.writeRegister(CMAP_CONTROL, bytearray([0, 0, 0, 0]))

    def enableAGC(self):
        self.writeRegister(TM_CONTROL, bytearray([23, 0, 0, 0]))
    
    def disableAGC(self):
        self.writeRegister(TM_CONTROL, bytearray([22, 0, 0, 0]))
    
    def close(self):
        self.ser.close()
        
   
        
# cam.close()
if __name__ == '__main__':
    
    try:
        gc=genicam(comport="/dev/ttyACM0", baud=115200)      
        t = gc.readRegister(0x2009, 1)
        gc.writeRegister(0x2009, bytearray([0]))

    finally:
        if gc:
            gc.close()        