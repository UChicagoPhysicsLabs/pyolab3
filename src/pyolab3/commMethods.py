#
# This file is part of PyOLab. https://github.com/matsselen/pyolab
# (C) 2017 Mats Selen <mats.selen@gmail.com>
#
# SPDX-License-Identifier:    BSD-3-Clause
# (https://opensource.org/licenses/BSD-3-Clause)
#

# system stuff
import time
import serial
import serial.tools.list_ports
import binascii
from threading import Thread

# local stuff
from .pyolabGlobals import G


"""
This file contains a set of methods that allows a user to communicate 
with the IOLab system via the virtual com port. 

"""

def getIOLabPortName():
    '''
    Returns a list of names of the serial ports that
    the OS thinks has an IOLab dongle is plugged into them
    '''

    # get a list of all serial ports
    ports = list(serial.tools.list_ports.comports())
    
    # loop over all of the ports found and get the name of 
    # the last one that is an IOLab USB virtual com port
    
    p = ''
    pList = []
    for port in ports:
         pInfo = list(port)
         if G.logData:
            G.logFile.write("\npInfo: "+str(pInfo))
         if 'IOLab' in pInfo[1] or 'PID=1881' in pInfo[2]:
            pList.append(pInfo)

    nFound = len(pList)

    if nFound == 0:
        print("Found no IOLab USB Dongles")
        if G.logData:
            G.logFile.write("\nFound no IOLab USB Dongles")
    
    else:

        p = pList[0][0]
        if nFound == 1:
            if G.logData:
                G.logFile.write("\nFound IOLab USB Dongle: " + p)
        else:
            if G.logData:
                G.logFile.write("\nWarning: found " +str(nFound)+ " IOLab USB Dongles.")
                G.logFile.write("\nUsing the first one found: " + str(p))

    return p

def openIOLabPort(pName):
    '''Opens the IOLab com port that has name pName'''

    # open the com port
    serialport = serial.Serial(pName)
    serialport.baudrate = 115200
    serialport.timeout  = 1
    G.port = serialport
    return serialport

def packet_2_hex(packet):
    '''convenience function, shows bytes as separated hex characters'''
    return (binascii.b2a_hex(packet," ")).upper()

def p2h(packet):
    '''shorthand version of packet_2_hex'''
    return packet_2_hex(packet)

def parse_packet(packet,validate = True):
    '''
    parses the payload of a packet in accordance with section 3.0 of the usb interface specs.

    Arguments:
    ----------
    packet : bytestring 
        The packet sent to or from the receiver
    validate : bool, optional
        Used to verify that the packet is properly formed according to the interface specifications.

    Returns:
    --------
    payload : bytestring
        The data contained within the packet
    '''
    if (validate == True) and not(packet[0] == 0x02 and packet[-1]==0x0A):
        raise ValueError("malformed packet, incorrect terminating characters. Expected 0x02 and 0x0A, got {} and {}".format(packet[0],packet[-1]))
    cmd = packet[1]
    lenp = packet[2]
    payload = packet[3:-1]
    if ((validate == True) and len(payload)>50):
        raise ValueError("payload too long, should be 50 bytes at most but is {}.".format(len(payload)))
    if (validate == True) and not(len(payload) == lenp):
        raise ValueError("payload ({}) should be {} bytes but got {}".format(p2h(packet),lenp,len(payload)))
    return payload

def parse_dongle_status(status):
    '''splits the dongle status response packet into its constituent parts'''
    if not(len(status) == 6):
        raise ValueError("expect 6 byte packet but got {}".format(len(status)))
    fw = int.from_bytes(status[0:2],byteorder='big')
    mode = status[2]
    id = p2h(status[3:])
    return fw,mode,id

def return_dongle_status():
    '''convenience function to get the dongle status with a single command'''
    getDongleStatus(G.serialPort)
    packet = parse_packet(G.serialPort.readline())
    return parse_dongle_status(packet)

def parse_pairing_status(status,validate = True):
    '''splits the pairing status response packet into its constituent parts'''
    if ((validate == True) and not(len(status) == 8 or len(status)==13)):
        raise ValueError("expect 8 or 13 byte packet but got {}".format(len(status)))
    r1_status = status[0]
    r1_id = p2h(status[1:4])
    r2_status = status[4]
    r2_id = p2h(status[5:8])
    freq = None
    if len(status)==13:
        freq = status[-4:]
    return (r1_status,r1_id,r2_status,r2_id,freq)

#=======================================================================
# This next bunch of routines sends commands to the IOLab remote via
# the serial port "s". For a description of the data packets that are returned
# by each one see the USB Interface Specification document 
# (Indesign document number 1814F03 Revision 11, available on the IOLab web page at
#  http://www.iolab.science/Documents/IOLab_Expert_Docs/IOLab_usb_interface_specs.pdf)

def getDongleStatus(s):
    '''Ask the dongle to send a data packet of type 0x14 telling us its status'''

    command = 0x14
    command_record = [0x02, command, 0x00, 0x0A] 
    s.write(bytearray(command_record))
    time.sleep(G.sleepCommand)  #give the serial port some time to receive the data

def startData(s):
    '''
    Start data acquisition.
    The response will be an ACK packet if successful, or NACK packet if not.
    The remote will asynchronously start sending data packets in the format 
    described by record returned by the "getPacketConfig" command. 
    The asynchronous data packets all have the same format and are identified by record type 0x41. 
    '''

    command = 0x20
    command_record = [0x02, command, 0x00, 0x0A]
    s.write(bytearray(command_record))
    time.sleep(G.sleepCommand)  #give the serial port some time to receive the data
    
def stopData(s):
    '''
    Stop data acquisition. 
    The response will be an ACK packet if successful, or NACK packet if not.
    '''

    command = 0x21
    command_record = [0x02, command, 0x00, 0x0A]
    s.write(bytearray(command_record))
    time.sleep(G.sleepCommand)  #give the serial port some time to receive the data

def setSensorConfig(s,idValueList,remote):
    '''
    Sends a sensor configuration record to the selected remote. 
    The response will be an ACK packet if successful, or NACK packet if not. 
    '''

    nPairs  = len(idValueList)/2 
    payload = [remote,nPairs]+idValueList
    nBytes  = len(payload)

    command = 0x22
    command_record = [0x02, command, nBytes] + payload + [0x0A] 

    s.write(bytearray(command_record))
    time.sleep(G.sleepCommand)  #give the serial port some time to receive the data

def getSensorConfig(s,remote):
    '''Gets a sensor configuration record from the selected remote. '''

    command = 0x23
    command_record = [0x02, command, 0x01, remote, 0x0A] 
    s.write(bytearray(command_record))
    time.sleep(G.sleepCommand)  #give the serial port some time to receive the data
 
def setOutputConfig(s,idValueList,remote):
    '''
    Sends an output configuration record to the selected remote.
    The response will be an ACK packet if successful, or NACK packet if not.
    '''

    nPairs  = int(len(idValueList)/2)
    payload = [remote,nPairs]+idValueList
    nBytes  = len(payload)
#    print(nPairs)
#    print(nBytes)
    command = 0x24
    command_record = [0x02, command, nBytes] + payload + [0x0A] 

    s.write(bytearray(command_record))
    time.sleep(G.sleepCommand)  #give the serial port some time to receive the data
 
def getOutputConfig(s,remote):
    '''Gets an output configuration record from the selected remote.'''

    command = 0x25
    command_record = [0x02, command, 0x01, remote, 0x0A] 
    s.write(bytearray(command_record))
    time.sleep(G.sleepCommand)  #give the serial port some time to receive the data
 
def setFixedConfig(s,config,remote):
    '''
    Ask remote to set the current sensor configuration to "config". 
    The response will be an ACK packet if successful, or NACK packet if not.
    '''

    command = 0x26
    command_record = [0x02, command, 0x02, remote, config, 0x0A] 
    s.write(bytearray(command_record))
    time.sleep(G.sleepCommand)  #give the serial port some time to receive the data

def getFixedConfig(s, remote):
    '''Ask remote to send a data packet of type 0x27 telling us the current sensor configuration '''

    command = 0x27
    command_record = [0x02, command, 0x01, remote, 0x0A] 
    s.write(bytearray(command_record))
    time.sleep(G.sleepCommand)  #give the serial port some time to receive the data

def getPacketConfig(s, remote):
    '''
    Ask remote to send a data packet of type 0x28 telling us the format of the 
    data packets that will be sent to us when acquisition is started
    '''
    command = 0x28
    command_record = [0x02, command, 0x01, remote, 0x0A] 
    s.write(bytearray(command_record))
    time.sleep(G.sleepCommand)  #give the serial port some time to receive the data

def getCalibration(s, sensor, remote):
    '''Ask remote to send a data packet of type 0x29 containing calibration information from sensor.'''

    command = 0x29
    command_record = [0x02, command, 0x02, remote, sensor, 0x0A] 
    s.write(bytearray(command_record))
    time.sleep(G.sleepCommand)  #give the serial port some time to receive the data

def getRemoteStatus(s, remote):
    '''Ask remote to send a data packet of type 0x2a telling us its status'''

    command = 0x2A
    command_record = [0x02, command, 0x01, remote, 0x0A] 
    s.write(bytearray(command_record))
    time.sleep(G.sleepCommand)  #give the serial port some time to receive the data

def powerDown(s,remote):
    '''
    Power down remote. 
    The response will be an ACK packet if successful, or NACK packet if not.
    '''

    command = 0x2B
    command_record = [0x02, command, 0x01, remote, 0x0A]
    s.write(bytearray(command_record))
    time.sleep(G.sleepCommand)  #give the serial port some time to receive the data

def sendIOLabCommand(s,command_record):
    '''This is a generic command '''

    s.write(bytearray(command_record))
    time.sleep(G.sleepCommand)  #give the serial port some time to receive the data