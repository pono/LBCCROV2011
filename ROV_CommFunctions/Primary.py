##############################################################
# File: ROV_CommFunctions\Primary.py                         #
# Written by Nathan Murrow, Jonah Brooks, and Keith Smee for #
# use with Linn-Benton Community College's ROV Team's        #
# "Phoenix" in the MATE 2011 international ROV competition   #
# at the NASA Johnson Space Center in Houston, Texas from    #
# June 16 - 18, 2011. The LBCC ROV Team is free to use and   #
# modify this code for use in any future activities related  #
# to ROVs.                                                   #
#       June 14, 2011                                        #
###############################################################
# Module for communicating with the primary microcontroller   #
###############################################################
# This file is the set of functions designed to initialize,   #
# maintain, reestablish, and use an Ethernet connection to    #
# the primary microcontroller on the ROV.                     #
###############################################################
# Complete overhaul of this module to use a mandatory         #
# "send/receive pair" to detect network failure without       #
# pinging superfluous packets or constantly reconnecting.     #
#                   Dec. 28th, 2010                           #
## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ###
# Original "proof of concept" module created by Jonah Brooks  #
#                   Dec. 20th, 2010                           #
## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ###
# Note: init() establishes a NEW connection to primary        #
#       reconnect() closes and reopens a connection to primary#
#       exchange() sends arg to primary, returns received data#
###############################################################

import socket

socketToPrimary = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
COMMAND_LENGTH = 6
DEFAULT_TIMEOUT = 1.5   # seconds to wait for connection on sends/receives
PrimaryIP = '192.168.0.177'   # IP of the Primary Arduino server
PrimaryPort = 23              

haltCommand = bytearray(6)
isConnected = False

#############################################################################
#                               init()                                      #
#   Initializes a new connection to the Primary microcontroller             #
#   Cycles endlessly waiting for a connection while no connection exists    #
#                                                                           #
#   WARNING: ties up the communications thread 'til connection established  #
#                                                                           #
#   Receives: nothing                                                       #
#   Returns: nothing                                                        #
#############################################################################
def init():
    global socketToPrimary, PrimaryIP, PrimaryPort, isConnected, haltCommand
    global COMMAND_LENGTH, DEFAULT_TIMEOUT

    haltCommand[0] = 'M'
    for i in range (1,6):
        haltCommand[i] = 0
            
    
    print "Connecting to the Primary microcontroller..."
    socketToPrimary.settimeout(5)
    if(reconnect()):
        socketToPrimary.settimeout(DEFAULT_TIMEOUT)
    else:
        print "\tError in establishing Ethernet connection:"
        print "\tCheck your TCP/IP settings (user defined IP address required"
        print "\tCheck the Ethernet cable as well"
        print "Listening for new connections, stand by..."
        print ""
    
    while(isConnected == False):
        reconnect()
    print "Connection to Primary microcontroller established"
    socketToPrimary.settimeout(DEFAULT_TIMEOUT)


#############################################################################
#                               reconnect()                                 #
#   Closes any existing connections to Primary and attempts to reopen them  #
#   Cycles endlessly waiting for a connection while no connection exists    #
#                                                                           #
#   WARNING: ties up the communications thread 'til connection established  #
#                                                                           #
#   Receives: nothing                                                       #
#   Returns: nothing                                                        #
#############################################################################    
def reconnect():
    global isConnected, DEFAULT_TIMEOUT
    global socketToPrimary, haltCommand

    
        # Establish a new connection
    isConnected = False
    while isConnected == False:
        try:
            #print "reconnecting..."
            socketToPrimary.close()
            socketToPrimary = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            socketToPrimary.settimeout(DEFAULT_TIMEOUT)
            socketToPrimary.connect((PrimaryIP, PrimaryPort))
        except Exception, e:
            pass#print "Error in reconnecting",e #pass
        else:
            isConnected = True
            exchange(haltCommand)

        
    return isConnected


#############################################################################
#                               close()                                     #
#   Closes the connection to the Primary microcontroller                    #
#                                                                           #
#   Receives: nothing                                                       #
#   Returns: nothing                                                        #
#############################################################################
def close():
    global socketToPrimary, isConnected
    socketToPrimary.close()
    isConnected = False


#############################################################################
#                               exchange()                                  #
#   Sends and receives the latest data sets to and from Primary             #
#   Automatically reconnects in the event of a connection failure           #
#                                                                           #
#   Receives: an array of data to send to Primary as an argument (arg)      #
#   Returns: an array of data received from Primary (None if none received) #
#############################################################################
def exchange(arg):
    global socketToPrimary, isConnected
    global COMMAND_LENGTH

    receivedData = None
    
    if isConnected:
        try:
            socketToPrimary.send(arg)
        except Exception, e:
            print "Error in exchange(), sending error: ",e
            isConnected = False

        try:
            receivedData = socketToPrimary.recv(1024)  # change length later
        except Exception, e:
            print "Error in exchange(), receiving error: ",e
            isConnected = False
            reconnect()
    else:
        reconnect()
    
    return receivedData
