##############################################################
# File: ROV_Communications.py                                #
# Written by Nathan Murrow, Jonah Brooks, and Keith Smee for #
# use with Linn-Benton Community College's ROV Team's        #
# "Phoenix" in the MATE 2011 international ROV competition   #
# at the NASA Johnson Space Center in Houston, Texas from    #
# June 16 - 18, 2011. The LBCC ROV Team is free to use and   #
# modify this code for use in any future activities related  #
# to ROVs.                                                   #
#       June 14, 2011                                        #
##############################################################
# Main Communications functions to interact with GUI/Controls#
##############################################################
# This file is the set of functions designed to handle the   #
# Communications thread, and to interact with the Graphical  #
# thread. This version contains Nathan's new threading code, #
# including his queue system for exchanging commands between #
# Graphics and Communications. It was modified slightly by me#
# to incorporate a new modular layout for our program by     #
# adding hooks for additional Communications modules.        #
# Most of this file is from my older Communications tests,   #
# and has yet to take advantage of the new modular layout.   #
##############################################################
# Modified by Jonah Brooks to use the new module: Primary.py #
#                   Dec. 29th, 2010                          #
## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ##
# New thread locking system implemented by Nathan Murrow     #
#                   Dec. 21st 2010                           #
## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ##
# New queue system and updated structure by Nathan Murrow    #
# New modular program layout by Jonah Brooks                 #
#                   Dec. 17th 2010                           #
## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ##
# Note: To use the ROV_CommFunctions package to store extra  #
# functions, create one or more .py files that contain the   #
# functions you need. Each file can contain any number of    #
# logically related functions and/or variables. To add the   #
# file to the ROV_CommFunctions package, save the .py file in#
# the ROV_CommFunctions folder, then add the name of the file#
# (without the .py) to the list held in the __init__.py file #
# in the ROV_CommFunctions folder. You can then call the     #
# function with the following syntax:                        #
#               fileName.functionName(arguments)             #
# This modularity might be a bit of a hassle right now, but  #
# it should save quite a bit of time and effort in the future#
##############################################################

# communications module for ROV interface
from collections import deque
import time, threading
import ROV_Control
from ROV_CommFunctions import *

#flag to shut down all threads and close program
exitFlag = False

#variables to send commands from GUI to communications
commCommand = ""
commQueue = deque([])
commLock = threading.Lock()

# bytearrays for storing the data to be sent to or received from Primary
toPrimary = bytearray(Primary.COMMAND_LENGTH)
fromPrimary = bytearray(Primary.COMMAND_LENGTH)
#socketToPrimary = socket.socket()  # to be initialized later

#pushes a new graphics command into the queue
def pushCommCommand(arg):
    global commQueue
    global commLock
    commLock.acquire()
    try:
        #print "Testing, pushCommCommand: ", arg
        if(type(arg) == bytearray):
            commQueue.append(arg)
        else:
            print "pushCommCommand error: invalid argument type or length"
    except:
        print "pushCommCommand error: invalid argument type or length"
    commLock.release()

#executes commands from GUI in Communications
def commExecute():
    global commQueue
    global commLock
    #interpret commands here
    #print "Testing, commExecute: ", commCommand  # Placeholder for testing purposes
    commLock.acquire()
    try:
        toPrimary = commQueue.popleft()
        #print toPrimary
        fromPrimary = Primary.exchange(toPrimary)
        if fromPrimary:
            try:
                pass
                #ROV_Graphical.pushGraphicCommand(fromPrimary)
                #print "Sent: ",chr(toPrimary[0]),toPrimary[1],toPrimary[2]\
                      #,toPrimary[3],toPrimary[4],toPrimary[5]\
                      #,"\tReceived: ",ord(fromPrimary[0])
                #print ord(fromPrimary[0]), ord(fromPrimary[1])
                #The following line sends the 10 bit pressure data to the GUI
                ROV_Control.pushControlCommand((int(ord(fromPrimary[0]))*256) + int(ord(fromPrimary[1])))
            except Exception, e:
                print "Error in comms:",e
        #commQueue.clear()
    except socket.error, e:
        print "Error in sending command to ROV: ", e
    commLock.release()

###############process for the Communications thread######################
   
def communications():
    global exitFlag
    global commQueue
    global socketToPrimary
    global DEFAULT_TIMEOUT
    commError = False
    
    #Open connections to ROV here
    print "Connecting to the ROV..."
    Primary.init()
            ## Receiving code ##
    while exitFlag == False:
       #detect and execute commands from ROV
        #executes commands from GUI
        if commQueue:   # if commQueue is not empty
            commExecute()
    
        if commError == True:
            pass
            #reopen connections
        
    #close connections to the ROV    
    Primary.close()

###########end of the main communications thread###############
