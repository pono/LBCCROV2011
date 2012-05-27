##############################################################
# File: ModuleTestCommand.py                                 #
# Written by Nathan Murrow, Jonah Brooks, and Keith Smee for #
# use with Linn-Benton Community College's ROV Team's        #
# "Phoenix" in the MATE 2011 international ROV competition   #
# at the NASA Johnson Space Center in Houston, Texas from    #
# June 16 - 18, 2011. The LBCC ROV Team is free to use and   #
# modify this code for use in any future activities related  #
# to ROVs.                                                   #
#       June 14, 2011                                        #
##############################################################
# Main driver program for the threading/modular layout       #
##############################################################
# This program imports the two main modules, ROV_Graphical   #
# and ROV_Communications, and launches two threads to manage #
# them. All the magic happens in those two threads~          #
##############################################################
#             Full program changes/updates:                  #
# Updated controller interface and GUI by Keith Smee         #
# New queue system and updated structure by Nathan Murrow    #
# New modular program layout by Jonah Brooks                 #
#                   Dec. 17th 2010                           #
##############################################################

import ROV_Communications, ROV_Control
import thread
import time

#starts threads for communications and GUI
try:
    thread.start_new_thread( ROV_Communications.communications, () )
    time.sleep(0.1)
    thread.start_new_thread( ROV_Control.control, () )
except:
    print "Error: unable to start thread"

#waits while the threads are executing
while ROV_Communications.exitFlag == False and ROV_Control.exitFlag == False:
    pass
