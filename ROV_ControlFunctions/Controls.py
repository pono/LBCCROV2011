##############################################################
# File: ROV_ControlFunctions\Controls.py                     #
# Written by Nathan Murrow, Jonah Brooks, and Keith Smee for #
# use with Linn-Benton Community College's ROV Team's        #
# "Phoenix" in the MATE 2011 international ROV competition   #
# at the NASA Johnson Space Center in Houston, Texas from    #
# June 16 - 18, 2011. The LBCC ROV Team is free to use and   #
# modify this code for use in any future activities related  #
# to ROVs.                                                   #
#       June 14, 2011                                        #
##############################################################
# Main set of Pygame-related controller and GUI functions    #
##############################################################
# Phil's Pygame Utilities (PGU) was used in the creation of  #
# this GUI software. PGU's website can be found at:          #
#   http://code.google.com/p/pgu/                            #
#                                                            #
# The initial call to this module sets up the GUI and the    #
# joystick device. After that, each execution of the main    #
# Refresh() process also runs the standard PGU GUI process,  #
# app.loop(). Additionally, user inputs are processed for    #
# sending through the communications modules.                #
##############################################################
# This is our least tidy code. It contains the entire GUI    #
# and has undergone many revisions during testing. It would  #
# be beneficial to clean it up a bit or maybe even write the #
# whole thing again from scratch now that we know where we   #
# want to go with it.                                        #
##############################################################

import pygame, math, sys, socket
from numpy import interp
from pygame.locals import *
from visual import *

#PGU is Phil's Pygame Utilities (used for our GUI)
from pgu import gui

#Joy stores whether a joystick is found
Joy = False

#Number of data points to sample in averaging
pdatapoints = 50

#The following variables are "quick fixes" for variations in thruster speed
Limiter = 0.84
Motormax = 255 * Limiter
Motormin = -255 * Limiter
LRmultiplier = 1 / Limiter

#These flags ensure proper 
pitchFlag = False
revFlag = False
yawFlag = False
Buttonflag = False
Buttonflag2 = False
Buttonflag3 = False
Buttonflag4 = False
Buttonflag5 = False
CG_arm = False
WC_arm = False
CandT_arm = False
PID_arm = False
PID_hover_flag = False
PID_godepth_flag = False

#Controller values
XS1_val = 0
YS1_val = 0
YS2_val = 0
XS2_val = 0
hat_val = (0,0)
theta = 0

#Pressure readings values, up to 100 stored
depth = 0
targetdepth = 0
depthsensor = [0,0,0,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0,0,0]
depthindex = 0
zerodepth = 0

#PID constants
kP = 60
kI = 10
kD = 20

#Objects related to Quarter/Half/Full Speed control modes
speedFlag = 0
speedVal = 1
speedLbl = None

#These arrays are sent to the microcontroller as data
Motorval = ['M',0,0,0,0,0]
Solval = ['S',0,0,0,0,0,0,0,0,0,0,0]
PIDdata = ['P', PID_arm, int(depth/256), depth%256, kP, kI, kD]

#------------------------------------------------------------------------------
# Methods of this class.
#------------------------------------------------------------------------------
def emergeClick():
    print "hello"

#Enables PID Hover mode
def pidHover():
    global PID_hover_flag, PID_godepth_flag
    PID_godepth_flag = False
    PID_hover_flag = True
    print "Hover: ", PID_hover_flag

#Sends a target depth to the microcontroller
def pidGoDepth():
    global targetdepth, depInput, PID_godepth_flag, PID_hover_flag
    #targetdepth = int(depthToDataConv(int(depInput.value)))
    targetdepth = float(depthToDataConv("0" + depInput.value))
    PID_godepth_flag = True
    PID_hover_flag = False
    print "GoDepth: ", targetdepth

#Determines which type of PID control to send to the microcontroller
def getPID_command():
    global PID_arm, PID_hover_flag, PID_godepth_flag
    myreturn = 0
    #print PID_arm, PID_hover_flag, PID_godepth_flag
    if(PID_arm == True):
        if(PID_hover_flag == True):
            myreturn = 2
        elif(PID_godepth_flag == True):
            myreturn = 3
        else:
            myreturn = 1
    #print myreturn
    return myreturn

#Manually establishes the pressure reading of the surface
def setZero():
    global zerodepth
    zerodepth = avgData()

#takes an average of all pressure data points
def avgData():
    global depthsensor, pdatapoints
    value = 0.0
    x = 0
    while x < pdatapoints:
        value = value + depthsensor[x]
        x = x + 1
    return value / pdatapoints

#add a new data point to the array
def addDepth(value):
    global depthsensor, depthindex
    average = avgData()
    depthsensor[depthindex] = value
    ##if (abs(value - average) < 50): 
    ##    depthsensor[depthindex] = value

#convert pressure data to pressure in kPa (not correct as is, more testing needed)
def dataToPresConv():
    global zerodepth
    #P = 0.1821*(avgData() - zerodepth) - 59.1508 #kPa
    P = 0.1480*(avgData() - zerodepth)
    return 0

#convert pressure data to depth in meters (not correct as is, more testing needed)
def dataToDepthConv():
    return ((avgData() - zerodepth) * 0.026) + 0.4

#convert a requested depth to a target value for PID
def depthToDataConv(value):
    value = float(value) - 0.4
    #value = 9.7276 * value #Conversion: Depth to Pressure
    #value = 6.7568 * value #Conversion: Pressure to Data
    value = value / 0.026
    value = value + zerodepth
    return value

#resets the PID defaults
def pidDefault():
    print "PID DEFAULT"
    setP(60)
    setI(10)
    setD(20)

#given an x and a y coordinate, determines the angle
def makeAngle (x,y):
    try:
        beta = degrees(atan(y/x))
        if(x < 0): #quad 2 and 3
            beta += 180

        elif(x >= 0 and YS1_val < 0): # Quad 4
            beta += 360

    except:
        if(y > 0):
            beta = 90
        else:
            beta = 270

    return beta

#--------------------------------------------------------------------------------
#Getters and setters.
#--------------------------------------------------------------------------------

def getPitchFlag ():
    global pitchFlag
    return pitchFlag

def setPitchFlag(x):
    global yawSwitch, yawFlag, pitchFlag
    pitchFlag = x
    yawFlag = False
    yawSwitch.value = False

def getPID_arm():
    global PID_arm
    return PID_arm

def setPID_arm(x):
    global PID_arm
    global PID_hover_flag, PID_godepth_flag
    PID_godepth_flag = False
    PID_hover_flag = True
    PID_arm = x
    
#-------------------------------------------------------------------------------

def getspeedVal():
    global speedVal
    return speedVal

def setspeedVal(x):
    global speedVal
    speedVal = x
    
#-------------------------------------------------------------------------------

def getMotormax ():
    global Motormax
    return Motormax
    
#-------------------------------------------------------------------------------

def getMotormin():
    global Motormin
    return Motormin

#-------------------------------------------------------------------------------

def setMotorMax (Max): #changes the motor max and min that are referenced for the interp function
    Motormax = Max.value
    Motormin = -(Max.value + 1)
    print(Motormax)
    print(Motormin)

#------------------------------------------------------------------------------

def speedCycle():
    global speedVal, Motormax, Motormin
    global app, speedLbl
    speedVal = speedVal * 2
    if(speedVal > 4):
        speedVal = 1

    speedLbl.value = "1/" + str(speedVal)

#-------------------------------------------------------------------------------        

def setMotorValLR(x, y):# Sets the value for the motors depending on where the position of the joystick.
    global speedVal, Motormax, Motormin, Motorval
    
    motor1 = 0
    motor2 = 0

    # Dead zone ---------------------------------
    if(abs(x) < 16): 
        x = 0

    if(abs(y) < 16):
        y = 0
    
    VecA = (x, y)
    value = mag(VecA)

    if(value > Motormax):
        value = Motormax
        
    alpha = makeAngle(x,y)
    
        
    # Forward ------------------------------------
    if(alpha >= 80 and alpha <= 100): 
        motor1 = value
        motor2 = value

    #alpha goes from -90 to 270, so two hard
    #reverse cases are needed

    # Reverse ------------------------------------
    elif(alpha >= 260 and alpha <= 270):
        motor1 = -value
        motor2 = -value
        
    # Reverse ------------------------------------
    elif(alpha >= -90 and alpha <= -80):
        motor1 = -value
        motor2 = -value

    # hard right ---------------------------------
    elif(alpha >= -10 and alpha <= 10):
        motor1 = value
        motor2 = -value

    # hard left ----------------------------------
    elif(alpha >= 170 and alpha <= 190): 
        motor1 = -value
        motor2 = value

    # Varfible right forward ---------------------
    elif(alpha > 10 and alpha < 80): 
        motor1 = value
        motor2 = cos(2*radians(alpha))*-value

    # Varible left forward -----------------------
    elif(alpha > 100 and alpha < 170): 
        motor1 = cos(2*radians(alpha) - math.pi)*value
        motor2 = value

    # varible left reverse ----------------------
    elif(alpha > 190 and alpha < 260): 
        motor1 = -value
        motor2 = cos(2*(radians(alpha) - math.pi))*value
        
    # Varible right reverse -----------------------
    elif(alpha + 360 > 280 and alpha + 360 < 350):
        motor1 = -value*(cos(2*radians(alpha-270)))
        motor2 = -value

    # Check to make sure the values arn't bigger the the motor max 
    if(motor1 > Motormax):
        motor1 = Motormax
        
    elif(motor1 < Motormin):
        motor1 = Motormin

    if(motor2 > Motormax):
        motor2 = Motormax
        
    elif(motor2 < Motormin):
        motor2 = Motormin

    #interp for the speed cycle
    motor1 = round(interp(motor1,[Motormin, Motormax], [Motormin/speedVal, Motormax/speedVal]), 0)
    motor2 = round(interp(motor2,[Motormin, Motormax], [Motormin/speedVal, Motormax/speedVal]), 0)

    #adjust L/R motors to run at higher speeds (quick fix)
    motor1 = int(motor1 * LRmultiplier)
    motor2 = int(motor2 * LRmultiplier)
    if (motor1 > 255):
        motor1 = 255
    if (motor2 > 255):
        motor2 = 255
    if (motor1 < -255):
        motor1 = -255
    if (motor2 < -255):
        motor2 = -255

    Motorval[1] = motor1
    Motorval[2] = motor2
    #print(Motorval[1])
    #print(Motorval[2])
    
    #--------------------------------------------
    # End of setMotorValLR
    #--------------------------------------------

def setMotorValFB (x, y):
    global pitchFlag, yawFlag, speedVal, Motormax, Motormin, Motorval
    
    motor3 = 0
    motor4 = 0

    if (yawFlag == True):
        y = x
   
    # Dead zone ---------------------------------
    if(abs(y) <= 16): 
        y = 0
        
    VecB = (0, y)
    value = mag(VecB)
    
    if(value > Motormax):
        value = Motormax

    # Asend/Pitch up ----------------------------
    if(y > 10):
        if(pitchFlag == True or yawFlag == True):
            motor3 = value
            motor4 = -value
        else:
            motor3 = value
            motor4 = value

    # Desend/Pitch down -------------------------
    elif(y < -10):
        if(pitchFlag == True or yawFlag == True):
            motor3 = -value
            motor4 = value
        else:
            motor3 = -value
            motor4 = -value

    #interp for the speed cycle
    motor3 = round(interp(motor3,[Motormin, Motormax], [Motormin/speedVal, Motormax/speedVal]), 0)
    motor4 = round(interp(motor4,[Motormin, Motormax], [Motormin/speedVal, Motormax/speedVal]), 0)
            
    Motorval[3] = motor3
    Motorval[4] = motor4
    #print(Motorval[3])
    #print(Motorval[4])

    
    #--------------------------------------------
    # End of setMotorValFB
    #--------------------------------------------

def arm_disarm_Component(comp): #Changes a component flag to it's opposite.
    global PID_arm, pitchFlag
    if(comp == 0):
        setPID_arm(not PID_arm)
        print("PID is " + str(PID_arm))
    elif (comp == 4):
        setPitchFlag(not pitchFlag)
        print("pitch is " + str(pitchFlag))

    
#------------------------------------------------------------------------------
# The following methods sync the GUI with the joystick when turning different
# control modes on and off.
#------------------------------------------------------------------------------
def SCButton():
    global Buttonflag, Joysticks

    if(Buttonflag == False and Joysticks.get_button(4) == True):
        speedCycle()
        Buttonflag = True
        
    elif(Buttonflag == True):
        if(Joysticks.get_button(4) == False):
            Buttonflag = False
            print Buttonflag


#------------------------------------------------------------------------------
def pitchButton():
    global Joysticks, Buttonflag2, pSwitch

    if(Buttonflag2 == False and Joysticks.get_button(5) == True):
        print "step 1"
        arm_disarm_Component(4)
        pSwitch.value = not pSwitch.value
        print pSwitch.value
        Buttonflag2 = True
        
    elif(Buttonflag2 == True):
        if(Joysticks.get_button(5) == False):
            Buttonflag2 = False

def PIDButton():
    global Joysticks, Buttonflag4, pidSwitch

    if(Buttonflag4 == False and Joysticks.get_button(6) == True):
        print "PIDButt"
        arm_disarm_Component(0)
        pidSwitch.value = not pidSwitch.value
        print pidSwitch.value
        Buttonflag4 = True
        
    elif(Buttonflag4 == True):
        if(Joysticks.get_button(6) == False):
            Buttonflag4 = False
            
def revButton():
    global Joysticks, Buttonflag3, revSwitch

    if(Buttonflag3 == False and Joysticks.get_button(7) == True):
        print "revButt"
        revSwitch.value = not revSwitch.value
        reversify()
        print revSwitch.value
        Buttonflag3 = True
        
    elif(Buttonflag3 == True):
        if(Joysticks.get_button(7) == False):
            Buttonflag3 = False
            
            
def yawButton():
    global Joysticks, Buttonflag5, yawSwitch

    if(Buttonflag5 == False and Joysticks.get_button(1) == True):
        print "yawButt"
        yawSwitch.value = not yawSwitch.value
        yawify()
        print yawSwitch.value
        Buttonflag5 = True
        
    elif(Buttonflag5 == True):
        if(Joysticks.get_button(1) == False):
            Buttonflag5 = False
    
def reversify():
    global revSwitch, revFlag
    revFlag = not revFlag
    print revFlag

def yawify():
    global yawSwitch, yawFlag, pitchFlag, pSwitch
    yawFlag = not yawFlag
    pitchFlag = False
    pSwitch.value = False
    print yawFlag
            
#------------------------------------------------------------------------------
# The meat of the GUI is in the next method. It's mostly just setting up tables
# within tables so that everything lines up right. Very tedious stuff.
#------------------------------------------------------------------------------

def GuiSetup():
    global app
    global MotorMin, MotorMax
    global LSlider, RSlider, FSlider, ASlider
    global speedLbl, depLbl, preLbl, dataLbl, depInput
    global pidSwitch, pSwitch, revSwitch, yawSwitch, Pbox, Ibox, Dbox

    app = gui.Desktop()
    app.connect(gui.QUIT, app.quit, None)

    Main = gui.Table(width = 900)
    Main.tr()

    # c.tr() makes a row
    # c.td(component) creates a cell

    # Top panel for displaying network status and emergancy options
    n = gui.Table()
    n.tr()

    n.td(gui.Label("Network: Status"), height = 40)
    n.td(gui.Label("       "))
    emergeButton = gui.Button("In Case of Emergency")
    emergeButton.connect(gui.CLICK, emergeClick)
    n.td(emergeButton)

    #Make sub tabels to be put into c that hold seperate elements.

    #tabel on the "west" side of the frame.
    w = gui.Table()
    w.tr()
    #Radio button group
    w.tr()
    w1 = gui.Table()

    w1.tr()
    w1.td(gui.Label("PID Controls"), colspan = 5, height = 50)
    w1.tr()
    pidSwitch =gui.Switch(value = False)
    pidSwitch.connect(gui.CLICK, arm_disarm_Component,0)
    w1.td(pidSwitch)
    w1.td(gui.Label("Arm "))
    w1.td(gui.Label(""), width = 95, height = 30)
    pidButton1 = gui.Button("Hover")
    pidButton1.connect(gui.CLICK, pidHover)
    w1.td(pidButton1, align = 1)

    w.td(w1, align = -1)

    w.tr()
    w2 = gui.Table()

    w2.tr()
    depInput = gui.Input(size = 6)
    w2.td(depInput, align = -1)
    w2.td(gui.Label("m "), align = -1, width = 30, height = 30)
    pidButton2 = gui.Button("Go to depth")
    pidButton2.connect(gui.CLICK, pidGoDepth)
    w2.td(pidButton2, align = 1)

    w.td(w2, align = -1)

    #Sliders to control coeficients
    w.tr()
    w3 = gui.Table()

    #P
    w3.tr()
    #w3.td(gui.Label("Stable "))
    Pbox = gui.Input(value = str(kP),height=16,width=120)
    w3.td(Pbox)
    w3.td(gui.Label(" Quick"), align = -1)

    #I
    w3.tr()
    #w3.td(gui.Label("Stable "))
    Ibox = gui.Input(value = str(kI),height=16,width=120)
    w3.td(Ibox)
    w3.td(gui.Label(" Accurate"), align = -1)

    #D
    w3.tr()
    #w3.td(gui.Label("Cautious "))
    Dbox = gui.Input(value = str(kD),height=16,width=120)
    w3.td(Dbox)
    w3.td(gui.Label(" Responsive"), align = -1)

    w.td(w3, align = -1)

    #Restore PID defaults button
    w.tr()
    w4 = gui.Table()

    w4.tr()
    pidButton3 = gui.Button("Restore PID Defaults")
    pidButton3.connect(gui.CLICK, pidDefault)
    w4.td(pidButton3, valign = 1)
    w4.td(gui.Label(""), height = 30)
    
##    w4.tr()
##    pidButton4 = gui.Button("Auto-Calibrate")
##    pidButton4.connect(gui.CLICK, pidCalibrate)
##    w4.td(pidButton4)
##    w4.td(gui.Label(""), height = 30)

    w.td(w4, align = -1)

    w.tr()
    w41 = gui.Table()
    w41.tr()
    zeroButton = gui.Button("Set Zero")
    zeroButton.connect(gui.CLICK, setZero)
    w41.td(zeroButton, valign = 1)
    w41.td(gui.Label(""), height = 30)

    w.td(w41, align = 0)

    #Second panel under top panel for depth, pressure, and subsytems label
    w.tr()
    w5 = gui.Table()

    w5.tr()
    w5.td(gui.Label("Depth "), height = 30, align = -1)
    depLbl = gui.Input("0", size = 6)
    w5.td(depLbl, align = 1)
    w5.td(gui.Label(" m"))
    w5.tr()
    w5.td(gui.Label("Pressure "), align = -1)
    preLbl = gui.Input("0", size = 6)
    w5.td(preLbl, align = 1)
    w5.td(gui.Label(" kPa"))
    w5.tr()
    w5.td(gui.Label("Raw data "), align = -1)
    dataLbl = gui.Input("0", size = 6)
    w5.td(dataLbl, align = 1)

    w.td(w5, align = -1)

    w.tr()

    # Center tabel that holds all the navigation data.
    c = gui.Table()

    #verticle sliders for motor bars and space for surface.
    c.tr()

    c1 = gui.Table()
    c1.tr()
    LSlider = gui.VSlider(value=0,min=Motormin,max=Motormax,size=20,height=120,width=16)
    RSlider = gui.VSlider(value=0,min=Motormin,max=Motormax,size=20,height=120,width=16)
    FSlider = gui.VSlider(value=0,min=Motormin,max=Motormax,size=20,height=120,width=16)
    ASlider = gui.VSlider(value=0,min=Motormin,max=Motormax,size=20,height=120,width=16)
    c1.td(LSlider, width = 20, valign = 1)
    c1.td(RSlider, width = 20, valign = 1)

    c1.td(FSlider, width = 20, valign = 1)
    c1.td(ASlider, width = 20, valign = 1)
    c1.tr()
    c1.td(gui.Label("L"))
    c1.td(gui.Label("R"))
    c1.td(gui.Label("F"))
    c1.td(gui.Label("A"))

    c.td(c1)

    # Motor duty cycle bar.
    c.tr()

    c2 = gui.Table()
    c2.tr()
    c2.td(gui.Label(" Motor Duty Cycle "), height = 50, colspan = 3, valign = 1)
    c2.tr()
    c2.td(gui.Label("0 "), height = 30)
    MotormaxS = gui.HSlider(value=25,min=0,max=127,size=20,height=16,width=120)
    MotormaxS.connect(gui.CHANGE, setMotorMax, MotormaxS) 
    c2.td(MotormaxS)
    c2.td(gui.Label(" 100"))

    c.td(c2)

    #Radio Buttons for controling pitch and button for speed cycle.
    c.tr()

    c4 = gui.Table()
    c4.tr()
    pSwitch =gui.Switch(value = False)
    pSwitch.connect(gui.CLICK, arm_disarm_Component,4)
    c4.td(pSwitch)
    c4.td(gui.Label("Pitch on"))
    c4.td(gui.Label("     "))

    spdCycButton = gui.Button("Speed Cycle")
    spdCycButton.connect(gui.CLICK, speedCycle)
    c4.td(spdCycButton)

    c4.tr()
    revSwitch = gui.Switch(value = False)
    revSwitch.connect(gui.CLICK, reversify)
    c4.td(revSwitch)
    c4.td(gui.Label("Reverse on"))
    speedLbl = gui.Input("1/1", size = 4, align = 1)
    c4.td(speedLbl, colspan = 4, align = 1)
    c.td(c4)
    
    c4.tr()
    yawSwitch =gui.Switch(value = False)
    yawSwitch.connect(gui.CLICK, yawify)
    c4.td(yawSwitch)
    c4.td(gui.Label("Yaw on"))
    c4.td(gui.Label("     "))

    #put it together.
    Main.tr()
    Main.td(n, colspan = 3)
    Main.tr()
    Main.td(gui.Label(""), height = 30)
    Main.tr()
    Main.td(w, valign = -1)
    Main.td(c, valign = -1)
    Main.tr()
    Main.td(gui.Label(""), height = 30)

    app.init(Main) #Main is main table.

#"First run" method, sets up joystick and GUI.
def init():
    global Joy, Joysticks
    pygame.init()

    #-----------------------------------------------------------------------------
    #Main gui window
    #-----------------------------------------------------------------------------
    Joy = True
    print(pygame.joystick.get_count())
    try:
        x = pygame.joystick.get_count()
        if (x == 0):
            print("no Joystick detected by the computer")
            Joy = False
        print(x)
    except:
        print("no Joystick detected in port")
        Joy = False
        
    if(Joy):
        Joysticks = pygame.joystick.Joystick(0)
        Joysticks.init()
        
    GuiSetup()

#------------------------------------------------------------------------------

#"Main loop" method, contains most of the code that is executed in the thread
def Refresh(press_arg): # Called from ROVcontrol to execute all control functions.
    global app, Motormin, Motormax, Motorval, Joy, Joysticks
    global LSlider, RSlider, FSlider, ASlider
    global depLbl, preLbl, dataLbl, revFlag, yawFlag, zerodepth, depthsensor, depthindex, pdatapoints
    global PID_hover_flag, PID_godepth_flag, Pbox, Ibox, Dbox, kP, kI, kD
    global targetdepth
    app.loop()
    pygame.event.pump()

    # Udate PID array.
    kP = int("0" + str(Pbox.value))
    kI = int("0" + str(Ibox.value))
    kD = int("0" + str(Dbox.value))

    #determines the current state of the directional sticks
    if(Joy):
        XS1_val = round(interp(Joysticks.get_axis(0),[-1,1], [Motormin, Motormax]), 0)
        YS1_val = round(-interp(Joysticks.get_axis(1),[-1,1], [Motormin, Motormax]), 0)
        XS2_val = round(interp(Joysticks.get_axis(2),[-1,1], [Motormin, Motormax]), 0)
        YS2_val = round(-interp(Joysticks.get_axis(3),[-1,1], [Motormin, Motormax]), 0)
        SCButton()
        pitchButton()
        revButton()
        yawButton()
        PIDButton()
    else:
        XS1_val = 0
        YS1_val = 0
        XS2_val = 0
        YS2_val = 0

    if (revFlag == True):
        XS1_val = XS1_val * -1
        YS1_val = YS1_val * -1

    #calculates the desired motor speeds based on joystick data
    setMotorValLR(XS1_val, YS1_val)
    setMotorValFB(XS2_val, YS2_val)

    LSlider.value = -Motorval[1]
    RSlider.value = -Motorval[2]
    FSlider.value = -Motorval[3]
    ASlider.value = -Motorval[4]

    #updates pressure data
    if (isinstance(press_arg, int)):
        addDepth(press_arg)
        depthindex = depthindex + 1
        if (depthindex >= pdatapoints):
            depLbl.value = "" + str(round(dataToDepthConv(),2))
            #depLbl.value = "" + str(round(avgData(),2))
            preLbl.value = "" + str(round(dataToPresConv(),2))
            dataLbl.value = avgData()
            depthindex = 0
            #print avgData()
        

    
    
    #---------------------------------------------------------------------------
    # send the values to micro controller.
    #---------------------------------------------------------------------------

    #reverses controls for use in "Critter Gitter" mode.
    if (revFlag == True):
        temp = Motorval[1]
        Motorval[1] = Motorval[2]
        Motorval[2] = temp

    #Fills the motor array to send to the microcontroller
    Motorval[5] = 0
    if(Motorval[1] < 0):
        Motorval[5] += 8
        Motorval[1] = abs(Motorval[1])
    if(Motorval[2] < 0):
        Motorval[5] += 4
        Motorval[2] = abs(Motorval[2])
    if(Motorval[3] < 0):
        Motorval[5] += 2
        Motorval[3] = abs(Motorval[3])
    if(Motorval[4] < 0):
        Motorval[5] += 1
        Motorval[4] = abs(Motorval[4])

    if (yawFlag == True):
        Motorval[5] += 16

    Instruction = bytearray(11)
    Instruction[0] = 'M'

    for i in range(1, 6):
        #[opCode,left, right, front, back, DirectionFlags]
        Instruction[i] = int(round(Motorval[i], 0))

    #Fills the PID array to send to the microcontroller
    PIDdata = bytearray(11)
    PIDdata[0] = 'P'
    PIDdata[1] = int(getPID_command())
    PIDdata[2] = int(int(targetdepth) / 256)
    PIDdata[3] = int(int(targetdepth) % 256)
    PIDdata[4] = kP
    PIDdata[5] = kI
    PIDdata[6] = kD
    #print PIDdata[2] * 256 + PIDdata[3]

    #Resets PID flags
    PID_hover_flag = False
    PID_godepth_flag = False
    
    #print int(PIDdata[1])
    
    #Solval2 = bytearray(11)
    #Solval2[0] = 'S'

    #for i in range(1, 11):
        #Solval2[i] = Solval[i]

    #return (PIDdata, Instruction, Solval2)
    return (PIDdata, Instruction)
    
#--------------------------------------------------------------------------------


##while(True):
##    Refresh(None)
##    pygame.time.wait(10)
