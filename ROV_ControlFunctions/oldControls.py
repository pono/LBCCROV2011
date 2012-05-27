import pygame, math, time
from numpy import interp
from pygame.locals import *
from visual import *

initializedFlag = False
#-------------------------------------------------------------------------------------------------
# Define methods to be used later.
#-------------------------------------------------------------------------------------------------

def init():
    global Motorval, Motormax, Motormin, initializedFlag
    global Joysticks, Quit, Rov_pos, XS1_val,XS2_val,YS1_val,YS2_val
    global Scale, Rov_axis, theta, Last #window, Status, Direction,
    initializedFlag = True
    pygame.init()
    if pygame.joystick.get_count() == 0:
        print "No controller detected."
    else:
        print (pygame.joystick.get_count())
    
    Motorval = [0, 0, 0, 0, 0, 0]
    Last = bytearray(6)
    Joysticks = pygame.joystick.Joystick(0)# Must be in the J drive for Keith's computer
    Joysticks.init()#Every joystick object must be initialized to be used.
    Quit = False
    Rov_pos = (0,0,0)

    XS1_val = 0
    YS1_val = 0
    YS2_val = 0
    XS2_val = 0
    Scale = 127
    Rov_axis = (0,0,-Scale*5)
    theta = 0
    Motormax = 200
    Motormin = -200
##    window = display(title = "ROV 2011 GUI", x = 0, y = 0,
##                     width = 600, hight = 800,
##                     autoscale = 0,
##                     range = (Scale*25, Scale*25, Scale*25),
##                     center = (0,0,0))
##
##    Status = label(pos = (-Scale*15 + 50, Scale*15, 0), text = " Howdy", visible = 1)
##
##    Direction = arrow(pos = Rov_pos, axis = Rov_axis, color = color.red,
##                     shaftwidth = Scale*1.5, fixedwidth = 1)


def setMotorRange(Max, Min): #changes the motor max and min that are referenced for the interp function
    global Motormax, Motormin
    Motormax = Max
    Motormin = Min
    
def setAngle(x, y):
    try:
        alpha = degrees(atan(y/x))
        if(x < 0): #quad 2 and 3
            alpha += 180

        elif(x >= 0 and YS1_val < 0): # Quad 4
            alpha += 360

    except:
        if(y > 0):
            alpha = 90
        else:
            alpha = 270

    return alpha

def setMotorVal(x, y): # Sets the value for the left motor depending on where the position of the joystick.
    global Motorval
    global Motormax, Motormin

    
    # Dead zone ---------------------------------
    if(abs(x) < 40): 
        x = 0 

    if(abs(y) < 40):
        y = 0
        
    VecA = (x, y)
    value = mag(VecA)

    if(value > Motormax):
        value = Motormax
        
    alpha = setAngle(x, y)
        
    # Forward ------------------------------------
    if(alpha >= 80 and alpha <= 100): 
        Motorval[1] = value 
        Motorval[2] = value

    # Reverse ------------------------------------
    elif(alpha >= 260 and alpha <= 280):
        Motorval[1] = -value 
        Motorval[2] = -value

    # hard right ---------------------------------
    elif((alpha >= 0 and alpha <= 10) or (alpha >= 350 and alpha <= 360)):
        Motorval[1] = -value 
        Motorval[2] = value

    # hard left ----------------------------------
    elif(alpha >= 170 and alpha <= 190): 
        Motorval[1] = value 
        Motorval[2] = -value

    # Varfible right forward ---------------------
    elif(alpha > 10 and alpha < 80):  
        Motorval[2] = value 
        Motorval[1] = cos(2*radians(alpha))*value

    # Varible left forward -----------------------
    elif(alpha > 100 and alpha < 170): 
        Motorval[2] = cos(2*radians(alpha) - math.pi)*value
        Motorval[1] = value

    # varible right reverse ----------------------
    elif(alpha > 190 and alpha < 260): 
        Motorval[1] = -value 
        Motorval[2] = cos(2*(radians(alpha) - math.pi))*value
       
    # Varible left reverse -----------------------
    elif(alpha > 280 and alpha < 350): 
        Motorval[1] = -value*(cos(2*radians(alpha-270))) 
        Motorval[2] = -value

    # Check to make sure the valus isn't bigger the the motor max 
    if(Motorval[1] > Motormax):
        Motorval[1] = Motormax        
    elif(Motorval[1] < Motormin):
        Motorval[1] = Motormin

    if(Motorval[2] > Motormax):
        Motorval[2] = Motormax
    elif(Motorval[2] < Motormin):
        Motorval[2] = Motormin

    #--------------------------------------------
    # End of setMotorValL
    #--------------------------------------------

def setVertMotorVal(y):
    global Motormax, Motormin, Motorval
    if y > Motormax:
        y = Motormax
    elif y < Motormin:
        y = Motormin
    if abs(y) < 15:
        y = 0
    Motorval[3] = y
    Motorval[4] = y
            
#---------------------------------------------------------------------------------------------------


#while(Quit == False):
def Refresh():
    global Motorval, Motormax, Motormin, initializedFlag
    global Joysticks, Quit, Rov_pos, XS1_val,XS2_val,YS1_val,YS2_val
    global Scale, Rov_axis, theta, Last #window, Status, Direction,

    
    pygame.event.pump()
    #rate(25)

    #setMotorRange(5, -5)

    XS1_val = round(interp(Joysticks.get_axis(0),[-1,1], [Motormin, Motormax]), 0) #Rounds to 3 decimal places. will be replaced with an interp funtion later
    YS1_val = round(interp(-Joysticks.get_axis(1),[-1,1], [Motormin, Motormax]), 0)
    XS2_val = round(interp(Joysticks.get_axis(2),[-1,1], [Motormin, Motormax]), 0)
    YS2_val = round(interp(-Joysticks.get_axis(3),[-1,1], [Motormin, Motormax]), 0)

    Instruction = bytearray(6)
    Instruction[0] = 0
    Instruction[1] = 0
    Instruction[2] = 0
    Instruction[3] = 0
    Instruction[4] = 0
    Instruction[5] = 0

##    if (XS2_val < -200):
##        Instruction[0] = 'L'
##        Instruction[1] = 1
##    elif (XS2_val > 200):
##        Instruction[0] = 'L'
##        Instruction [1] = 0
##
##    if chr(Instruction[0]) == 'L':
##        pass
##        return Instruction

##    if(XS1_val == 0 and YS1_val == 0):
##        Direction.color = color.red
##    else:
##        Direction.color = color.green

    #---------------------------------------------------------------------------------------------------
    #Chage the direction and position of the arrow.
    #---------------------------------------------------------------------------------------------------
    #Control the yaw.
    theta = radians(-1*XS1_val)
##    Direction.axis = rotate(Direction.axis, theta, (0, 1, 0))
##
##    #Control the position
##    Unitvec = norm(Direction.axis)# gives a unit vector in the direction that the arrow is pointing.
##    Unitvec = Unitvec*(-1)*YS1_val #Scale the vector by the value of the joystick
##    Direction.pos = Direction.pos + Unitvec
##    
    #---------------------------------------------------------------------------------------------------
    # Bound the Arrow within a 40 x 40 x 40 "box"
    #---------------------------------------------------------------------------------------------------
##    if(Direction.pos.x > Scale*25 or Direction.pos.x < -25*Scale):
##        if(Direction.pos.x > 0):
##            Direction.pos.x = Scale*25
##        else:
##            Direction.pos.x = -25*Scale
##
##    if(Direction.pos.y > Scale*25 or Direction.pos.y < -25*Scale):
##        if(Direction.pos.y > 0):
##            Direction.pos.y = Scale*25
##        else:
##            Direction.pos.y = -25*Scale
##
##    if(Direction.pos.z > Scale*25 or Direction.pos.z < -25*Scale):
##        if(Direction.pos.z > 0):
##            Direction.pos.z = Scale*25
##        else:
##            Direction.pos.z = -25*Scale

    #------------------------------------------------------------------------------------------------
    # Set and send motor values.
    #------------------------------------------------------------------------------------------------

    setMotorVal(XS1_val, YS1_val)
    setVertMotorVal(YS2_val)
    
    text = ""
        
    # send the values.

    Motorval[5] = 0
    if(Motorval[1] < 0):
        Motorval[5] += 8 #8
        Motorval[1] = abs(Motorval[1])
    if(Motorval[2] < 0):
        Motorval[5] += 4 #4
        Motorval[2] = abs(Motorval[2])
    if Motorval[3] < 0:
        Motorval[5] += 2
        Motorval[3] = abs(Motorval[3])
    if Motorval[4] < 0:
        Motorval[5] += 1
        Motorval[4] = abs(Motorval[4])
    
    Instruction[0] = 'M'
    redundantFlag = True
    
##    temp = Motorval[1]
##    Motorval[1] = Motorval[2]
##    Motorval[2] = temp

    for i in range(1, 6):
        #[opCode,left, right, front, back, DirectionFlags]
        Instruction[i] = int(round(Motorval[i], 0))
        text +=str(round(Motorval[i], 0)) + "  "
        if Instruction[i] != Last[i]:
            redundantFlag = False
            Last[i] = Instruction[i]

    #text+= " X:" + str(XS1_val) + " Y:" + str(YS1_val)
    
    #print(text)
    #Status.text = text
    #time.sleep(.1)
    #rate(100)

    #test
    #redundantFlag == False
    #test
    
    if redundantFlag == False:
        return Instruction
    else:
        return None
