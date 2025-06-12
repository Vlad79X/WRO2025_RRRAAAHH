# SPDX-License-Identifier: BSD-3-Clause
#
# Copyright (c) 2025 Nerdvana Centaurus
# All rights reserved.
#
# Use of this software in programs that control a robot, with or without modification, is permitted  
# provided that the following conditions are met:
#
# 1. Any program using this software must include the above copyright notice,  
#    this list of conditions, and the following disclaimer in its documentation or source code.
# 2. Any distributed software or firmware that incorporates this library must reproduce  
#    the above copyright notice, this list of conditions, and the following disclaimer  
#    in the documentation and/or other materials provided with the distribution.
# 3. Neither the name of Nerdvana Centaurus nor the names of its contributors may be used  
#    to endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,  
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ARE DISCLAIMED. IN NO EVENT  
# SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,  
# OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;  
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,  
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Enjoy coding with Pybricks and SPIKE!

from pupremote import PUPRemoteHub
from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Axis,Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.hubs import PrimeHub
from pybricks.tools import wait, StopWatch, multitask, run_task
from micropython import const
from ustruct import calcsize
from umath import ceil

#---Global_Values/Variables-----------------------------------------------------

#Grade claw si grade lift
deltaClaw = 0
deltaLift = 0

#Variabile pentru convertire de masuri
cm = const(10)
LaSuta = const(10)

#Specificatii robot
wheelDiameter = const(8.16) * cm
wheelDistance = const(14) * cm
wheelSensorDistance = const(11) * cm

#Valoarea maxima si minima de viteza
V0 = const(48)
Vmax = const(94)
V0rot = const(42)
V0comp = const(46)
V0arc = const(62)

#Acceleratie si deceleratie mers fata spate
AccelerationEncoder = const(250)
DecelerationEncoder = const(200)

#Acceleratie si deceleratie rotiri
AccelerationEncoderRot = const(10)
DecelerationEncoderRot = const(50)

#Acceleratie si deceleratie compas
AccelerationEncoderComp = const(10)
DecelerationEncoderComp = const(35)

#Acceleratie si deceleratie compas
AccelerationEncoderCompM = const(20)
DecelerationEncoderCompM = const(30)

AccelerationEncoderArc = const(30)
DecelerationEncoderArc = const(30)

#Valori culori
white = const(90)
black = const(25)

#K-uri pentru LineFollower
kpLF = const(0.19)
kdLF = const(4.5)
kiLF = const(0)

#K-uri pentru LineFollower cu Senzorii 3 si 5
kpLFSA = 0.5
kdLFSA = 1.65
kiLFSA = 0

#K-uri pentru Encoder RobotSpin
kpSP = const(0.5)
kdSP = const(1.2)
kiSP = const(0.001)

#K-uri pentru giroscop spin
kpANG = const(0.1)
kdANG = const(0.5)
kiANG = const(0.0001)

#K-uri pentru ARC
kpARC = const(1)
kdARC = const(1)
kiARC = const(0.0001)

#K-uri pentru Encoder RotateTo(Color)
kpSPB = const(0.8)
kdSPB = const(1)
kiSPB = const(0.001)

#K-uri pentru Encoder MoveSync
kpMS = const(1)
kdMS = const(0)
kiMS = const(0)

#K-uri pentru giroscop movesync
kpANGMS = const(3)
kdANGMS = const(0)
kiANGMS = const(0)

#Axis-uri pentru giroscop
TopAxis = Axis.Y
FrontAxis = -Axis.Z

#Directii
STANGA = const(-1)
DREAPTA = const(1)

#State uri pt motoare:

#Motor gheara
CLOSED = const(-200)
SEMIOPEN = const(-100)
OPEN = const(0)

#Motor lift
UP = const(50)
MID = const(25)
DOWN = const(0)

#maximum timer for rotation gyro correction
rotationTimer = const(379)

#---Declarations----------------------------------------------------------------

#Motoare
LeftMotor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
RightMotor = Motor(Port.C)

LiftMotor = Motor(Port.E)

ClawMotor = Motor(Port.D)


#Brick
Brick = InventorHub(TopAxis, FrontAxis)
InventorHub()
Navigation = DriveBase(LeftMotor, RightMotor, wheelDiameter, wheelDistance)

#Camera = ColorSensor(Port.C)
#Sensor = ColorSensor(Port.F)

#RotateTo(Color) PID Timer
timerPIDBlack = StopWatch()

pr = PUPRemoteHub(Port.B)
# Line Tracker commands
pr.add_command('lfall', to_hub_fmt='BBBBBBB', from_hub_fmt='')
pr.add_command('lfone', to_hub_fmt='B',        from_hub_fmt='B')
pr.add_command('lftwo', to_hub_fmt='BB',       from_hub_fmt='BB')
pr.add_command('lftre', to_hub_fmt='BBB',      from_hub_fmt='BBB')
pr.add_command('lfacl', from_hub_fmt='repr')
pr.add_command('lficl', from_hub_fmt='repr')

# HuskyLens: closest block to (x, y)
pr.add_command('camCl', to_hub_fmt='B', from_hub_fmt='2H')

#---Functions-------------------------------------------------------------------

def mm2deg(mm: int) -> int:
    return mm / (3.1415 * wheelDiameter) * 360
#end mm2deg

def deg2mm(degree: int) -> int:
    return degree * (360 / (3.1415 * wheelDiameter))
#end deg2mm

def Init():
    LiftMotor.reset_angle(0)
    run_task(liftGoTo(-3,1.2,0,300))
    ClawMotor.reset_angle(0)

#end init

def CheckSOL():
    s1,s2,s3,s4,s5,s6,s7 = pr.call("lfall",1,2,3,4,5,6,7)
    if(s3 <= 35 and s5 > 35): 
        return DREAPTA
    elif(s3 > 35 and s5 <= 35):
        return STANGA

    if(s2 <= 35 and s6 > 35): 
        return DREAPTA
    elif(s2 > 35 and s6 <= 35): 
        return STANGA

#end CheckSOL

async def liftGoTo(degrees: float, kp: float, kd: float, time: int):
    timerLift = StopWatch()
    exitCondition = 0
    errOldLift = 0

    if abs(LiftMotor.angle()) > abs(degrees):
        case = 1
    else:
        case = 2
    #endif
    
    while exitCondition == 0 and timerLift.time() < time:
        errLift = degrees - LiftMotor.angle()
        speedLift = (kp * errLift + kd * (errLift - errOldLift)) * 10
        errOldLift = errLift

        if case == 1:
            if abs(LiftMotor.angle()) < abs(degrees):
                exitCondition = 1
            #endif
        else:
            if abs(LiftMotor.angle()) > abs(degrees):
                exitCondition = 1
            #endif
        #endif

        LiftMotor.run(speedLift)

    #endwhile

    LiftMotor.hold()
#end liftGoTo

async def clawGoTo(degrees: int, kpClaw: float, kdClaw: float, time: int):
    timerClaw = StopWatch()

    if abs(ClawMotor.angle()) > abs(degrees):
        case = 1
    else:
        case = 2
    #endif

    exitCondition = 0
    errOldClaw = 0

    while exitCondition == 0 and timerClaw.time() < time:
        errClaw = degrees - ClawMotor.angle()
        speedClaw = (kpClaw * errClaw + kdClaw * (errClaw - errOldClaw)) * 10
        errOldClaw = errClaw

        if case == 1:
            if abs(ClawMotor.angle()) < abs(degrees):
                exitCondition = 1
            #endif
        else:
            if abs(ClawMotor.angle()) > abs(degrees):
                exitCondition = 1
            #endif
        #endif

        ClawMotor.run(speedClaw)

    #endwhile

    ClawMotor.hold()
#end clawGoTo

def DefaultDriveBase():
    Navigation.use_gyro(True)
    Navigation.settings(straight_speed = 90 * LaSuta,
                        straight_acceleration = 90 * cm,
                        turn_rate = 80 * LaSuta,
                        turn_acceleration = 80 * cm)
#end DefaultDriveBase

def CheckGyro():
    Navigation.reset()
    while(1):
        print(Brick.imu.rotation(TopAxis))
        wait(100)
    #endwhile
#end CheckGyro

def ArcMove(speedext: int, raza: int, grade: float, direction: int, accel: bool, decel: bool, brake: bool, turbo = 0):
    imux = Brick.imu.rotation(Axis.Y)
    speedsemn = speedext / abs(speedext)

    ratio = abs((raza - wheelDistance/2)/(raza + wheelDistance/2))

    speedint = speedext * ratio

    V0compL = V0compR = V0comp + turbo
    
    if(direction == STANGA):
        V0compL = V0arc * ratio
        V0compR = V0arc 
        leftSpeed = speedint
        rightSpeed = speedext
    else:
        V0compL = V0arc
        V0compR = V0arc * ratio
        leftSpeed = speedext
        rightSpeed = speedint


    FinalAngle = imux - grade * direction * speedsemn
    LastAngle = imux
    FirstAngle = imux

    error = 0
    error_old = 0
    error_sum = 0

    LeftMotor.reset_angle(0)
    RightMotor.reset_angle(0)

    VLeft = leftSpeed
    VRight = rightSpeed
    CurAngle = imux
    flag = 1

    if leftSpeed > 0:
        sens = 1
    else:
        sens = -1
    
    # print(leftSpeed,rightSpeed)

    while(flag == 1):
        LastAngle = CurAngle
        CurAngle = Brick.imu.rotation(Axis.Y)
        AngleEncoder = CurAngle - FirstAngle
        
        if accel == True:
            if abs(AngleEncoder) <= AccelerationEncoderArc:
                VLeft = (abs(AngleEncoder) / AccelerationEncoderArc) * (leftSpeed - V0compL) + V0compL
                if abs(VLeft) > abs(leftSpeed):
                   VLeft = leftSpeed
                #endif
                if abs(VLeft) < abs(V0compL) :
                    VLeft = V0compL * sens
                #endif

                VRight = (abs(AngleEncoder) / AccelerationEncoderArc) * (rightSpeed - V0compR) + V0compR
                if abs(VRight) > abs(rightSpeed):
                   VRight = rightSpeed
                #endif
                if abs(VRight) < abs(V0compR) :
                    VRight = V0compR * sens
                #endif
            #endif
        #endif

        if decel == True:
            if grade - abs(AngleEncoder) <= DecelerationEncoderArc :
                VLeft = (( (grade - abs(AngleEncoder) ) / DecelerationEncoderArc ) * ( abs(leftSpeed) - V0compL ) + V0compL)*sens
                if abs(VLeft) > abs(leftSpeed):
                   VLeft = leftSpeed
                #endif
                if abs(VLeft) < abs(V0compL) :
                    VLeft = V0compL * sens
                #endif

                VRight = (( (grade - abs(AngleEncoder) ) / DecelerationEncoderArc ) * ( abs(rightSpeed) - V0compR ) + V0compR)*sens
                if abs(VRight) > abs(rightSpeed):
                   VRight = rightSpeed
                #endif
                if abs(VRight) < abs(V0compR) :
                    VRight = V0compR * sens
                #endif
            #endif
        #endif

        if leftSpeed == speedint:
            error = LeftMotor.angle() - RightMotor.angle()*ratio
        else:
            error = LeftMotor.angle()*ratio - RightMotor.angle()

        P = kpARC * error
        D = kdARC * (error - error_old)
        I = kiARC * error_sum

        error_old = error
        error_sum = error_sum + error

        speedL = VLeft - (P + D + I)
        speedR = VRight + (P + D + I)

        
        LeftMotor.dc(speedL)
        RightMotor.dc(speedR)
        # print(CurAngle,FinalAngle,AngleEncoder,speedL,speedR)
        # print(abs(AngleEncoder) + (CurAngle-LastAngle)/2,CurAngle,LastAngle)
        
        # if direction == 1 and leftSpeed > 0 or direction == -1 and leftSpeed < 0:
        #     if(AngleEncoder <= grade):
        #         flag=0
        # else:
        if(abs(AngleEncoder) >= grade or LeftMotor.stalled() or RightMotor.stalled()):
            flag=0
    #endwhile
    timerPID = StopWatch()
    time = rotationTimer
    error = errorSum = errorOld = 0
    exitCondition = 0

    while exitCondition == 1:
        CurAngle = Brick.imu.rotation(TopAxis)
        error = FinalAngle - CurAngle

        vit = V0rot + 5

        if(error < 0):
            vit = -V0rot
        else:
            vit = V0rot
        #endif

        speedL = -vit
        speedR = vit

        if(direction == 1):
            RightMotor.dc(speedR)
            LeftMotor.dc(speedL)
        else:
            LeftMotor.dc(speedL)
            RightMotor.dc(speedR)
        #endif

        if timerPID.time() > time or (abs(error) < 0.07 and timerPID.time() > 150):
            exitCondition = 0
            LeftMotor.brake()
            RightMotor.brake()
        #endif

    #endwhile

    timerPID.pause()
    if brake == True:
        LeftMotor.brake()
        RightMotor.brake()
    #endif
#end ArcMoveSpeedAngle

def LF1SEncoder(speed: int, mm: float,sol, accel: bool, decel: bool, brake: bool, grey: int):
    #sol = Side Of Line (Stanga/Dreapta)

    FinalEncoder = mm2deg(mm)

    LeftMotor.reset_angle(0)
    RightMotor.reset_angle(0)

    V = ErrorOld = ErrorSum = Error = CurEncoder = 0

    while(CurEncoder <= FinalEncoder):
        s = Sensor.reflection()

        Error = (grey - s) * sol

        LeftEncoder = LeftMotor.angle()
        RightEncoder = RightMotor.angle()
        CurEncoder = (LeftEncoder + RightEncoder) / 2

        P = kpLF * Error
        I = kiLF * ErrorSum
        D = kdLF * (Error - ErrorOld)

        ErrorOld = Error
        ErrorSum = ErrorSum + Error

        if accel == True:
            if CurEncoder <= AccelerationEncoder:
                V = abs((CurEncoder / AccelerationEncoder) * (speed - V0) + V0)
                V = min(max(V, V0rot), speed)
            #endif
        else:
            V = speed
        #endif

        if decel == True:
            if FinalEncoder - CurEncoder <= DecelerationEncoder :
                V = abs(((FinalEncoder - CurEncoder) / DecelerationEncoder ) * ( speed - V0) + V0)
                V = min(max(V, V0rot), speed)
            #endif
        #endif
        # print(V + (P + I + D), V - (P + I + D),Error)
        
        LeftMotor.dc(V + (P + I + D))
        RightMotor.dc(V - (P + I + D))


    #endwhile
    
    if brake == True:
        LeftMotor.hold()
        RightMotor.hold()
    #endif
#end LF1SEncoder

def LFEncoder(speed: int, mm: float, accel: bool, decel: bool, brake: bool):
    FinalEncoder = mm2deg(mm)

    LeftMotor.reset_angle(0)
    RightMotor.reset_angle(0)

    V = ErrorOld = ErrorSum = Error = CurEncoder = 0
    

    if (speed < 0):
        sens = -1
    else:
        sens = 1
    #endif

    speed = abs(speed)

    while(CurEncoder <= FinalEncoder):
        s1 = LeftSensor.reflection()
        s2 = RightSensor.reflection()
        Error = s1 - s2

        LeftEncoder = LeftMotor.angle()
        RightEncoder = RightMotor.angle()
        CurEncoder = (LeftEncoder + RightEncoder) / 2

        P = kpLF * Error
        I = kiLF * ErrorSum
        D = kdLF * (Error - ErrorOld)

        ErrorOld = Error
        ErrorSum = ErrorSum + Error

        if accel == True:
            if CurEncoder <= AccelerationEncoder:
                V = abs((CurEncoder / AccelerationEncoder) * (speed - V0 * sens) + V0 * sens)
                V = min(max(V, V0rot), speed)
            #endif
        else:
            V = speed
        #endif

        if decel == True:
            if FinalEncoder - CurEncoder <= DecelerationEncoder :
                V = abs(((FinalEncoder - CurEncoder) / DecelerationEncoder ) * ( speed - V0 * sens ) + V0 * sens)
                V = min(max(V, V0rot), speed)
            #endif
        #endif

        V = V * sens

        LeftMotor.dc(V + (P + I + D))
        RightMotor.dc(V - (P + I + D))

    #endwhile
    
    if brake == True:
        LeftMotor.hold()
        RightMotor.hold()
    #endif

#end LFEncoder

def LFEncoderSA(speed: int, mm: float, kp: float, kd: float, sensor1: int, sensor2: int, color: str, accel: bool, decel: bool, brake: bool):
    FinalEncoder = mm2deg(mm)

    LeftMotor.reset_angle(0)
    RightMotor.reset_angle(0)

    V = ErrorOld = ErrorSum = Error = CurEncoder = 0
    

    if (speed < 0):
        sens = -1
    else:
        sens = 1
    #endif

    speed = abs(speed)
    pr.call('lfacl', color)

    while(CurEncoder <= FinalEncoder):
        s1, s2 = pr.call('lftwo', sensor1, sensor2)
        Error = (s1 - s2)
        # print(Error)

        LeftEncoder = LeftMotor.angle()
        RightEncoder = RightMotor.angle()
        CurEncoder = (LeftEncoder + RightEncoder) / 2

        P = kp * Error
        I = kiLFSA * ErrorSum
        D = kd * (Error - ErrorOld)

        ErrorOld = Error
        ErrorSum = ErrorSum + Error

        if accel == True:
            if CurEncoder <= AccelerationEncoder:
                V = abs((CurEncoder / AccelerationEncoder) * (speed - V0 * sens) + V0 * sens)
                V = min(max(V, V0rot), speed)
            #endif
        else:
            V = speed
        #endif

        if decel == True:
            if FinalEncoder - CurEncoder <= DecelerationEncoder :
                V = abs(((FinalEncoder - CurEncoder) / DecelerationEncoder ) * ( speed - V0 * sens ) + V0 * sens)
                V = min(max(V, V0rot), speed)
            #endif
        #endif

        V = V * sens

        LeftMotor.dc(V + (P + I + D))
        RightMotor.dc(V - (P + I + D))

    #endwhile
    
    if brake == True:
        LeftMotor.brake()
        RightMotor.brake()
    #endif
#end LFEncoderSA

def LFIntersectionSA(speed: int, sensor1: int, sensor2: int, blackV:int, intersections: int, color: str, accel: bool, aliniere: bool, brake: bool):

    LeftMotor.reset_angle(0)
    RightMotor.reset_angle(0)

    V = ErrorOld = ErrorSum = Error = CurEncoder = flag = 0
    
    if (speed < 0):
        sens = -1
    else:
        sens = 1
    #endif

    speed = abs(speed)
    pr.call('lfacl', color)

    while(flag < intersections):
        s1, s2 = pr.call('lftwo', sensor1, sensor2)
        Error = (s1 - s2)
        #print(s1,s2)

        LeftEncoder = LeftMotor.angle()
        RightEncoder = RightMotor.angle()
        CurEncoder = (LeftEncoder + RightEncoder) / 2

        P = kpLFSA * Error
        I = kiLFSA * ErrorSum
        D = kdLFSA * (Error - ErrorOld)

        ErrorOld = Error
        ErrorSum = ErrorSum + Error

        if accel == True:
            if CurEncoder <= AccelerationEncoder:
                V = abs((CurEncoder / AccelerationEncoder) * (speed - V0 * sens) + V0 * sens)
                V = min(max(V, V0rot), speed)
            #endif
        else:
            V = speed
        #endif

        V = V * sens

        LeftMotor.dc(V + (P + I + D))
        RightMotor.dc(V - (P + I + D))

        if(blackV <= 35):
            if(s1 <= blackV or s2 <= blackV):
                flag+=1
        else:
            if(s1 >= blackV or s2 >= blackV):
                flag +=1
        #endif

    #endwhile
    
    if(aliniere == True):
        Navigation.settings(straight_speed = speed * LaSuta)
        Navigation.settings(straight_acceleration = speed * LaSuta)
        Navigation.straight(wheelSensorDistance)
    #endif

    if brake == True:
        LeftMotor.brake()
        RightMotor.brake()
    #endif
#end LFIntersectionSA

def LF2SIntersectionBlack(speed: int, intersections: int, accel: bool, aliniere: bool, brake: bool):
    LeftMotor.reset_angle(0)
    RightMotor.reset_angle(0)

    sensitivityencoder = 250
    last = -sensitivityencoder

    V = CurIntersections = Error = ErrorOld = ErrorSum = 0
    sens = 1

    speed = abs(speed)

    while CurIntersections < intersections:
        s1 = LeftSensor.reflection()
        s2 = RightSensor.reflection()
        Error = s1 - s2

        LeftEncoder = LeftMotor.angle()
        RightEncoder = RightMotor.angle()
        CurEncoder = (LeftEncoder + RightEncoder) / 2
        
        if((s1 < black) or (s2 < black)) and (CurEncoder > last + sensitivityencoder):
            CurIntersections += 1
            last = CurEncoder
        #endIf

        P = kpLF * Error
        I = kiLF * ErrorSum
        D = kdLF * (Error - ErrorOld)

        ErrorOld = Error
        ErrorSum = ErrorSum + Error

        if accel == True:
            if CurEncoder <= AccelerationEncoder:
                V = abs((CurEncoder / AccelerationEncoder) * (speed - V0 * sens) + V0 * sens)
                V = min(max(V, V0rot), speed)
            #endif
        else:
            V = speed
        #endif

        V = V * sens

        LeftMotor.dc(V + (P + I + D))
        RightMotor.dc(V - (P + I + D))

    #endwhile

    if(aliniere == True):
        Navigation.settings(straight_speed = speed * LaSuta)
        Navigation.settings(straight_acceleration = speed * LaSuta)
        Navigation.straight(wheelSensorDistance)
    #endif
        
    if(brake == True):
        LeftMotor.brake()
        RightMotor.brake()
    #endif

#end LF2SIntersection

def MoveSync(speed: int, mm: float, accel: bool, decel: bool, brake: bool):
    FinalEncoder = mm2deg(mm)
    IniAngle = Navigation.angle()

    LeftMotor.reset_angle(0)
    RightMotor.reset_angle(0)
    
    V = ErrorEncoder = ErrorEncoderSum = ErrorEncoderOld = ErrorAngle = ErrorAngleSum = ErrorAngleOld = CurEncoder = 0

    if speed < 0:
        sens = -1
    else:
        sens = 1
    #endif

    speed = abs(speed)
    
    while(abs(CurEncoder) <= abs(FinalEncoder)):
        LeftEncoder = LeftMotor.angle()
        RightEncoder = RightMotor.angle()
        CurEncoder = abs((LeftEncoder + RightEncoder) / 2)
        ErrorEncoder = LeftEncoder - RightEncoder
        
        CurAngle = Navigation.angle()
        ErrorAngle = CurAngle - IniAngle

        Pe = kpMS * ErrorEncoder
        Ie = kiMS * ErrorEncoderSum
        De = kdMS * (ErrorEncoder - ErrorEncoderOld)

        ErrorEncoderOld = ErrorEncoder
        ErrorEncoderSum = ErrorEncoderSum + ErrorEncoder

        Pa = kpANG * ErrorAngle
        Ia = kiANG * ErrorAngleSum
        Da = kdANG * (ErrorAngle - ErrorAngleOld)
        
        ErrorAngleOld = ErrorAngle
        ErrorAngleSum = ErrorAngleSum + ErrorAngle

        if accel == True:
            if CurEncoder <= AccelerationEncoder:
                V = abs((CurEncoder / AccelerationEncoder) * (speed - V0 * sens) + V0 * sens)
                V = min(max(V, V0rot), speed)
            #endif
        else:
            V = speed
        #endif

        if decel == True:
            if FinalEncoder - CurEncoder <= DecelerationEncoder :
                V = abs(((FinalEncoder - CurEncoder) / DecelerationEncoder ) * ( speed - V0 * sens ) + V0 * sens)
                V = min(max(V, V0rot), speed)
            #endif
        #endif

        V = V * sens

        LeftMotor.dc(V - (Pe + Ie + De + Pa + Ia + Da))
        RightMotor.dc(V + (Pe + Ie + De + Pa + Ia + Da))

    #endwhile

    if brake == True:
        LeftMotor.brake()
        RightMotor.brake()
    #endif

#end MoveSync

def RobotSpin(speed, grade, direction, pid = 1, accel = 0, decel = 1, brake = 1, turbo = 0, turbopid = 0):
    """
    Execută o rotație a robotului.

    Parametri:
    - speed: Viteza de rotație.
    - grade: Unghiul în grade.
    - direction: STANGA sau DREAPTA.
    - pid: 1 = PID activat, 0 = dezactivat.
    - accel: 1 = accelerație activă, 0 = dezactivată.
    - decel: 1 = decelerație activă, 0 = dezactivată.
    - brake: 1 = frână activă, 0 = dezactivată.
    - turbo: turbo peste V0, 0 = dezactivat.
    """
    imux = Brick.imu.rotation(TopAxis)
    FirstAngle = CurAngle = imux
    FinalAngle = imux + grade * direction

    LeftMotor.reset_angle(0)
    RightMotor.reset_angle(0)

    V = ErrorEncoder = ErrorEncoderSum = ErrorEncoderOld = ErrorAngle = ErrorAngleSum = ErrorAngleOld = CurEncoder = 0
    flag = 1
    
    while(flag == 1 and LeftMotor.stalled() == False and RightMotor.stalled() == False):
        LeftEncoder = abs(LeftMotor.angle())
        RightEncoder = abs(RightMotor.angle())
        ErrorEncoder = LeftEncoder - RightEncoder

        CurAngle = Brick.imu.rotation(TopAxis)
        AngleEncoder = CurAngle - FirstAngle
        ErrorAngle = abs(FinalAngle - CurAngle)

        Pe = kpSP * ErrorEncoder
        Ie = kiSP * ErrorEncoderSum
        De = kdSP * (ErrorEncoder - ErrorEncoderOld)
        
        ErrorEncoderOld = ErrorEncoder
        ErrorEncoderSum = ErrorEncoderSum + ErrorEncoder

        if accel == True:
            if abs(AngleEncoder) <= AccelerationEncoderRot:
                V = abs((abs(AngleEncoder) / AccelerationEncoderRot) * (speed - V0rot) + V0rot)
                V = min(max(V, V0rot), speed)
            #endif
        else:
            V = speed
        #endif

        if decel == True:
            if (grade - abs(AngleEncoder)) <= DecelerationEncoderRot :
                V = abs((grade - abs(AngleEncoder)) / DecelerationEncoderRot  * (speed - V0rot) + V0rot)
                V = min(max(V, V0rot), speed)
            #endif
        #endif

        speedL = ((V - (Pe + Ie + De)) * direction)
        speedR = -((V + (Pe + Ie + De)) * direction) 

        if(direction == 1):
            RightMotor.dc(speedR)
            LeftMotor.dc(speedL)
        else:
            LeftMotor.dc(speedL)
            RightMotor.dc(speedR)
        #endif

        if(abs(AngleEncoder) >= grade):
            flag=0
        #endif

    #endwhile

    timerPID = StopWatch()
    time = rotationTimer
    error = errorSum = errorOld = 0
    exitCondition = pid

    vit = V0rot

    while exitCondition == 1:
        CurAngle = Brick.imu.rotation(TopAxis)
        error = FinalAngle - CurAngle

        if(error < 0):
            vit = V0rot + turbopid
        else:
            vit = -V0rot - turbopid
        #endif

        speedL = -vit
        speedR = vit

        if(direction == 1):
            RightMotor.dc(speedR)
            LeftMotor.dc(speedL)
        else:
            LeftMotor.dc(speedL)
            RightMotor.dc(speedR)
        #endif

        if timerPID.time() > time or (abs(error) < 0.05 and timerPID.time() > 100):
            LeftMotor.brake()
            RightMotor.brake()
            exitCondition = 0
        #endif

    #endwhile

    timerPID.pause()

    if brake == True:
        LeftMotor.brake()
        RightMotor.brake()
    #endif
    
#end RobotSpin

def goToHeading(speed: int, grade:float, accel: bool, decel: bool, brake: bool):
    
    imux = Brick.imu.rotation(TopAxis)
    FirstAngle = CurAngle = imux
    FinalAngle = imux + grade * direction

    LeftMotor.reset_angle(0)
    RightMotor.reset_angle(0)

    V = ErrorEncoder = ErrorEncoderSum = ErrorEncoderOld = ErrorAngle = ErrorAngleSum = ErrorAngleOld = CurEncoder = 0
    flag = 1
    
    while(flag == 1 and LeftMotor.stalled() == False and RightMotor.stalled() == False):
        LeftEncoder = abs(LeftMotor.angle())
        RightEncoder = abs(RightMotor.angle())
        ErrorEncoder = LeftEncoder - RightEncoder

        CurAngle = Brick.imu.rotation(TopAxis)
        AngleEncoder = CurAngle - FirstAngle
        ErrorAngle = abs(FinalAngle - CurAngle)

        Pe = kpSP * ErrorEncoder
        Ie = kiSP * ErrorEncoderSum
        De = kdSP * (ErrorEncoder - ErrorEncoderOld)
        
        ErrorEncoderOld = ErrorEncoder
        ErrorEncoderSum = ErrorEncoderSum + ErrorEncoder

        if accel == True:
            if abs(AngleEncoder) <= AccelerationEncoderRot:
                V = abs((abs(AngleEncoder) / AccelerationEncoderRot) * (speed - V0rot) + V0rot)
                V = min(max(V, V0rot), speed)
            #endif
        else:
            V = speed
        #endif

        if decel == True:
            if (grade - abs(AngleEncoder)) <= DecelerationEncoderRot :
                V = abs((grade - abs(AngleEncoder)) / DecelerationEncoderRot  * (speed - V0rot) + V0rot)
                V = min(max(V, V0rot), speed)
            #endif
        #endif

        speedL = ((V - (Pe + Ie + De)) * direction)
        speedR = -((V + (Pe + Ie + De)) * direction) 

        if(direction == 1):
            RightMotor.dc(speedR)
            LeftMotor.dc(speedL)
        else:
            LeftMotor.dc(speedL)
            RightMotor.dc(speedR)
        #endif

        if(abs(AngleEncoder) >= grade):
            flag=0
        #endif

    #endwhile

    timerPID = StopWatch()
    time = rotationTimer
    # print("PID ACUM")
    error = errorSum = errorOld = 0
    exitCondition = 1

    while exitCondition == 1 and pid == True:
        CurAngle = Brick.imu.rotation(TopAxis)
        error = FinalAngle - CurAngle

        vit = V0rot + 5

        if(error < 0):
            vit = V0rot
        else:
            vit = -V0rot
        #endif

        speedL = -vit
        speedR = vit

        if(direction == 1):
            RightMotor.dc(speedR)
            LeftMotor.dc(speedL)
        else:
            LeftMotor.dc(speedL)
            RightMotor.dc(speedR)
        #endif

        if timerPID.time() > time or (abs(error) < 0.07 and timerPID.time() > 79):
            exitCondition = 0
        #endif

    #endwhile

    timerPID.pause()

    if brake == True:
        LeftMotor.brake()
        RightMotor.brake()
    #endif
    
#end RobotSpin


def RobotCompas(speed: int, grade: float, direction: int, accel: bool, decel: bool, brake: bool, turbo: int):
    encoder = ((wheelDistance * 3.14) * grade ) / 360
    FinalEncoder = mm2deg(encoder)


    LeftMotor.reset_angle(0)
    RightMotor.reset_angle(0)

    speedsemn = speed / abs(speed)

    imux = Brick.imu.rotation(TopAxis)
    FinalAngle = imux + grade * direction * speedsemn
    FirstAngle = CurAngle = imux

    V = CurEncoder = Enc = 0
    
    if speed > 0:
        sens = 1
    else:
        sens = -1
    #endif

    speed = abs(speed)

    flag = 1
    
    while(flag == 1):
        LeftEncoder = abs(LeftMotor.angle())
        RightEncoder = abs(RightMotor.angle())
        CurEncoder = (LeftEncoder + RightEncoder) / 2

        CurAngle = Brick.imu.rotation(TopAxis)
        AngleEncoder = CurAngle - FirstAngle
        
        if direction == -1:
            Enc = RightEncoder
        else:
            Enc = LeftEncoder
        #endIf

        if accel == True:
            if abs(AngleEncoder) <= AccelerationEncoderComp:
                V = abs((abs(AngleEncoder) / AccelerationEncoderComp) * (speed - V0comp - turbo) + V0comp+ turbo)
                V = min(max(V, V0comp), speed)
            #endif
        else:
            V = speed
        #endif

        if decel == True:
            if grade - abs(AngleEncoder) <= DecelerationEncoderComp :
                V = abs(( (grade - abs(AngleEncoder) ) / DecelerationEncoderComp ) * ( abs(speed) - V0comp - turbo ) + V0comp + turbo)
                V = min(max(V, V0comp), speed)
            #endif
        #endif
        
        V = V * sens
        #print(CurAngle,FinalAngle)

        if direction == 1:
            RightMotor.hold()
            LeftMotor.dc(V)
        else:
            LeftMotor.hold()
            RightMotor.dc(V)
        #endif
        
        if (direction == 1 and speed*sens > 0) or (direction == -1 and speed*sens < 0):
            if(CurAngle >= FinalAngle):
                flag=0
        else:
            if(CurAngle <= FinalAngle):
                flag=0
        #endif

    #endwhile
    
    if brake == True:
        LeftMotor.brake()
        RightMotor.brake()
    #endif

def CompasTime(speed: int, time: float, direction:int, brake: bool):
    speedsemn = speed / abs(speed)
    timercompas = StopWatch()
    V = 0
    
    if speed > 0:
        sens = 1
    else:
        sens = -1
    #endif

    speed = abs(speed)

    flag = 1
    
    while(flag == 1):
        V = speed * sens

        if direction == 1:
            RightMotor.hold()
            LeftMotor.dc(V)
        else:
            LeftMotor.hold()
            RightMotor.dc(V)
        #endif
        
        if (timercompas.time() >= time*1000):
            flag = 0
        #endif

    #endwhile
    
    if brake == True:
        LeftMotor.brake()
        RightMotor.brake()
    #endif


#end RobotCompas


def CompasCareMerge(speed: int, grade: float, direction: int, accel: bool, decel: bool, brake: bool, turbo: int):
    encoder = ((wheelDistance * 3.14) * grade ) / 360
    FinalEncoder = mm2deg(encoder)

    LeftMotor.reset_angle(0)
    RightMotor.reset_angle(0)

    speedsemn = speed / abs(speed)

    imux = Brick.imu.rotation(TopAxis)
    FinalAngle = imux + grade * direction * speedsemn
    FirstAngle = CurAngle = imux

    V = CurEncoder = Enc = 0
    
    if speed > 0:
        sens = 1
    else:
        sens = -1
    #

    speed = abs(speed)

    flag = 1

    CurAngle = AngleEncoder = Brick.imu.rotation(TopAxis)
    delta = 0
    
    while(flag == 1):
        delta = abs(CurAngle - Brick.imu.rotation(TopAxis)) 
        CurAngle = Brick.imu.rotation(TopAxis)
        AngleEncoder = CurAngle - FirstAngle

        if accel == True:
            if abs(AngleEncoder) <= AccelerationEncoderCompM:
                V = abs((abs(AngleEncoder) / AccelerationEncoderCompM) * (speed - V0comp - turbo) + V0comp+ turbo)
                V = min(max(V, V0comp), speed)
            #
        else:
            V = speed
        #

        if decel == True:
            if grade - abs(AngleEncoder) <= DecelerationEncoderCompM :
                V = abs(( (grade - abs(AngleEncoder) ) / DecelerationEncoderCompM ) * ( abs(speed) - V0comp - turbo ) + V0comp + turbo)
                V = min(max(V, V0comp), speed)
            #
        #
        
        V = V * sens
        #print([CurAngle,FinalAngle,AngleEncoder,V,delta])

        if direction == 1:
            RightMotor.hold()
            LeftMotor.dc(V)
        else:
            LeftMotor.hold()
            RightMotor.dc(V)
        #
        
        if (direction == 1 and speed*sens > 0) or (direction == -1 and speed*sens < 0):
            if(CurAngle + delta >= FinalAngle):
                flag=0
        else:
            if(CurAngle - delta <= FinalAngle):
                flag=0
        #
    #
    
    if brake == True:
        LeftMotor.hold()
        RightMotor.hold()
    #
#

def SMove(speed: int, grade: float, direction: int, accel: bool, decel: bool, brake: bool, turbo: int):
    RobotCompas(speed, grade, direction, accel, decel, brake, turbo)
    wait(100)
    RobotCompas(speed, grade, 0 - direction, accel, decel, brake, turbo)
    wait(100)

# ┌ 𝙙𝙚 𝙛𝙪𝙣𝙘𝙩𝙞𝙖 𝙖𝙨𝙩𝙖 𝙣𝙪 𝙢𝙖 𝙖𝙩𝙞𝙣𝙜 ┐ -𝓿𝓵𝓪𝓭
def RobotSpinWhite(speed: int, direction: int, endBrake: bool):
    exitCondition = 1
    if direction == 1:
        sens = 1
    else:
        sens = 2
    #endif

    while exitCondition == 1:
        
        s1 = LeftSensor.reflection()
        s2 = RightSensor.reflection()

        if sens == 1:
            
            RightMotor.dc(speed * -1)
            LeftMotor.dc(speed)

            if s1 > white:
                exitCondition = 0
            #endif
        else:

            RightMotor.dc(speed)
            LeftMotor.dc(speed*-1)

            if s2 > white:
                exitCondition = 0
            #endif
        #endif
    #endwhile
    if endBrake == True:
        LeftMotor.brake()
        RightMotor.brake()

        LeftMotor.hold()
        RightMotor.hold()
        Navigation.straight(0,Stop.HOLD)
#end RobotSpinWhite

def MoveTime(speed: int, time: float, accel: bool, brake: bool):
    LeftMotor.reset_angle(0)
    RightMotor.reset_angle(0)
    
    timeWatch = StopWatch()
    ms = time * 1000
    
    sens = 1
    if speed < 0:
        sens = -1
    #endif

    speed = abs(speed)

    V =  speed

    while(timeWatch.time() < ms):
        LeftEncoder = LeftMotor.angle()
        RightEncoder = RightMotor.angle()
        CurEncoder = (LeftEncoder + RightEncoder) / 2
        
        if accel == True:
            if CurEncoder <= AccelerationEncoder:
                V = abs((CurEncoder / AccelerationEncoder) * (speed - V0 * sens) + V0 * sens)
                V = min(max(V, V0rot), speed)
            #endif
        else:
            V = speed
        #endif

        V = V * sens

        LeftMotor.dc(V)
        RightMotor.dc(V)

    #endWhile

    if(brake == True):
        LeftMotor.brake()
        RightMotor.brake()
    #endif

#end MoveTime

def RobotSpinBlack(speed: int, direction: int, grey: int, endBrake: bool, turbo = 0):

    exitCondition = 1

    if direction == 1:
        sens = 1
    else:
        sens = 2
    #endif

    pr.call("lfacl","red")
    while exitCondition == 1:

        s1,s2 = pr.call("lftwo",3,5)

        if sens == 1:

            RightMotor.dc(speed * -1)
            LeftMotor.dc(speed)

            if s2 < grey:
                exitCondition = 0
            #endif

        else:

            RightMotor.dc(speed)
            LeftMotor.dc(speed * -1)

            if s1 < grey:
                exitCondition = 0
            #endif

        #endif

    #endwhile
    timerPID = StopWatch()
    time = 1000
    error = 0
    errorSum = 0
    errorOld = 0

    exitCondition = 1

    while exitCondition == 1:
        #print(timerPID.time())
        s1,s2 = pr.call("lftwo",3,5)

        error = s1 - s2
        errorSum = errorSum + error
        errorOld = error
        Pb = kpSPB * error
        Db = kdSPB * (error - errorOld)
        Ib = kiSPB * errorSum

        LeftMotor.dc(-(V0rot+turbo+(Pb + Db + Ib)))
        RightMotor.dc(V0rot+turbo+Pb + Db + Ib)

        if timerPIDBlack.time() > time:
            exitCondition = 0
        #endif

    #endwhile

    if endBrake == True:
        LeftMotor.brake()
        RightMotor.brake()
    #endif
#end RobotSpinBlack

def SquaringWhite(speed: int, white: int, accel: bool, aliniere: bool, brake: bool):
    IniAngle = Brick.imu.rotation(TopAxis)
    
    V = ErrorEncoder = ErrorEncoderSum = ErrorEncoderOld = ErrorAngle = ErrorAngleSum = ErrorAngleOld = CurEncoder = 0

    if speed < 0:
        sens = -1
    else:
        sens = 1
    #endif

    speed = abs(speed)

    while(Sensor.reflection() < white): #and RightSensor.reflection() < white):
        LeftEncoder = LeftMotor.angle()
        RightEncoder = RightMotor.angle()
        CurEncoder = (LeftEncoder + RightEncoder) / 2
        ErrorEncoder = LeftEncoder - RightEncoder

        CurAngle = Brick.imu.rotation(TopAxis)
        ErrorAngle = -(CurAngle - IniAngle)*sens

        Pe = 0
        Ie = 0     # 𝙗𝙪𝙩 𝙬𝙝𝙮
        De = 0

        ErrorEncoderOld = 0
        ErrorEncoderSum = 0

        Pa = (kpANG+10) * ErrorAngle
        Ia = kiANG * ErrorAngleSum
        Da = kdANG * (ErrorAngle - ErrorAngleOld)
        
        ErrorAngleOld = ErrorAngle
        ErrorAngleSum = ErrorAngleSum + ErrorAngle

        if accel == True:
            if CurEncoder <= AccelerationEncoder:
                V = abs((CurEncoder / AccelerationEncoder) * (speed - V0 ) + V0 )
                V = min(max(V, V0rot), speed)
            #endif
        else:
            V = speed
        #endif

        LeftMotor.dc((V + (Pe + Ie + De + Pa + Ia + Da))* sens)
        RightMotor.dc((V - (Pe + Ie + De + Pa + Ia + Da))* sens)

    #endwhile

    if(aliniere == True):
        Navigation.settings(straight_speed = speed * LaSuta)
        Navigation.settings(straight_acceleration = speed * LaSuta)
        Navigation.straight(wheelSensorDistance + 2*cm)
    #endIf

    if brake == True:
        LeftMotor.brake()
        RightMotor.brake()
    #endif

#end SquaringWhite

def SquaringWhiteSA(speed: int, whiteV: int, sensor: int, color: str, accel: bool, aliniere: bool, brake: bool):
    IniAngle = Brick.imu.rotation(TopAxis)
    
    V = ErrorEncoder = ErrorEncoderSum = ErrorEncoderOld = ErrorAngle = ErrorAngleSum = ErrorAngleOld = CurEncoder = 0

    if speed < 0:
        sens = -1
    else:
        sens = 1
    #endif

    speed = abs(speed)
    pr.call('lfacl', color)

    s = pr.call('lfone', sensor)
    #print(s)
    while(s < whiteV):
        s = pr.call('lfone', sensor)
        #print(s)
        LeftEncoder = LeftMotor.angle()
        RightEncoder = RightMotor.angle()
        CurEncoder = (LeftEncoder + RightEncoder) / 2
        ErrorEncoder = LeftEncoder - RightEncoder

        CurAngle = Brick.imu.rotation(TopAxis)
        ErrorAngle = -(CurAngle - IniAngle)*sens

        Pe = 0
        Ie = 0     # 𝙗𝙪𝙩 𝙬𝙝𝙮
        De = 0

        ErrorEncoderOld = 0
        ErrorEncoderSum = 0

        Pa = (kpANG+10) * ErrorAngle
        Ia = kiANG * ErrorAngleSum
        Da = kdANG * (ErrorAngle - ErrorAngleOld)
        
        ErrorAngleOld = ErrorAngle
        ErrorAngleSum = ErrorAngleSum + ErrorAngle

        if accel == True:
            if CurEncoder <= AccelerationEncoder:
                V = abs((CurEncoder / AccelerationEncoder) * (speed - V0 ) + V0 )
                V = min(max(V, V0rot), speed)
            #endif
        else:
            V = speed
        #endif

        LeftMotor.dc((V + (Pe + Ie + De + Pa + Ia + Da))* sens)
        RightMotor.dc((V - (Pe + Ie + De + Pa + Ia + Da))* sens)

    #endwhile

    if(aliniere == True):
        Navigation.settings(straight_speed = speed * LaSuta)
        Navigation.settings(straight_acceleration = speed * LaSuta)
        Navigation.straight(wheelSensorDistance + 2*cm)
    #endIf

    if brake == True:
        LeftMotor.brake()
        RightMotor.brake()
    #endif

#end SquaringWhiteSA


def SquaringBlack(speed: int, black: int, maxcm:float, accel: bool, aliniere: bool, brake: bool):
    IniAngle = Brick.imu.rotation(TopAxis)
    FinalEncoder = mm2deg(maxcm)
    
    V = ErrorEncoder = ErrorEncoderSum = ErrorEncoderOld = ErrorAngle = ErrorAngleSum = ErrorAngleOld = CurEncoder = 0

    if speed < 0:
        sens = -1
    else:
        sens = 1
    #endif

    speed = abs(speed)

    while(Sensor.reflection() > black and abs(CurEncoder) < abs(FinalEncoder) ): #and RightSensor.reflection() > black):
        LeftEncoder = LeftMotor.angle()
        RightEncoder = RightMotor.angle()
        CurEncoder = (LeftEncoder + RightEncoder) / 2
        ErrorEncoder = LeftEncoder - RightEncoder

        CurAngle = Brick.imu.rotation(TopAxis)
        ErrorAngle = -(CurAngle - IniAngle) * sens

        Pe = 0
        Ie = 0     # 𝙗𝙪𝙩 𝙬𝙝𝙮
        De = 0

        ErrorEncoderOld = 0
        ErrorEncoderSum = 0

        Pa = (kpANG+10) * ErrorAngle
        Ia = kiANG * ErrorAngleSum
        Da = kdANG * (ErrorAngle - ErrorAngleOld)
        
        ErrorAngleOld = ErrorAngle
        ErrorAngleSum = ErrorAngleSum + ErrorAngle

        if accel == True:
            if CurEncoder <= AccelerationEncoder:
                V = abs((CurEncoder / AccelerationEncoder) * (speed - V0 ) + V0 )
                V = min(max(V, V0rot), speed)
            #endif
        else:
            V = speed
        #endif

        LeftMotor.dc((V + (Pe + Ie + De + Pa + Ia + Da))*sens)
        RightMotor.dc((V - (Pe + Ie + De + Pa + Ia + Da))*sens)

    #endwhile

    if(aliniere == True):
        Navigation.settings(straight_speed = speed * LaSuta)
        Navigation.settings(straight_acceleration = speed * LaSuta)
        Navigation.straight(wheelSensorDistance + 2*cm)
    #endIf

    if brake == True:
        LeftMotor.brake()
        RightMotor.brake()
    #endif

#end SquaringBlack

def SquaringBlackSA(speed: int, blackV: int, sensor:int, color: str, maxcm:float, accel: bool, aliniere: bool, brake: bool):
    IniAngle = Brick.imu.rotation(TopAxis)
    FinalEncoder = mm2deg(maxcm)
    
    V = ErrorEncoder = ErrorEncoderSum = ErrorEncoderOld = ErrorAngle = ErrorAngleSum = ErrorAngleOld = CurEncoder = 0

    if speed < 0:
        sens = -1
    else:
        sens = 1
    #endif

    speed = abs(speed)
    pr.call('lfacl', color)

    s = pr.call('lfone', sensor)
    while(s > blackV and abs(CurEncoder) < abs(FinalEncoder)):
        s = pr.call('lfone', sensor)
        LeftEncoder = LeftMotor.angle()
        RightEncoder = RightMotor.angle()
        CurEncoder = (LeftEncoder + RightEncoder) / 2
        ErrorEncoder = LeftEncoder - RightEncoder

        CurAngle = Brick.imu.rotation(TopAxis)
        ErrorAngle = -(CurAngle - IniAngle) * sens

        Pe = 0
        Ie = 0     # 𝙗𝙪𝙩 𝙬𝙝𝙮
        De = 0

        ErrorEncoderOld = 0
        ErrorEncoderSum = 0

        Pa = (kpANG+10) * ErrorAngle
        Ia = kiANG * ErrorAngleSum
        Da = kdANG * (ErrorAngle - ErrorAngleOld)
        
        ErrorAngleOld = ErrorAngle
        ErrorAngleSum = ErrorAngleSum + ErrorAngle

        if accel == True:
            if CurEncoder <= AccelerationEncoder:
                V = abs((CurEncoder / AccelerationEncoder) * (speed - V0 ) + V0 )
                V = min(max(V, V0rot), speed)
            #endif
        else:
            V = speed
        #endif

        LeftMotor.dc((V + (Pe + Ie + De + Pa + Ia + Da))*sens)
        RightMotor.dc((V - (Pe + Ie + De + Pa + Ia + Da))*sens)

    #endwhile

    if(aliniere == True):
        Navigation.settings(straight_speed = speed * LaSuta)
        Navigation.settings(straight_acceleration = speed * LaSuta)
        Navigation.straight(wheelSensorDistance)
    #endIf

    if brake == True:
        LeftMotor.brake()
        RightMotor.brake()
    #endif

#end SquaringBlackS4

def MoveSyncGyro(speed: int, mm: float, accel: bool, decel: bool, brake: bool):
    FinalEncoder = mm2deg(mm)

    LeftMotor.reset_angle(0)
    RightMotor.reset_angle(0)

    IniAngle = Brick.imu.rotation(TopAxis)

    V = ErrorEncoder = ErrorEncoderSum = ErrorEncoderOld = ErrorAngle = ErrorAngleSum = ErrorAngleOld = CurEncoder = 0

    if speed < 0:
        sens = -1
    else:
        sens = 1
    #endif

    speed = abs(speed)
        
    while(abs(CurEncoder) <= abs(FinalEncoder)):
        LeftEncoder = LeftMotor.angle()
        RightEncoder = RightMotor.angle()
        CurEncoder = abs((LeftEncoder + RightEncoder) / 2)
        ErrorEncoder = LeftEncoder - RightEncoder

        CurAngle = Brick.imu.rotation(TopAxis)
        ErrorAngle = -(CurAngle - IniAngle)*sens

        Pe = 0
        Ie = 0     # 𝙗𝙪𝙩 𝙬𝙝𝙮
        De = 0
        
        ErrorEncoderOld = 0
        ErrorEncoderSum = 0

        Pa = kpANGMS * ErrorAngle
        Ia = kiANGMS * ErrorAngleSum
        Da = kdANGMS * (ErrorAngle - ErrorAngleOld)

        ErrorAngleOld = ErrorAngle
        ErrorAngleSum = ErrorAngleSum + ErrorAngle

        if accel == True:
            if CurEncoder <= AccelerationEncoder:
                V = abs((CurEncoder / AccelerationEncoder) * (speed - V0 ) + V0)
                V = min(max(V, V0rot), speed)
            #endif
        else:
            V = speed
        #endif

        if decel == True:
            if FinalEncoder - CurEncoder <= DecelerationEncoder :
                V = abs(((FinalEncoder - CurEncoder) / DecelerationEncoder ) * (speed - V0) + V0)
                V = min(max(V, V0rot), speed)
            #endif
        #endif

        LeftMotor.dc((V + (Pe + Ie + De + Pa + Ia + Da)) * sens)
        RightMotor.dc((V - (Pe + Ie + De + Pa + Ia + Da)) * sens)

    #endwhile

    if brake == True:
        LeftMotor.brake()
        RightMotor.brake()
    #endif

def MSGandCLOSE(speed: int, mm: float, accel: bool, decel: bool, brake: bool, clawspeed = 0, liftspeed = 0):
    FinalEncoder = mm2deg(mm)

    LeftMotor.reset_angle(0)
    RightMotor.reset_angle(0)

    IniAngle = Brick.imu.rotation(TopAxis)

    V = ErrorEncoder = ErrorEncoderSum = ErrorEncoderOld = ErrorAngle = ErrorAngleSum = ErrorAngleOld = CurEncoder = 0
    clawCondition = liftCondition = 1
    if speed < 0:
        sens = -1
    else:
        sens = 1
    #endif

    speed = abs(speed)
        
    while(abs(CurEncoder) <= abs(FinalEncoder)):
        LeftEncoder = LeftMotor.angle()
        RightEncoder = RightMotor.angle()
        CurEncoder = abs((LeftEncoder + RightEncoder) / 2)
        ErrorEncoder = LeftEncoder - RightEncoder

        CurAngle = Brick.imu.rotation(TopAxis)
        ErrorAngle = -(CurAngle - IniAngle)*sens

        Pe = 0
        Ie = 0     # 𝙗𝙪𝙩 𝙬𝙝𝙮
        De = 0
        
        ErrorEncoderOld = 0
        ErrorEncoderSum = 0

        Pa = kpANGMS * ErrorAngle
        Ia = kiANGMS * ErrorAngleSum
        Da = kdANGMS * (ErrorAngle - ErrorAngleOld)

        ErrorAngleOld = ErrorAngle
        ErrorAngleSum = ErrorAngleSum + ErrorAngle

        if accel == True:
            if CurEncoder <= AccelerationEncoder:
                V = abs((CurEncoder / AccelerationEncoder) * (speed - V0 ) + V0)
                V = min(max(V, V0rot), speed)
            #endif
        else:
            V = speed
        #endif

        if decel == True:
            if FinalEncoder - CurEncoder <= DecelerationEncoder :
                V = abs(((FinalEncoder - CurEncoder) / DecelerationEncoder ) * (speed - V0) + V0)
                V = min(max(V, V0rot), speed)
            #endif
        #endif

        LeftMotor.dc((V + (Pe + Ie + De + Pa + Ia + Da)) * sens)
        RightMotor.dc((V - (Pe + Ie + De + Pa + Ia + Da)) * sens)
        if liftCondition:
            LiftMotor.run(liftspeed*10)
        if LiftMotor.stalled():
            liftCondition = 0

        if clawCondition:
            ClawMotor.run(-clawspeed*10)
        if ClawMotor.stalled():
            clawCondition = 0

    #endwhile
    LiftMotor.hold()
    ClawMotor.hold()

    if brake == True:
        LeftMotor.brake()
        RightMotor.brake()
    #endif

def GetSwitchesAndCase(cub1:int,cub2:int,cub3:int,cub4:int):
    switchleftright1 = False
    switchleftright2 = False
    switchinsideoutside = False
    caz = "drept"
    if ((cub1 == 4 and cub2 == 2) or (cub1 == 2 and cub2 == 4)) or ((cub1 == 1 and cub2 == 3) or (cub1 == 3 and cub2 == 1)):
        caz = "lateral"
        if (cub1 == 2 and cub2 == 4) or (cub1 == 1 and cub2 == 3):
            switchleftright1 = True
        if (cub3 == 4 and cub4 == 2) or (cub3 == 3 and cub4 == 1):
            switchleftright2 = True
        if (cub1 == 4 and cub2 == 2) or (cub1 == 2 and cub2 == 4):
            switchinsideoutside = True
    if ((cub1 == 3 and cub2 == 2) or (cub1 == 2 and cub2 == 3)) or ((cub1 == 4 and cub2 == 1) or (cub1 == 1 and cub2 == 4)):
        caz = "diagonal"
        if (cub1 == 2 and cub2 == 3) or (cub1 == 1 and cub2 == 4):
            switchleftright1 = True
        if (cub3 == 4 and cub4 == 1) or (cub3 == 3 and cub4 == 2):
            switchleftright2 = True
        if (cub3 == 4 and cub4 == 1) or (cub3 == 1 and cub4 == 4):
            switchinsideoutside = True
    if caz == "drept":
        if (cub1 == 2 and cub2 == 1) or (cub1 == 4 and cub2 == 3):
            switchleftright1 = True
        if (cub3 == 1 and cub4 == 2) or (cub3 == 3 and cub4 == 4):
            switchleftright2 = True
        if (cub1 == 3 and cub2 == 4) or (cub1 == 4 and cub2 == 3):
            switchinsideoutside = True
    
    return switchleftright1,switchleftright2,switchinsideoutside,caz                   
#end MoveSyncGyro

def SwitchAll():
    imux = Brick.imu.rotation(TopAxis)
    run_task(clawGoTo(-80,1,0,400))
    MoveSyncGyro(-90,10*cm,1,1,1)
    wait(50)
    RobotCompas(90,15,DREAPTA,0,1,1,10)
    wait(50)
    MoveSyncGyro(90,10*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1.2,0,400))
    wait(50)
    MoveSyncGyro(-90,13*cm,1,1,1)
    wait(50)
    RobotCompas(90,30,STANGA,0,1,1,10)
    wait(50)
    run_task(clawGoTo(-82,1,0,400))
    MoveSyncGyro(90,11*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1.2,0,400))
    wait(50)
    run_task(liftGoTo(UP+10,3,1,400))
    wait(50)
    MoveSyncGyro(-90,22*cm,1,1,1)
    run_task(multitask(
        liftGoTo(DOWN,1.5,0,400),
        clawGoTo(OPEN,1,0,400),
        liftGoTo(UP,2,2,400)
    ))
    wait(50)
    MoveSyncGyro(90,8*cm,1,1,1)
    run_task(liftGoTo(DOWN,1.5,0,400))
    run_task(clawGoTo(-80,1,0,400))
    MoveSyncGyro(-90,9*cm,1,1,1)
    wait(50)
    RobotCompas(90,15,DREAPTA,0,1,1,10)
    wait(50)
    MoveSyncGyro(90,10*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1.2,0,400))
    wait(50)
    MoveSyncGyro(-90,13*cm,1,1,1)
    wait(50)
    RobotCompas(90,30,STANGA,0,1,1,10)
    wait(50)
    run_task(clawGoTo(-82,1,0,400))
    MoveSyncGyro(90,11*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1.2,0,400))
    wait(50)
    RobotCompas(-90,50,STANGA,0,1,1,10)
    RobotSpin(80,abs(imux - Brick.imu.rotation(TopAxis)),(imux - Brick.imu.rotation(TopAxis))/abs(imux - Brick.imu.rotation(TopAxis)),1,0,0,1)
    wait(50)
    MSGandCLOSE(90,28*cm,1,1,1,10)
#end SwitchAll

def SwitchLRthenIO():
    imux = Brick.imu.rotation(TopAxis)
    run_task(clawGoTo(-80,1,0,400))
    MoveSyncGyro(-90,10*cm,1,1,1)
    wait(50)
    RobotCompas(90,15,DREAPTA,0,1,1,10)
    wait(50)
    MoveSyncGyro(90,10*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1.2,0,400))
    wait(50)
    MoveSyncGyro(-90,13*cm,1,1,1)
    wait(50)
    RobotCompas(90,30,STANGA,0,1,1,10)
    wait(50)
    run_task(clawGoTo(-82,1,0,400))
    MoveSyncGyro(90,11*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1.2,0,400))
    wait(50)
    run_task(liftGoTo(UP+10,3,1,400))
    wait(50)
    MoveSyncGyro(-90,22*cm,1,1,1)
    run_task(multitask(
        liftGoTo(DOWN,1.5,0,400),
        clawGoTo(OPEN,1,0,400),
        liftGoTo(UP,2,2,400)
    ))
    wait(50)
    MoveSyncGyro(90,8*cm,1,1,1)
    run_task(liftGoTo(DOWN,1.5,0,400))
    MoveSyncGyro(90,4*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1,0,400))
    RobotSpin(80,abs(imux - Brick.imu.rotation(TopAxis)),(imux - Brick.imu.rotation(TopAxis))/abs(imux - Brick.imu.rotation(TopAxis)),1,0,0,1)
    wait(50)
    MSGandCLOSE(90,30*cm,1,1,1,10)
#end SwitchLRthenIO

def SwitchIOthenLR():
    imux = Brick.imu.rotation(TopAxis)
    run_task(clawGoTo(CLOSED,1.2,0,400))
    wait(50)
    run_task(liftGoTo(UP+10,3,1,400))
    wait(50)
    MoveSyncGyro(-90,22*cm,1,1,1)
    run_task(multitask(
        liftGoTo(DOWN,1.5,0,400),
        clawGoTo(OPEN,1,0,400),
        liftGoTo(UP,2,2,400)
    ))
    wait(50)
    MoveSyncGyro(90,8*cm,1,1,1)
    run_task(liftGoTo(DOWN,1.5,0,400))
    run_task(clawGoTo(-80,1,0,400))
    MoveSyncGyro(-90,9*cm,1,1,1)
    wait(50)
    RobotCompas(90,15,DREAPTA,0,1,1,10)
    wait(50)
    MoveSyncGyro(90,10*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1.2,0,400))
    wait(50)
    MoveSyncGyro(-90,13*cm,1,1,1)
    wait(50)
    RobotCompas(90,30,STANGA,0,1,1,10)
    wait(50)
    run_task(clawGoTo(-82,1,0,400))
    MoveSyncGyro(90,11*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1.2,0,400))
    wait(50)
    RobotCompas(-90,25,STANGA,0,1,1,10)
    RobotSpin(80,abs(imux - Brick.imu.rotation(TopAxis)),(imux - Brick.imu.rotation(TopAxis))/abs(imux - Brick.imu.rotation(TopAxis)),1,0,0,1)
    wait(50)
    MSGandCLOSE(90,37*cm,1,1,1,10)
#end SwitchIOthenLR

def SwitchLefttoRight():
    imux = Brick.imu.rotation(TopAxis)
    run_task(clawGoTo(-80,1,0,400))
    MoveSyncGyro(-80,10*cm,1,1,1)
    wait(50)
    RobotCompas(80,15,DREAPTA,0,1,1,10)
    wait(50)
    MoveSyncGyro(80,10*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1.2,0,400))
    wait(50)
    MoveSyncGyro(-80,13*cm,1,1,1)
    wait(50)
    RobotCompas(80,30,STANGA,0,1,1,10)
    wait(50)
    run_task(clawGoTo(-82,1,0,400))
    MoveSyncGyro(80,11*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1.2,0,400))
    wait(50)
    RobotCompas(-80,25,STANGA,0,1,1,10)
    RobotSpin(65,abs(imux - Brick.imu.rotation(TopAxis)),(imux - Brick.imu.rotation(TopAxis))/abs(imux - Brick.imu.rotation(TopAxis)),1,0,0,1)
    wait(50)
#end SwitchLefttoRight

def SwitchInsidetoOutside():
    #NOTE: ASIGURA-TE CA AI MACAR 25 CM IN SPATE SI GHEARA INCHISA
    run_task(clawGoTo(CLOSED,2,0,600))
    run_task(liftGoTo(UP+10,3,1,600))
    wait(50)
    MoveSyncGyro(-70,22*cm,1,1,1)
    run_task(multitask(
        liftGoTo(DOWN,1.5,0,500),
        clawGoTo(OPEN,1,0,500),
        liftGoTo(UP,2,2,500)
    ))
    wait(50)
    MoveSyncGyro(70,9*cm,1,1,1)
    run_task(liftGoTo(DOWN,1.5,0,500))
    MoveSyncGyro(70,9*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1,0,500))
#end SwitchInsidetoOutside

def Cazuri(caz: str,mistake:float, imux: float):
    if caz == "drept":
        MoveSyncGyro(80,3*cm,1,1,1)
        wait(100)
        RobotCompas(65,89,STANGA,0,1,1,0)
        wait(100)
        SquaringWhiteSA(60,35,5,"red",1,0,0)
        MoveSyncGyro(60,5*cm,0,0,0)
        LFEncoderSA(50,14.5*cm,0.8,2.4,3,5,"red",0,0,0)
        MSGandCLOSE(42,8.5*cm,0,0,0,0,70)
        MSGandCLOSE(42,11*cm,0,1,1,35,-80)
        MoveSyncGyro(-80,20*cm,0,1,1)
        RobotCompas(-80,88,STANGA,0,0,0,0)
        MoveTime(-70,0.5,0,1)
    elif caz == "lateral":
        MoveSyncGyro(60,28*cm,1,1,1)
        wait(100)
        RobotSpin(80,90,DREAPTA,1,0,1,1)
        wait(100)
        MoveSyncGyro(-80,50*cm,1,0,0)
        MoveTime(-75,1,0,1)
        run_task(liftGoTo(UP,1,0,550))
        wait(100)
        MoveSyncGyro(60,10*cm,1,1,1)
        wait(100)
        # RobotSpin(80,89,DREAPTA,1,0,1,1)
        RobotCompas(80,20,DREAPTA,0,1,1,0)
        RobotCompas(-80,66,STANGA,0,1,1,0)
        MoveTime(-60,0.4,0,1)
        wait(100)
        MoveSyncGyro(50,7*cm,1,0,0)
        MSGandCLOSE(42,10*cm,0,0,0,35,-70)
        MoveTime(60,0.6,0,1)
        ArcMove(-95,15*cm,175,STANGA,0,1,1)
        MoveTime(-70,1,0,1)
    elif caz == "diagonal":
        run_task(clawGoTo(-80,1,0,500))
        MoveSyncGyro(80,10*cm,1,1,1)
        wait(100)
        RobotCompas(65,88,STANGA,0,1,1,0)
        wait(100)
        SquaringWhiteSA(60,40,5,"red",1,0,0)
        MSGandCLOSE(70,28*cm,0,1,1,0,40)
        wait(100)
        MSGandCLOSE(45,6*cm,0,1,1,0,-40)
        run_task(clawGoTo(CLOSED,1,0,500))
        MoveSyncGyro(-90,12*cm,1,1,1)
        wait(100)
        run_task(clawGoTo(-84,1,0,500))
        MoveSyncGyro(-90,9*cm,1,1,1)
        wait(100)
        RobotSpin(90,15,STANGA,1,0,1,1,5)
        wait(100)
        MoveSyncGyro(60,15*cm,1,1,1)
        run_task(clawGoTo(CLOSED,1.5,0,630))
        run_task(liftGoTo(UP+10,3,1,550))
        MoveSyncGyro(-90,9*cm,1,0,0)
        MSGandCLOSE(-90,12*cm,0,1,1,0,-45)
        run_task(clawGoTo(-86,1,0,500))
        run_task(liftGoTo(UP,1.3,1,550))
        MoveSyncGyro(80,7*cm,1,1,1)
        run_task(liftGoTo(DOWN,1.2,1,550))
        MoveSyncGyro(80,9*cm,1,1,1)
        RobotSpin(90,13,STANGA,1,0,1,1,5)
        wait(100)
        MoveSyncGyro(60,8*cm,1,1,1)
        run_task(clawGoTo(CLOSED,1,0,500))
        RobotSpin(90,26,DREAPTA,1,0,1,1)
        wait(100)
        MoveSyncGyro(-90,17*cm,1,1,1)
        wait(100)
        run_task(clawGoTo(CLOSED,1,0,150))
        wait(20)
        RobotSpin(90,90,DREAPTA,1,0,1,1)
        wait(100)
        MoveTime(-75,1,0,1)
#end Cazuri
    
def compute_wait_ms(fps, from_hub_fmt, per_byte_ms=1.5, overhead_ms=10):
    """
    fps:            Pixy frame‐rate in frames per second (e.g. 25)
    from_hub_fmt:   struct format string for the outbound payload (e.g. '2H')
    per_byte_ms:    empirical cost to send one byte over I²C (default 1.5 ms/byte)
    overhead_ms:    extra margin for I²C retries + processing (default 10 ms)

    Returns total wait time in ms, rounded *up* to the nearest multiple of 5.
    """
    # 1) How long between fresh frames:
    frame_period_ms = 1000.0 / fps               # e.g. 25 fps → 40.0 ms

    # 2) How long to send the outbound payload:
    payload_bytes   = calcsize(from_hub_fmt)  # e.g. '2H' → 4 bytes
    payload_time_ms = payload_bytes * per_byte_ms    # e.g. 4 * 1.5 ms = 6.0 ms

    # 3) Sum with overhead:
    total_ms = frame_period_ms + payload_time_ms + overhead_ms  # e.g. 40 + 6 + 10 = 56 ms

    # 4) Round *up* to the next 5 ms:
    return int(ceil(total_ms / 5.0) * 5)

WAIT_CL = compute_wait_ms(fps=15, from_hub_fmt='2H')
def ReadCubes(coords:tuple):
    for x, y in coords:
        closest_sig = pr.call('camCl', x, y, wait_ms=WAIT_CL)
        # print(f"Closest block to ({x:3}, {y:3}): sig={closest_sig}")
    return closest_sig

if __name__ == "__main__":
    print(f"NU MAI COMPILA FUNCTIILE")