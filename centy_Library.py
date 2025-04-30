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

from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Axis,Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.hubs import PrimeHub
from pybricks.tools import wait, StopWatch, multitask, run_task
from micropython import const
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
V0 = const(45)
Vmax = const(95)
V0rot = const(35)
V0comp = 40
V0arc = const(55)

#Acceleratie si deceleratie mers fata spate
AccelerationEncoder = const(250)
DecelerationEncoder = const(100)

#Acceleratie si deceleratie rotiri
AccelerationEncoderRot = const(10)
DecelerationEncoderRot = const(70)

#Acceleratie si deceleratie compas
AccelerationEncoderComp = const(10)
DecelerationEncoderComp = const(40)

AccelerationEncoderArc = const(30)
DecelerationEncoderArc = const(60)

#Valori culori
white = const(90)
black = const(25)

#K-uri pentru LineFollower
kpLF = const(0.17)
kdLF = const(4.5)
kiLF = const(0)

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
kpSPB = const(0.2)
kdSPB = const(0.5)
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
rotationTimer = const(279)

#---Declarations----------------------------------------------------------------

#Motoare
LeftMotor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
RightMotor = Motor(Port.C)

LiftMotor = Motor(Port.A)

ClawMotor = Motor(Port.D)


#Brick
Brick = InventorHub(TopAxis, FrontAxis)
InventorHub()
Navigation = DriveBase(LeftMotor, RightMotor, wheelDiameter, wheelDistance)

#Camera = ColorSensor(Port.C)
Sensor = ColorSensor(Port.F)

#RotateTo(Color) PID Timer
timerPIDBlack = StopWatch()

#---Functions-------------------------------------------------------------------

def mm2deg(mm: int) -> int:
    return mm / (3.1415 * wheelDiameter) * 360
#end mm2deg

def deg2mm(degree: int) -> int:
    return degree * (360 / (3.1415 * wheelDiameter))
#end deg2mm

def Init():
    Brick.imu.reset_heading(0)
    wait(300)
    ClawMotor.reset_angle(0)
    LiftMotor.reset_angle(0)
    run_task(clawGoTo(-250,1,0,500))
#end init

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
        wait(50)
    #endwhile
#end CheckGyro

def ArcMove(speedext: int, raza: int, grade: float, direction: int, accel: bool, decel: bool, brake: bool):
    imux = Brick.imu.rotation(Axis.Y)
    speedsemn = speedext / abs(speedext)

    ratio = abs((raza - wheelDistance/2)/(raza + wheelDistance/2))

    speedint = speedext * ratio

    V0compL = V0comp
    V0compR = V0comp
    
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
    
    print(leftSpeed,rightSpeed)

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
        print(CurAngle,FinalAngle,AngleEncoder,speedL,speedR)
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
    exitCondition = 1

    while exitCondition == 1:
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
        print(V + (P + I + D), V - (P + I + D),Error)
        
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
        LeftMotor.hold()
        RightMotor.hold()
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
        LeftMotor.hold()
        RightMotor.hold()
    #endif

#end MoveSync

def RobotSpin(speed: int, grade:float, direction: int, pid:bool, accel: bool, decel: bool, brake: bool):
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
        LeftMotor.hold()
        RightMotor.hold()
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
    print("PID ACUM")
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
        LeftMotor.hold()
        RightMotor.hold()
    #endif
    
#end RobotSpin


def RobotCompas(speed: int, grade: float, direction: int, accel: bool, decel: bool, brake: bool, turbo: int):
    encoder = ((wheelDistance * 3.14) * grade ) / 360
    FinalEncoder = mm2deg(encoder)


    LeftMotor.reset_angle()
    RightMotor.reset_angle()

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

#end RobotCompas

def SMove(speed: int, grade: float, direction: int, accel: bool, decel: bool, brake: bool):
    RobotCompas(speed, grade, direction, accel, decel, brake, 0)
    wait(100)
    RobotCompas(speed, grade, 0 - direction, accel, decel, brake, 0)
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
    LeftMotor.reset_angle()
    RightMotor.reset_angle()
    
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
        LeftMotor.hold()
        RightMotor.hold()
    #endif

#end MoveTime

# ┌ 𝙙𝙚 𝙛𝙪𝙣𝙘𝙩𝙞𝙖 𝙖𝙨𝙩𝙖 𝙣𝙪 𝙢𝙖 𝙖𝙩𝙞𝙣𝙜 ┐ -𝓿𝓵𝓪𝓭
def RobotSpinBlack(speed: int, direction: int, endBrake: bool):

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
    # RobotSpin(speed,10,direction*-1,0,0,1)
    timerPID = StopWatch()
    time = 1000
    error = 0
    errorSum = 0
    errorOld = 0

    exitCondition = 1

    while exitCondition == 1:
        #print(timerPID.time())
        s1 = LeftSensor.reflection()
        s2 = RightSensor.reflection()

        error = s1 - s2
        errorSum = errorSum + error
        errorOld = error
        Pb = kpSPB * error
        Db = kdSPB * (error - errorOld)
        Ib = kiSPB * errorSum

        LeftMotor.dc(-(Pb + Db + Ib))
        RightMotor.dc(Pb + Db + Ib)

        if timerPIDBlack.time() > time:
            exitCondition = 0
        #endif

    #endwhile

    if endBrake == True:
        LeftMotor.brake()
        RightMotor.brake()

        LeftMotor.hold()
        RightMotor.hold()
        Navigation.straight(0,Stop.HOLD)
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

        Pa = kpANG * ErrorAngle
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
        LeftMotor.hold()
        RightMotor.hold()
    #endif

#end SquaringWhite

def SquaringBlack(speed: int, black: int, accel: bool, aliniere: bool, brake: bool):
    IniAngle = Brick.imu.rotation(TopAxis)
    
    V = ErrorEncoder = ErrorEncoderSum = ErrorEncoderOld = ErrorAngle = ErrorAngleSum = ErrorAngleOld = CurEncoder = 0

    if speed < 0:
        sens = -1
    else:
        sens = 1
    #endif

    speed = abs(speed)

    while(Sensor.reflection() > black): #and RightSensor.reflection() > black):
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

        Pa = kpANG * ErrorAngle
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
        LeftMotor.hold()
        RightMotor.hold()
    #endif

#end SquaringBlack

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
        LeftMotor.hold()
        RightMotor.hold()
    #endif

#end MoveSyncGyro
