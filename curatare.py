

from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Axis,Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.hubs import PrimeHub
from pybricks.tools import wait, StopWatch, multitask, run_task
from micropython import const
from umath import ceil

LeftMotor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
RightMotor = Motor(Port.C)

while 1==1:
    LeftMotor.run(1000)
    RightMotor.run(1000)

