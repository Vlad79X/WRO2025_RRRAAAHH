from centy_Library import *

print(f"battery voltage : {Brick.battery.voltage()}")

Init()
MoveSyncGyro(-80,6*cm,1,0,0)
MoveTime(-70,0.3,0,1)
MoveSyncGyro(90,13*cm,1,1,1)
wait(100)
SMove(70,110,DREAPTA,1,1,1)
run_task(liftGoTo(UP,2,0,400))
MoveSyncGyro(90,42*cm,1,1,1)
run_task(liftGoTo(DOWN,1.5,0,400))