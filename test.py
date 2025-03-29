from centy_Library import *

print(f"battery voltage : {Brick.battery.voltage()}")

#rn facem patratu ala mare cu parti colorate, 
Init()

heading = Brick.imu.rotation(TopAxis)
imux = heading + 90
#rosu
MoveSyncGyro(80,59*cm,1,1,1)
wait(100)
RobotSpin(75,abs(imux - Brick.imu.rotation(TopAxis)),(imux - Brick.imu.rotation(TopAxis))/abs(imux - Brick.imu.rotation(TopAxis)),1,0,1,1)
wait(100)
imux = heading - 90
run_task(clawGoTo(0,1,0,500))
MoveSyncGyro(80,27*cm,1,1,0)
MoveSyncGyro(50,6*cm,0,1,1)
run_task(clawGoTo(-200,1,0,500))
wait(50)
ArcMove(-90,10*cm,180,DREAPTA,0,1,1)
wait(100)
MoveSyncGyro(-90,20*cm,0,0,0)
MoveTime(-90,1,0,1)
print(imux,Brick.imu.rotation(TopAxis))
