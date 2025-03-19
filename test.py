from centy_Library import *

print(f"battery voltage : {Brick.battery.voltage()}")

#rn facem patratu ala mare cu parti colorate, 
Init()

heading = Brick.imu.rotation(TopAxis)
imux = heading + 90
#rosu
MoveSyncGyro(80,61.5*cm,1,1,1)
wait(50)
RobotSpin(75,abs(imux - Brick.imu.rotation(TopAxis)),(imux - Brick.imu.rotation(TopAxis))/abs(imux - Brick.imu.rotation(TopAxis)),1,0,0,1)
wait(50)
imux = heading - 90
run_task(clawGoTo(0,1,0,500))
MoveSyncGyro(80,33*cm,1,1,1)
run_task(clawGoTo(-200,1,0,500))
wait(50)
RobotCompas(-80,90,DREAPTA,0,0,1)
wait(150)
MoveSyncGyro(-90,9*cm,0,1,1)
wait(100)
RobotSpin(85,abs(imux - Brick.imu.rotation(TopAxis)),(imux - Brick.imu.rotation(TopAxis))/abs(imux - Brick.imu.rotation(TopAxis)),1,0,0,1)
MoveSyncGyro(80,23*cm,1,1,1)
print(imux,Brick.imu.rotation(TopAxis))