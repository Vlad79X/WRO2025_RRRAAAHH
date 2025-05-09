from centy_Library import *

imux = Brick.imu.rotation(TopAxis)

print(f"battery voltage : {Brick.battery.voltage()}")
timer = StopWatch()
Init()


MoveSyncGyro(80,3.5*cm,1,1,1)
wait(50)
RobotCompas(70, 87, STANGA, 0, 1, 1, 0)
wait(100)
LF1SEncoder(55, 13*cm, DREAPTA, 1, 0, 0, 85)
MoveSyncGyro(60,7.5*cm,0,1,1)


print(timer.time()/1000)



#rosu
#print(Sensor.reflection())
