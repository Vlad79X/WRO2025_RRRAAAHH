from centy_Library import *

imux = Brick.imu.rotation(TopAxis)

print(f"battery voltage : {Brick.battery.voltage()}")
timer = StopWatch()
Init()
SquaringBlack(60,20,70*cm,0,0,0)

print(timer.time()/1000)



#rosu
#print(Sensor.reflection())
