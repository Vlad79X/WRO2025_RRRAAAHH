from centy_Library import *

imux = Brick.imu.rotation(TopAxis)

print(f"battery voltage : {Brick.battery.voltage()}")
timer = StopWatch()
# Init()
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

cpatrat = 2

LFEncoderSA(60,10*cm,3,5,"red",1,0,0)
LFIntersectionSA(60,3,5,55,1,"red",0,0,0)
MoveSyncGyro(60,4*cm,0,1,1)

print(timer.time()/1000)



#rosu
#print(Sensor.reflection())
