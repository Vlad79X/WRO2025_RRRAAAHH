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


# MoveSyncGyro(60,2*cm,1,1,1)
# run_task(clawGoTo(CLOSED,1,0,400))
# LFEncoderSA(70,23*cm,2,1,"blue",1,0,0)
# MoveSyncGyro(70,25*cm,0,0,0)
# SquaringBlackSA(60,30,2,"red",50*cm,0,0,0)
# MoveSyncGyro(50,1*cm,0,1,1)
# wait(100)
# #MoveSyncGyro(-70,2*cm,0,0,1)
# run_task(clawGoTo(OPEN,1,0,450))
# wait(100)


print(timer.time()/1000)



#rosu
#print(Sensor.reflection())
