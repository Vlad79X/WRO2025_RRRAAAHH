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
Init()
imux = Brick.imu.rotation(TopAxis) 
cub1 = 4
cub2 = 1
cub3 = 3
cub4 = 2

switchleftright1,switchleftright2,switchinsideoutside,caz = GetSwitchesAndCase(cub1,cub2,cub3,cub4)

run_task(clawGoTo(OPEN,1,0,450))
MSGandCLOSE(70,14*cm,1,1,1,40)
wait(200)
MoveSyncGyro(-70,14*cm,1,1,1)
wait(100)
RobotSpin(90,91,STANGA)
wait(50)
MoveSyncGyro(90,80*cm,1,1,1)
run_task(clawGoTo(OPEN,1,0,450))
MoveSyncGyro(-90,80*cm,1,1,1)
RobotCompas(-80,89.5,STANGA,0,1,1,10)