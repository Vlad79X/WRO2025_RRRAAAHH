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

run_task(clawGoTo(CLOSED,1,0,400))
RobotSpin(90,90,DREAPTA,1,0,1,1,20,)
wait(100)
MoveSyncGyro(90,6*cm,1,0,0)
wait(100)
RobotSpin(90,90,STANGA,1,0,1,1,20,5)
wait(100)
# SMove(95,53,DREAPTA,0,1,1,25)
# wait(100)
MoveSyncGyro(90,30*cm,1,0,0)

print(timer.time()/1000)