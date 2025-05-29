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
imux = Brick.imu.rotation(TopAxis) - 180 
cpatrat = 4
run_task(liftGoTo(-2,1,0,450))
if cpatrat == 1:
    # verde
    MoveSyncGyro(90,26*cm,1,0,0)
    ArcMove(90,54*cm,90,DREAPTA,0,1,1)
    wait(100)
    mistake = imux - Brick.imu.rotation(TopAxis) + 270
    RobotSpin(80,mistake + 90,DREAPTA,1,0,1,1)
    MoveSyncGyro(80,8*cm,1,0,0)
    RobotCompas(80,28,DREAPTA,0,1,1,10)
    wait(100)
    MSGandCLOSE(85,7*cm,1,0,0,45)
    RobotCompas(80,30,STANGA,0,1,1,10)
    wait(100)
    SquaringBlackSA(80,28,4,"red",35*cm,1,0,0)
    MoveSyncGyro(75,10*cm,0,1,1)
    wait(100)
    mistake = imux - Brick.imu.rotation(TopAxis) + 360
    RobotSpin(60,65-mistake,DREAPTA,0,0,0,1)
    RobotSpinBlack(60,DREAPTA,30,1,25)
elif cpatrat == 2:
    # rosu
    MoveSyncGyro(80,22*cm,1,0,0)
    wait(100)
    ArcMove(90,31*cm,90,DREAPTA,0,1,1)
    wait(100)
    mistake = imux - Brick.imu.rotation(TopAxis) + 270
    RobotSpin(70,mistake,DREAPTA,1,0,0,1)
    wait(100)
    MSGandCLOSE(75,13*cm,1,0,0,50)
    MoveTime(70,0.4,0,1)
    wait(100)
    ArcMove(-90,11.5*cm,180,DREAPTA,0,1,1,25)
elif cpatrat == 3:
    # albastru
    MoveSyncGyro(90,34*cm,1,0,0)
    ArcMove(90,48*cm,90,DREAPTA,0,1,1)
    wait(100)
    mistake = imux - Brick.imu.rotation(TopAxis) + 270
    RobotSpin(80,mistake + 90,DREAPTA,1,0,1,1)
    MSGandCLOSE(85,23*cm,1,0,0,20)
    wait(100)
    SquaringBlackSA(85,25,4,"red",35*cm,0,0,0)
    MoveSyncGyro(75,15*cm,0,1,1)
    wait(100)
    mistake = imux - Brick.imu.rotation(TopAxis) + 360
    RobotSpin(60,65-mistake,DREAPTA,0,0,0,1)
    RobotSpinBlack(60,DREAPTA,35,1,25)
else:
    # galben
    MoveSyncGyro(90,10*cm,1,0,0)
    ArcMove(90,25*cm,90,DREAPTA,0,1,1)
    wait(100)
    mistake = imux - Brick.imu.rotation(TopAxis) + 270
    RobotSpin(80,mistake,DREAPTA,1,0,1,1)
    wait(100)
    LFEncoderSA(70,10*cm,3,5,"red",1,0,0)
    LFIntersectionSA(70,3,5,30,1,"red",0,0,1)
    wait(100)
    RobotSpin(90,90,STANGA,1,0,1,1)
    MSGandCLOSE(60,12*cm,1,1,1,40)
    wait(100)
    MoveSyncGyro(-80,14*cm,1,1,1)
    wait(100)
    RobotSpin(90,90,STANGA,1,0,1,1)
#endif

run_task(clawGoTo(0,1,0,400))
wait(100)
MoveSyncGyro(60,1*cm,1,1,1)
run_task(clawGoTo(CLOSED,1,0,400))
wait(100)
kpLFSA = 1.5
MoveSyncGyro(-70,15*cm,1,1,1)
LFEncoderSA(60,20*cm,3,5,"red",1,1,1)
SMove(90,45,DREAPTA,0,1,1,25)
wait(100)
MoveSyncGyro(70,20*cm,1,0,0)
SquaringBlackSA(60,30,2,"red",40*cm,0,0,0)
MoveSyncGyro(50,4*cm,0,1,1)
wait(100)
MoveSyncGyro(-70,4*cm,1,1,1)
run_task(clawGoTo(OPEN,1,0,450))
wait(100)
print(timer.time()/1000)



#rosu
#print(Sensor.reflection())
