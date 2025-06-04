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


MSGandCLOSE(80,60*cm,1,0,0,20)
SquaringWhiteSA(80,45,4,"red",0,0,0)
MoveSyncGyro(70,4*cm,0,1,1)
MoveSyncGyro(-70,4*cm,1,1,1)
wait(20)
RobotSpin(80,90,DREAPTA)
imux = Brick.imu.rotation(TopAxis)
wait(20)
MoveSyncGyro(-70,53*cm,1,1,1)
wait(20)
CompasTime(70,0.6,DREAPTA,1)
wait(20)
RobotCompas(-70,abs(imux - Brick.imu.rotation(TopAxis)),DREAPTA, 0, 1, 1, 20)
imux = imux - 90
wait(20)
print("gata gard galben in", timer.time()/1000, "secunde")
# gata gard galben

SquaringBlackSA(-70,30,3,"red",70*cm,0,0,1)
RobotCompas(-80,90,STANGA,0,1,1,10)
wait(50)
run_task(
    multitask(
        liftGoTo(8,1.5,0,500),
        clawGoTo(OPEN,1,0,500)
    )
)
MoveSyncGyro(60,23.5*cm,1,0,0)
MoveTime(60,0.4,0,1)
wait(50)
run_task(clawGoTo(CLOSED,1,0,550))
MoveSyncGyro(-85,11*cm,1,1,1)
run_task(clawGoTo(OPEN,2,0,550))
MoveSyncGyro(-85,10*cm,1,0,0)
MoveTime(-80,0.8,0,1)
run_task(liftGoTo(DOWN,1,0,450))
# luare nasi
wait(100)
MoveSyncGyro(70,14*cm,1,1,1)
wait(100)
RobotSpin(80,90,DREAPTA,1,0,1,1)
wait(50)
LFEncoderSA(65,6*cm,1,2,3,5,"red",1,0,0)
LFIntersectionSA(65,3,5,30,2,"red",0,0,0)
MoveSyncGyro(70,2*cm,0,0,0)
LFEncoderSA(65,10*cm,1,2,3,5,"red",0,1,1)
wait(100)
run_task(liftGoTo(UP,2,2,500))
wait(100)
MoveSyncGyro(60,14*cm,1,1,1)
run_task(clawGoTo(-30,1,0,500))
run_task(liftGoTo(DOWN,1.5,0,500))
wait(200)
MSGandCLOSE(50,5*cm,0,0,1,50)
wait(100)
# gata luare nasi

# lasare nas galben
MoveSyncGyro(-80,20*cm,1,1,1)
RobotSpin(90,90,STANGA)
wait(20)
LFEncoderSA(80,35*cm,1,2,3,5,"red",1,0,0)
MoveSyncGyro(80,11*cm,0,1,1)
RobotCompas(85,132,DREAPTA,0,1,1,25)
wait(20)
MoveSyncGyro(80,3*cm,0,1,1)
wait(20)
run_task(clawGoTo(OPEN,1,0,550))
wait(100)
# gata nas galben

# lasare nas rosu
MoveSyncGyro(-80,19*cm,1,1,1)
wait(100)
RobotCompas(80,65,STANGA,0,0,1,5)
wait(100)
MoveSyncGyro(70,14*cm,1,1,1)
run_task(liftGoTo(30,1.5,0,400))
MoveSyncGyro(-80,20*cm,1,1,1)
run_task(liftGoTo(DOWN,1.5,0,500))
MSGandCLOSE(60,9*cm,1,1,1,50)
RobotCompas(-70,135,STANGA,0,0,1,5)
MoveSyncGyro(-80,10*cm,1,0,0)
MoveTime(-70,0.5,0,1)
run_task(clawGoTo(OPEN,1,0,450))
wait(50)
MSGandCLOSE(80,16*cm,1,1,1,20)
MoveSyncGyro(-80,10*cm,1,0,0)
MoveTime(-70,0.5,0,1)

MoveSyncGyro(80,8.7*cm,1,1,1)
wait(100)
RobotSpin(80,90,DREAPTA,1,0,1,1)
wait(100)
MoveSyncGyro(-60,3*cm,1,1,1)
SquaringBlackSA(50,25,4,"red",10*cm,1,0,0)
MoveSyncGyro(60,2*cm,0,1,1)
wait(50)
run_task(clawGoTo(OPEN,1,0,450))


print(timer.time()/1000)