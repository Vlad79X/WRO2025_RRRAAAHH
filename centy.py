from centy_Library import *

print(f"battery voltage : {Brick.battery.voltage()}")
timer = StopWatch()
Init()
print(Brick.imu.rotation(TopAxis))

cpatrat = 2
switchleftright1,switchleftright2,switchinsideoutside,caz = GetSwitchesAndCase(1,2,4,3)

#   START

MoveTime(-80,0.5,0,1)
imux = Brick.imu.rotation(TopAxis)
wait(100)
RobotCompas(90,80,DREAPTA,0,1,1,0)
wait(100)
MoveSyncGyro(80,12*cm,1,1,1)
wait(100)
RobotCompas(90,80,STANGA,0,1,1,0)
wait(150)
RobotSpin(70,abs(imux - Brick.imu.rotation(TopAxis)),(imux - Brick.imu.rotation(TopAxis))/abs(imux - Brick.imu.rotation(TopAxis)),1,0,0,1)
wait(150)

# steag alb
run_task(liftGoTo(40,2,0,500))
MoveSyncGyro(90,50*cm,1,1,1)
run_task(liftGoTo(10,1,0,300))
# gata steag alb

wait(200)
run_task(liftGoTo(35,2,0,500))
wait(100)
mistake = imux - Brick.imu.rotation(TopAxis)
RobotSpin(80,90 + mistake,DREAPTA,1,0,1,1)
wait(100)
MoveSyncGyro(90,18*cm,1,1,1)
wait(100)
cpatrat = ReadCubes([(300,120)])
while cpatrat == 0:
    cpatrat = ReadCubes([(300,120)])

print("cazul patratului",cpatrat)
wait(100)
RobotSpin(80,abs(imux - Brick.imu.rotation(TopAxis)),(imux - Brick.imu.rotation(TopAxis))/abs(imux - Brick.imu.rotation(TopAxis)),1,0,1,1)
wait(100)

# steag rosu
MoveSyncGyro(75,7*cm,1,1,1)
MoveTime(75,0.3,0,1)
run_task(clawGoTo(-50,1,0,500))
wait(100)
MoveSyncGyro(-70,9*cm,0,0,1)
wait(100)
# gata steag rosu

print("gata steaguri in ",timer.time()/1000," secunde")

imux = imux + 90
RobotSpin(85,abs(imux - Brick.imu.rotation(TopAxis)),(imux - Brick.imu.rotation(TopAxis))/abs(imux - Brick.imu.rotation(TopAxis)),1,0,1,1)
wait(100)
imux = imux - 90
MoveSyncGyro(80,3*cm,1,1,1)
wait(100)
RobotSpin(80,89,DREAPTA,1,0,1,1)
wait(100)
MoveSyncGyro(-80,20*cm,1,0,0)
MoveTime(-80,0.6,0,1)
run_task(liftGoTo(0,0.7,1,500))
run_task(clawGoTo(0,1,0,500))
MoveSyncGyro(80,10*cm,1,1,1)
wait(50)
mistake = imux - Brick.imu.rotation(TopAxis) + 180
RobotSpin(80,90,STANGA,1,0,1,1)
MoveSyncGyro(-70,8*cm,1,1,1)
wait(100)
SquaringBlackSA(60,40,4,"red",15*cm,1,0,1)
wait(100)
cub1 = ReadCubes([(50,200)])
wait(50)
cub2 = ReadCubes([(250,180)])
wait(50)
cub3 = ReadCubes([(100,100)])
wait(50)
cub4 = ReadCubes([(180,100)])
wait(50)
print(cub1,cub2,cub3,cub4)
switchleftright1,switchleftright2,switchinsideoutside,caz = GetSwitchesAndCase(cub1,cub2,cub3,cub4)
# SquaringBlackSA(-60,30,3,"red",6*cm,1,0,1,)
RobotSpin(80,90,DREAPTA,1,0,1,1)
wait(50)
MoveTime(-70,0.8,1,1)

# patrat
if cpatrat == 2:
    # rosu
    MoveSyncGyro(80,58*cm,1,1,1)
    wait(100)
    imux = imux + 270
    RobotSpin(70,90,DREAPTA,1,0,1,1)
    wait(100)
    imux = imux - 270
    run_task(clawGoTo(0,1,0,500))
    MoveSyncGyro(80,27*cm,1,1,0)
    MoveSyncGyro(50,10*cm,0,1,1)
    run_task(clawGoTo(-200,1,0,500))
    wait(100)
    RobotCompas(90, 88, STANGA, 0, 1, 1, 10)
    wait(100)
    MoveSyncGyro(-90,15*cm,0,0,0)
    SquaringBlackSA(-60,30,4,"red",70*cm,0,0,0)
    MoveSyncGyro(-60,7*cm,0,1,1)
    wait(100)
    MoveSyncGyro(80,29.2*cm,1,1,1)
    wait(100)
    RobotCompas(-90, 88, DREAPTA, 0, 1, 1, 30)
    wait(100)
    MoveTime(-90,1,0,1)
elif cpatrat == 4:
    # galben
    MoveSyncGyro(80,31*cm,1,1,1)
    wait(100)
    RobotSpin(80,90,STANGA,1,0,1,1)
    wait(100)
    MoveSyncGyro(-80,70*cm,1,0,0)
    MoveTime(-90,0.5,0,1)
    MoveSyncGyro(80,12*cm,1,1,1)
    RobotCompas(80, 88, DREAPTA, 0, 1, 1, 0)
    MoveSyncGyro(60,2*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1,0,450))
    wait(100)
    MoveSyncGyro(-60,8*cm,0,0,0)
    SquaringBlackSA(-60,40,4,"red",50*cm,0,0,0)
    MoveSyncGyro(-60,7*cm,0,1,1)
    wait(100)
    MoveSyncGyro(80,29.2*cm,1,1,1)
    RobotCompas(-90, 90, DREAPTA, 0, 1, 1, 30)
    wait(100)
    MoveTime(-90,1,0,1)
elif cpatrat == 3:
    # albastru
    MoveSyncGyro(80,87*cm,1,1,1)
    RobotSpin(70,90,STANGA,1,0,1,1)
    wait(100)
    MoveSyncGyro(-80,70*cm,0,0,0)
    MoveTime(-90,0.6,0,1)
    MoveSyncGyro(70,12*cm,1,1,1)
    RobotCompas(65, 88, STANGA, 0, 1, 1, 0)
    MoveSyncGyro(50,2*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1,0,500))
    wait(100)
    MoveSyncGyro(80,25*cm,1,0,0)
    SquaringBlackSA(70,30,4,"red",70*cm,0,0,0)
    MoveSyncGyro(65,3*cm,0,1,1)
    wait(100)
    MoveSyncGyro(-75,6*cm,0,1,1)
    wait(100)
    RobotCompas(90, 88, DREAPTA, 0, 1, 1, 30)
    MoveTime(-90,1,0,1)
else:
    # verde
    MoveSyncGyro(80,70*cm,1,0,0)
    SquaringBlackSA(70,30,4,"red",70*cm,0,0,1)
    wait(100)
    RobotSpin(70,90,STANGA,1,0,1,1)
    wait(100)
    MoveSyncGyro(-90,70*cm,1,0,0)
    MoveTime(-90,0.6,0,1)
    MoveSyncGyro(70,5*cm,1,1,1)
    wait(100)
    RobotSpin(70,90,STANGA,1,0,1,1)
    wait(100)
    MoveSyncGyro(70,7*cm,1,1,1)
    wait(100)
    RobotCompas(80,50,DREAPTA,0,1,1,10)
    wait(100)
    run_task(clawGoTo(CLOSED,1,0,450))
    MoveSyncGyro(80,15*cm,1,1,1)
    wait(10)
    RobotCompas(80,50,STANGA,0,1,1,10)
    wait(100)
    SquaringBlack(80,30,70*cm,0,0,0)
    MoveSyncGyro(65,3*cm,0,1,1)
    wait(100)
    MoveSyncGyro(-75,6*cm,0,1,1)
    wait(100)
    RobotCompas(90, 88, DREAPTA, 0, 1, 1, 30)
    MoveTime(-90,1,0,1)
#endif

run_task(clawGoTo(SEMIOPEN+20,1,0,400))
MoveSyncGyro(60,2*cm,1,1,1)
run_task(clawGoTo(CLOSED,1,0,400))
LFEncoderSA(70,23*cm,2,1,"blue",1,0,0)
MoveSyncGyro(70,25*cm,0,0,0)
SquaringBlackSA(60,30,2,"red",30*cm,0,0,0)
MoveSyncGyro(50,2*cm,0,1,1)
wait(100)
MoveSyncGyro(-70,2*cm,0,0,1)
run_task(clawGoTo(OPEN,1,0,450))
wait(100)
# gata patrat
print("gata patrat in",timer.time()/1000,"secunde")

SMove(-80, 70, DREAPTA, 0, 1, 1, 0)
wait(100)
MoveSyncGyro(-85,50*cm,0,0,0)
MoveTime(-85,0.7,0,1)
wait(100)

# luare bolti
mistake = imux - Brick.imu.rotation(TopAxis) + 90
Cazuri(caz,mistake,imux)
# gata luare bolti

wait(100)
run_task(clawGoTo(OPEN-100,1,0,450))
wait(100)
MSGandCLOSE(80,30*cm,1,1,1,15)
wait(100)
run_task(clawGoTo(CLOSED,1,0,100))
wait(10)
if switchinsideoutside:
    dist = 23.5
    if switchleftright2:
        SwitchLefttoRight()
        dist = 25
        switchleftright2 = False
    SwitchInsidetoOutside()
    if switchleftright1:
        SwitchLefttoRight()
        dist = 25
else:
    dist = 25
    if switchleftright1:
        dist = 15
        SwitchLefttoRight()
run_task(clawGoTo(OPEN-70,1,0,450))
wait(10)
MSGandCLOSE(80,dist*cm,1,1,1,25)
wait(100)
RobotCompas(80,88,DREAPTA,1,1,1,20)
wait(200)
SMove(-80,63,STANGA,0,1,1,30)
wait(200)
MoveSyncGyro(-80,20*cm,1,0,0)
MoveTime(-80,0.8,0,1)
wait(100)

# lasare bolti 1 si 2
MoveSyncGyro(80,8*cm,1,1,1)
wait(100)
mistake = imux - Brick.imu.rotation(TopAxis)+180
RobotSpin(80,90,STANGA,1,0,1,1)
wait(100)
MoveSyncGyro(-50,4*cm,1,1,1)
wait(100)
SquaringBlackSA(50,30,4,"red",10*cm,0,0,0)
MoveSyncGyro(50,1.5*cm,0,1,1)
wait(100)
run_task(clawGoTo(OPEN,1,0,550))
wait(100)
MoveSyncGyro(-60,3*cm,1,1,1)
wait(100)
RobotSpin(80,90,DREAPTA,1,0,1,1)
wait(100)
MoveTime(-70,0.5,0,1)
wait(100)
# gata lasare bolti 1 si 2

print("gata bolti 1 si 2 in", timer.time()/1000, "secunde")

MoveSyncGyro(80,60*cm,1,0,0)
SquaringWhiteSA(70,70,4,"red",0,0,1)
wait(200)
MoveSyncGyro(-80,4*cm,1,1,1)
wait(100)
RobotSpin(80,90,DREAPTA,1,0,1,1)
wait(100)
run_task(clawGoTo(CLOSED,1,0,500))

imux = imux + 270

# gard galben
wait(100)
MoveSyncGyro(-70,53*cm,1,1,1)
wait(100)
CompasTime(80,1,DREAPTA,1)
wait(100)
RobotCompas(-60,abs(imux-Brick.imu.rotation(TopAxis)),DREAPTA, 0, 1, 1, 20)
wait(100)
# gata gard galben
print("gata gard galben in", timer.time()/1000, "secunde")

SquaringBlackSA(-70,30,4,"red",70*cm,0,0,0)
MoveSyncGyro(-70,46*cm,0,0,0)
SquaringBlackSA(-70,30,4,"red",10*cm,0,0,0)
MoveSyncGyro(-70,5*cm,0,1,1)
wait(100)
RobotCompas(80,88,STANGA,0,1,1,15)
wait(100)
MoveSyncGyro(-80,60*cm,1,0,0)
MoveTime(-60,0.8,0,1)
run_task(clawGoTo(OPEN,1,0,550))
wait(100)
mistake = imux - Brick.imu.rotation(TopAxis) + 180
MoveSyncGyro(80,20*cm,1,1,1)
run_task(liftGoTo(30,1,1,450))
wait(100)
MoveSyncGyro(-70,17*cm,1,1,1)
wait(100)
run_task(liftGoTo(0,0.7,1,450))
wait(100)
MoveSyncGyro(70,3*cm,1,0,0)
MSGandCLOSE(70,20*cm,0,1,1,40)
if switchleftright2:
    SwitchLefttoRight()
wait(100)

MoveSyncGyro(-80,25*cm,1,0,0)
MoveTime(-70,0.5,0,1)

# lasare bolti 3 si 4
MoveSyncGyro(80,8.2*cm,1,1,1)
wait(100)
RobotSpin(80,90,DREAPTA,1,0,1,1)
wait(100)
MoveSyncGyro(-60,5*cm,1,1,1)
SquaringBlackSA(50,30,4,"red",10*cm,1,0,0)
MoveSyncGyro(50,2*cm,0,1,1)
wait(100)
run_task(clawGoTo(OPEN,1,0,450))
wait(100)
MoveSyncGyro(-60,8*cm,1,1,1)
RobotCompas(60,90,STANGA,0,1,1,1)
wait(100)
MoveTime(-60,1,0,1)
# gata lasare bolti 3 si 4
print("gata bolti 3 si 4 in", timer.time()/1000, "secunde")

MoveSyncGyro(80,50*cm,1,1,1)
wait(100)
RobotSpin(70,90,DREAPTA,1,0,1,1)
wait(100)
MoveSyncGyro(70,7*cm,1,0,0)
SquaringWhiteSA(70,70,4,"red",0,0,0)
MoveSyncGyro(70,10*cm,0,0,1)
wait(100)
run_task(liftGoTo(30,1,1,500))
RobotSpin(70,90,DREAPTA,1,0,1,1)
wait(100)

# gard rosu
MoveTime(70,0.5,1,1)
wait(100)
run_task(clawGoTo(CLOSED,1,0,550))
wait(100)
MoveSyncGyro(-70,12*cm,1,1,1)
run_task(clawGoTo(OPEN,1,0,550))
# gata gard rosu
print("gata gard rosu in", timer.time()/1000, "secunde")

SMove(-90,80,DREAPTA,0,0,1,30)
wait(200)
MoveSyncGyro(-80,20*cm,1,0,0)
MoveTime(-80,1,0,1)
run_task(liftGoTo(DOWN,0.7,1,500))
wait(100)
MoveSyncGyro(70,8*cm,1,1,1)
RobotCompas(80,88,DREAPTA,1,0,1,0)
wait(100)

# luare nas rosu
LFEncoderSA(60,8*cm,3,5,"red",1,1,1)
run_task(liftGoTo(UP,1,1,500))
wait(100)
MoveSyncGyro(40,5*cm,1,1,1)
run_task(liftGoTo(20,0.7,1,500))
wait(100)
MoveSyncGyro(-80,10*cm,1,1,1)
run_task(liftGoTo(UP,1,1,500))
wait(100)
MoveSyncGyro(35,5*cm,1,1,1)
run_task(clawGoTo(-70,1,0,500))
wait(100)
run_task(liftGoTo(DOWN,0.7,1,500))
wait(100)
# gata luare nas rosu

# luare nas galben
MoveSyncGyro(40,12*cm,1,0,0)
MoveTime(40,1,0,1)
wait(100)
MoveSyncGyro(-40,1*cm,0,0,1)
run_task(clawGoTo(CLOSED,1,0,550))
wait(100)
# gata luare nas galben

MoveSyncGyro(-80,20*cm,1,1,1)
RobotCompas(-80,88,STANGA,0,0,1,0)
MoveSyncGyro(-80,70*cm,1,0,0)
MoveTime(-70,0.5,0,1)
wait(100)

# lasare nas galben
MoveSyncGyro(80,26*cm,1,1,1)
wait(100)
RobotCompas(70,45,STANGA,0,0,1,0)
wait(100)
MoveSyncGyro(70,7*cm,1,1,1)
wait(100)
run_task(clawGoTo(OPEN,1,0,550))
wait(100)
# gata lasare nas galben

#lasare nas rosu
MoveSyncGyro(-70,15*cm,1,1,1)
wait(100)
RobotCompas(70,65,STANGA,0,0,1,0)
wait(100)
MoveSyncGyro(70,15*cm,1,1,1)
# gata lasare nas rosu

# THE END



print("FINAL",timer.time()/1000)
wait(20000)