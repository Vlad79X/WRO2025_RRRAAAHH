from centy_Library import *

print(f"battery voltage : {Brick.battery.voltage()}")
timer = StopWatch()
Init()

cpatrat = 0
switchleftright1,switchleftright2,switchinsideoutside,caz = GetSwitchesAndCase(1,2,4,3)



#   START

# steag alb
run_task(clawGoTo(CLOSED,1,0,500))
MoveSyncGyro(70,10*cm,1,0,0)
# SquaringBlackSA(70,40,4,"red",15*cm,1,0,0)
LFEncoderSA(70,23.8*cm,3,5,"red",0,1,1)
wait(100)
imux = Brick.imu.rotation(TopAxis) - 90
RobotSpin(80,90,STANGA,1,0,1,1)
wait(100)
run_task(liftGoTo(40,2,1,500))
MoveSyncGyro(90,50*cm,1,1,1)
run_task(liftGoTo(10,1.8,0,400))
# gata steag alb

wait(100)
run_task(liftGoTo(40,2,0,500))
wait(100)
mistake = imux - Brick.imu.rotation(TopAxis)
RobotSpin(80,90 + mistake,DREAPTA,1,0,1,1)
wait(100)
# citire caz patrat
MoveSyncGyro(90,18.8*cm,1,1,1)
wait(100)
cpatrat = ReadCubes([(300,120)])
while cpatrat == 0:
    cpatrat = ReadCubes([(300,120)])

print("cazul patratului",cpatrat)
# gata citire caz patrat

wait(100)
RobotSpin(80,abs(imux - Brick.imu.rotation(TopAxis))+5,(imux - Brick.imu.rotation(TopAxis))/abs(imux - Brick.imu.rotation(TopAxis)),1,0,1,1)
wait(100) 

# steag rosu
MoveSyncGyro(70,6*cm,1,1,1)
MoveTime(60,0.3,0,1)
run_task(clawGoTo(-20,1,0,500))
wait(100)
MoveSyncGyro(-70,9*cm,0,0,1)
wait(100)
print("gata steaguri in ",timer.time()/1000," secunde")
# gata steag rosu

imux = imux + 90
RobotSpin(85,abs(imux - Brick.imu.rotation(TopAxis)),(imux - Brick.imu.rotation(TopAxis))/abs(imux - Brick.imu.rotation(TopAxis)),1,0,1,1)
wait(100)
imux = imux - 90
MoveSyncGyro(80,7*cm,1,1,1)
wait(100)
RobotSpin(80,90,DREAPTA,1,0,1,1)
wait(100)
MoveSyncGyro(-80,20*cm,1,0,0)
MoveTime(-70,0.3,0,1)
run_task(liftGoTo(-2,1,0,550))
run_task(clawGoTo(0,1,0,550))

# citire caz bolti
MoveSyncGyro(60,10*cm,1,1,1)
wait(100)
mistake = imux - Brick.imu.rotation(TopAxis) + 180
RobotSpin(80,90,STANGA,1,0,1,1)
MoveSyncGyro(-70,3.5*cm,1,0,0)
MoveTime(-65,0.6,0,1)
wait(100)
MoveSyncGyro(60,3*cm,1,0,0)
SquaringBlackSA(60,40,4,"red",15*cm,0,0,1)
print(timer.time()/1000)
cub1 = ReadCubes([(50,200)])
cub2 = ReadCubes([(250,170)])
cub3 = ReadCubes([(100,100)])
cub4 = ReadCubes([(200,100)])

for i in range(1,4):
    if(cub1 != i and cub2 != i and cub3 != i):
        cub4 = i
    
print(cub1,cub2,cub3,cub4)
switchleftright1,switchleftright2,switchinsideoutside,caz = GetSwitchesAndCase(cub1,cub2,cub3,cub4)
RobotSpin(80,90,DREAPTA,1,0,1,1)
wait(100)
MoveTime(-70,0.7,1,1)
# gata citire bolti

# patrat
if cpatrat == 1:
    # verde
    MoveSyncGyro(90,26*cm,1,0,0)
    ArcMove(90,54*cm,90,DREAPTA,0,1,1)
    wait(100)
    mistake = imux - Brick.imu.rotation(TopAxis) + 270
    RobotSpin(80,90,DREAPTA,1,0,1,1)
    RobotSpin(60,abs(mistake),abs(mistake)/mistake*DREAPTA,1,0,0,1)
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
    RobotCompas(-80,60,STANGA,0,1,1,20)
    RobotSpinBlack(60,DREAPTA,30,1,25)
elif cpatrat == 2:
    # rosu
    MoveSyncGyro(80,22*cm,1,0,0)
    wait(100)
    ArcMove(90,31*cm,90,DREAPTA,0,1,1)
    wait(100)
    mistake = imux - Brick.imu.rotation(TopAxis) + 270
    RobotSpin(50,mistake,DREAPTA,1,0,0,1)
    wait(100)
    MSGandCLOSE(75,13*cm,1,0,0,50)
    MoveTime(70,0.4,0,1)
    wait(100)
    ArcMove(-90,11*cm,180,DREAPTA,0,1,1,25)
elif cpatrat == 3:
    # albastru
    MoveSyncGyro(90,34*cm,1,0,0)
    ArcMove(90,48*cm,90,DREAPTA,0,1,1)
    wait(100)
    mistake = imux - Brick.imu.rotation(TopAxis) + 270
    RobotSpin(80,90,DREAPTA,1,0,1,1)
    RobotSpin(60,abs(mistake),abs(mistake)/mistake*DREAPTA,1,0,0,1)
    MSGandCLOSE(85,23*cm,1,0,0,20)
    wait(100)
    SquaringBlackSA(85,25,4,"red",35*cm,0,0,0)
    MoveSyncGyro(75,18*cm,0,1,1)
    wait(100)
    mistake = imux - Brick.imu.rotation(TopAxis) + 360
    RobotCompas(-80,60,STANGA,0,1,1,20)
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
kpLFSA = 1.6
kdLFSA = 2.4
LFEncoderSA(50,6*cm,3,5,"red",1,1,1)
wait(100)
run_task(clawGoTo(CLOSED,1,0,400))
wait(50)
MoveSyncGyro(-80,22*cm,1,0,0)
MoveTime(-80,0.6,0,1)
wait(50)
LFEncoderSA(65,26*cm,3,5,"red",1,1,1)
wait(50)
kdLFSA = 1.65
kpLFSA = 0.5
SMove(90,48,DREAPTA,0,1,1,20)
wait(100)
MoveSyncGyro(70,20*cm,1,0,0)
SquaringBlackSA(60,30,2,"red",40*cm,0,0,0)
MoveSyncGyro(50,4*cm,0,1,1)
wait(100)
MoveSyncGyro(-70,4*cm,1,1,1)
run_task(clawGoTo(OPEN,1,0,450))
wait(100)

print("gata patrat in",timer.time()/1000,"secunde")
# gata patrat

SMove(-80, 60, DREAPTA, 0, 1, 1, 0)
wait(100)
MoveSyncGyro(-85,50*cm,0,0,0)
MoveTime(-75,0.7,0,1)
wait(100)
print("acum cazuri", timer.time()/1000)
# luare bolti
imux = imux + 360
mistake = imux - Brick.imu.rotation(TopAxis)
print(imux, Brick.imu.rotation(TopAxis))
Cazuri(caz,mistake,imux)
# gata luare bolti

wait(100)
MoveSyncGyro(80,28*cm,1,1,1)
wait(100)

# switch-uri
if switchinsideoutside:
    dist = 26.5
    if switchleftright2:
        SwitchLefttoRight()
        dist = 28
        switchleftright2 = False
    SwitchInsidetoOutside()
    if switchleftright1:
        SwitchLefttoRight()
        dist = 28
else:
    dist = 28
    if switchleftright1:
        dist = 18
        SwitchLefttoRight()

print("hai la campeni", timer.time()/1000)

run_task(clawGoTo(OPEN-70,1,0,450))
wait(100)
MSGandCLOSE(80,dist*cm,1,1,1,25)
wait(100)
# gata switch-uri

RobotCompas(80,88,DREAPTA,1,1,1,20)
wait(200)
SMove(-80,50,STANGA,0,1,1,30)
wait(200)
MoveSyncGyro(-80,20*cm,1,0,0)
MoveTime(-70,0.8,0,1)
wait(100)

# lasare bolti 1 si 2
MoveSyncGyro(80,8.5*cm,1,1,1)
wait(100)
mistake = imux - Brick.imu.rotation(TopAxis)+180
RobotSpin(80,90,STANGA,1,0,1,1)
wait(100)
MoveSyncGyro(-50,7*cm,1,1,1)
wait(100)
SquaringBlackSA(50,30,4,"red",10*cm,0,0,0)
MoveSyncGyro(50,2.5*cm,0,1,1)
wait(100)
run_task(clawGoTo(OPEN,1,0,550))
wait(100)
MoveSyncGyro(-60,3*cm,1,1,1)
wait(100)
RobotSpin(80,90,DREAPTA,1,0,1,1)
wait(100)
MoveTime(-70,0.5,0,1)
wait(100)
print("gata bolti 1 si 2 in", timer.time()/1000, "secunde")
# gata lasare bolti 1 si 2

# gard galben
MoveSyncGyro(80,60*cm,1,0,0)
SquaringWhiteSA(80,55,4,"red",0,0,0)
MoveSyncGyro(70,4*cm,0,1,1)
wait(100)
MoveSyncGyro(-70,4*cm,1,1,1)
wait(100)
RobotSpin(80,90,DREAPTA,1,0,1,1)
imux = Brick.imu.rotation(TopAxis)
wait(100)
run_task(clawGoTo(CLOSED,1,0,500))
wait(100)
MoveSyncGyro(-70,53*cm,1,1,1)
wait(100)
CompasTime(70,1,DREAPTA,1)
wait(50)
print(imux,Brick.imu.rotation(TopAxis))
RobotCompas(-70,abs(imux - Brick.imu.rotation(TopAxis)),DREAPTA, 0, 1, 1, 20)
imux = imux - 90
wait(50)
print("gata gard galben in", timer.time()/1000, "secunde")
# gata gard galben

SquaringBlackSA(-70,35,3,"red",70*cm,0,0,0)
MoveSyncGyro(-70,46*cm,0,0,0)
SquaringBlackSA(-70,35,3,"red",10*cm,0,0,0)
MoveSyncGyro(-70,2*cm,0,1,1)
wait(200)
RobotCompas(80,85,STANGA,0,1,1,15)
wait(100)
MoveSyncGyro(-80,60*cm,1,0,0)
MoveTime(-60,0.8,0,1)

# switch
run_task(clawGoTo(OPEN,1,0,550))
wait(100)
mistake = imux - Brick.imu.rotation(TopAxis) + 180
MoveSyncGyro(80,25*cm,1,1,1)
run_task(liftGoTo(UP,1.5,1,550))
wait(100)
MoveSyncGyro(-70,25*cm,1,1,1)
wait(100)
run_task(liftGoTo(DOWN,1,1,450))
wait(100)
MoveSyncGyro(70,10*cm,1,0,0)
MSGandCLOSE(70,20*cm,0,1,1,40)
if switchleftright2:
    SwitchLefttoRight()
wait(100)
MoveSyncGyro(-80,25*cm,1,0,0)
MoveTime(-70,0.5,0,1)
# gata switch

# lasare bolti 3 si 4
MoveSyncGyro(80,8.5*cm,1,1,1)
wait(100)
RobotSpin(80,90,DREAPTA,1,0,1,1)
wait(100)
MoveSyncGyro(-60,5*cm,1,1,1)
SquaringBlackSA(50,30,4,"red",10*cm,1,0,0)
MoveSyncGyro(50,2*cm,0,1,1)
wait(100)
run_task(clawGoTo(OPEN,1,0,450))
wait(100)
MoveSyncGyro(-60,10*cm,1,1,1)
RobotCompas(60,90,STANGA,0,1,1,10)
wait(100)
MoveTime(-60,1,0,1)
print("gata bolti 3 si 4 in", timer.time()/1000, "secunde")
# gata lasare bolti 3 si 4

# gard rosu
MoveSyncGyro(80,52*cm,1,1,1)
wait(100)
RobotSpin(70,90,DREAPTA,1,0,1,1)
wait(100)
MoveSyncGyro(70,7*cm,1,0,0)
SquaringWhiteSA(70,50,4,"red",0,0,0)
MoveSyncGyro(70,10*cm,0,0,1)
wait(100)
run_task(liftGoTo(25,1,1,500))
RobotSpin(70,90,DREAPTA,1,0,1,1)
wait(100)
MoveTime(70,0.5,1,1)
wait(100)
run_task(clawGoTo(CLOSED,1,0,550))
wait(100)
MoveSyncGyro(-70,12*cm,1,1,1)
run_task(clawGoTo(OPEN,1,0,550))
wait(100)
print("gata gard rosu in", timer.time()/1000, "secunde")
# gata gard rosu

MoveSyncGyro(-80,20*cm,1,0,0)
MoveTime(-70,1,0,1)
run_task(liftGoTo(DOWN,1,0,450))

# luare nas rosu
wait(100)
MoveSyncGyro(70,15*cm,1,1,1)
wait(100)
RobotSpin(70,70,DREAPTA,1,0,1,1)
RobotSpinBlack(70,DREAPTA,35,1)
wait(100)
LFEncoderSA(50,6*cm,3,5,"red",1,0,0)
LFIntersectionSA(70,3,5,35,2,"red",0,0,0)
MoveSyncGyro(70,2*cm,0,0,0)
LFEncoderSA(70,8*cm,3,5,"red",0,1,1)
wait(100)
run_task(liftGoTo(UP,1.3,1,500))
wait(100)
MoveSyncGyro(60,17.2*cm,1,1,1)
run_task(clawGoTo(-30,1,0,500))
run_task(liftGoTo(DOWN,1.5,0,500))
wait(100)
# gata luare nas rosu

# luare nas galben
MoveSyncGyro(40,5*cm,1,0,0)
MSGandCLOSE(40,8*cm,0,0,1,30)
wait(100)
# gata luare nas galben

MoveSyncGyro(-80,20*cm,1,1,1)
RobotCompas(-80,88,STANGA,0,0,1,0)
MoveSyncGyro(-80,70*cm,1,0,0)
MoveTime(-70,0.5,0,1)
wait(100)

# lasare nas galben
MoveSyncGyro(80,23*cm,1,1,1)
wait(100)
RobotCompas(70,45,STANGA,0,0,1,0)
wait(100)
MoveSyncGyro(70,10*cm,1,1,1)
wait(100)
run_task(clawGoTo(OPEN,1,0,550))
wait(100)
# gata lasare nas galben

# lasare nas rosu
MoveSyncGyro(-70,15*cm,1,1,1)
wait(100)
RobotCompas(70,65,STANGA,0,0,1,5)
wait(100)
MoveSyncGyro(70,12*cm,1,1,1)
# gata lasare nas rosu

# THE END



print("FINAL",timer.time()/1000)
wait(20000)