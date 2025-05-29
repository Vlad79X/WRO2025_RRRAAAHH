from centy_Library import *

print(f"battery voltage : {Brick.battery.voltage()}")
timer = StopWatch()
Init()
print(Brick.imu.rotation(TopAxis))

cpatrat = 0
switchleftright1,switchleftright2,switchinsideoutside,caz = GetSwitchesAndCase(1,2,4,3)



#   START

# steag alb
run_task(clawGoTo(CLOSED,1,0,500))
SquaringBlackSA(70,40,4,"red",15*cm,1,0,0)
LFEncoderSA(70,25*cm,3,5,"red",0,1,1)
wait(100)
imux = Brick.imu.rotation(TopAxis) - 90
RobotSpin(80,90,STANGA,1,0,1,1)
wait(100)
run_task(liftGoTo(40,2,1,500))
MoveSyncGyro(90,50*cm,1,1,1)
run_task(liftGoTo(10,1.2,0,400))
# gata steag alb

wait(100)
run_task(liftGoTo(35,2,0,500))
wait(100)
mistake = imux - Brick.imu.rotation(TopAxis)
RobotSpin(80,90 + mistake,DREAPTA,1,0,1,1)
wait(100)

# citire caz patrat
MoveSyncGyro(90,20*cm,1,1,1)
wait(100)
cpatrat = ReadCubes([(300,120)])
while cpatrat == 0:
    cpatrat = ReadCubes([(300,120)])

print("cazul patratului",cpatrat)
# gata citire caz patrat

wait(100)
RobotSpin(80,abs(imux - Brick.imu.rotation(TopAxis)),(imux - Brick.imu.rotation(TopAxis))/abs(imux - Brick.imu.rotation(TopAxis)),1,0,1,1)
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
MoveTime(-70,1,0,1)
run_task(liftGoTo(DOWN,1,0,550))
run_task(clawGoTo(0,1,0,550))

# citire caz bolti
MoveSyncGyro(60,10*cm,1,1,1)
wait(100)
mistake = imux - Brick.imu.rotation(TopAxis) + 180
RobotSpin(80,90,STANGA,1,0,1,1)
MoveSyncGyro(-70,3.5*cm,1,0,0)
MoveTime(-65,0.8,0,1)
wait(100)
imux = Brick.imu.rotation(TopAxis) + 180
MoveSyncGyro(60,3*cm,1,0,0)
SquaringBlackSA(60,40,4,"red",15*cm,0,0,1)
wait(200)
cub1 = ReadCubes([(50,200)])
cub2 = ReadCubes([(250,180)])
cub3 = ReadCubes([(100,100)])
cub4 = ReadCubes([(200,100)])
print(cub1,cub2,cub3,cub4)
switchleftright1,switchleftright2,switchinsideoutside,caz = GetSwitchesAndCase(cub1,cub2,cub3,cub4)
RobotSpin(80,90,DREAPTA,1,0,1,1)
wait(100)
MoveTime(-70,0.8,1,1)
# gata citire bolti

# patrat
if cpatrat == 1:
    # verde
    MoveSyncGyro(80,70*cm,1,0,0)
    SquaringBlackSA(70,35,3,"red",70*cm,0,0,1)
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
    RobotCompas(80,48,DREAPTA,0,1,1,30)
    wait(100)
    run_task(clawGoTo(CLOSED,1,0,450))
    MoveSyncGyro(80,15*cm,1,1,1)
    wait(100)
    RobotCompas(80,48,STANGA,0,1,1,30)
    wait(200)
    MoveSyncGyro(70,3*cm,1,0,0)
    SquaringBlackSA(70,35,3,"red",5*cm,0,0,0)
    MoveSyncGyro(70,3*cm,0,1,1)
    wait(2000)
    MoveSyncGyro(-75,5.5*cm,1,1,1)
    wait(100)
    RobotCompas(90, 88, DREAPTA, 0, 1, 1, 30)
    MoveSyncGyro(-80,18*cm,1,0,0)
    MoveTime(-70,0.8,0,1)
elif cpatrat == 2:
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
    run_task(clawGoTo(CLOSED,1,0,500))
    wait(100)
    RobotCompas(80, 88, STANGA, 0, 1, 1, 10)
    wait(100)
    MoveSyncGyro(-70,30*cm,1,0,0)
    SquaringBlackSA(-60,30,4,"red",45*cm,0,0,0)
    MoveSyncGyro(-60,4*cm,0,1,1)
    wait(200)
    MoveSyncGyro(80,28*cm,1,1,1)
    wait(100)
    RobotCompas(-80, 88, DREAPTA, 0, 1, 1, 20)
    wait(100)
    MoveSyncGyro(-80,7*cm,1,0,0)
    MoveTime(-70,1,0,1)
elif cpatrat == 3:
    # albastru
    MoveSyncGyro(80,87*cm,1,1,1)
    RobotSpin(70,90,STANGA,1,0,1,1)
    wait(100)
    MoveSyncGyro(-80,70*cm,0,0,0)
    MoveTime(-90,0.6,0,1)
    MoveSyncGyro(70,12*cm,1,1,1)
    RobotCompas(65, 88, STANGA, 0, 1, 1, 5)
    MoveSyncGyro(50,2*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1,0,500))
    wait(100)
    MoveSyncGyro(80,20*cm,1,0,0)
    SquaringBlackSA(70,30,4,"red",55*cm,0,0,0)
    MoveSyncGyro(70,3*cm,0,1,1)
    wait(200)
    MoveSyncGyro(-70,6*cm,1,1,1)
    wait(100)
    RobotCompas(90, 88, DREAPTA, 0, 1, 1, 30)
    MoveSyncGyro(-80,15*cm,1,0,0)
    MoveTime(-70,0.7,0,1)
else:
    # galben
    MoveSyncGyro(80,28*cm,1,1,1)
    wait(100)
    RobotSpin(80,90,STANGA,1,0,1,1)
    wait(100)
    MoveSyncGyro(-80,72*cm,1,0,0)
    MoveTime(-70,0.5,0,1)
    wait(100)
    MoveSyncGyro(80,11*cm,1,1,1)
    wait(100)
    RobotCompas(60, 88, DREAPTA, 0, 1, 1, 5)
    wait(100)
    MoveSyncGyro(50,5*cm,0,0,1)
    run_task(clawGoTo(CLOSED,1,0,450))
    wait(100)
    MoveSyncGyro(-60,8*cm,0,0,0)
    SquaringBlackSA(-60,30,4,"red",45*cm,0,0,0)
    MoveSyncGyro(-60,4*cm,0,1,1)
    wait(200)
    MoveSyncGyro(80,28*cm,1,1,1)
    wait(100)
    RobotCompas(-80, 88, DREAPTA, 0, 1, 1, 20)
    wait(100)
    MoveSyncGyro(-80,7*cm,1,0,0)
    MoveTime(-70,0.6,0,1)
#endif

run_task(clawGoTo(0,1,0,400))
wait(100)
MoveSyncGyro(60,2*cm,1,1,1)
run_task(clawGoTo(CLOSED,1,0,400))
wait(100)
# AICI E O PROBLEMA
LFEncoderSA(60,21*cm,2,1,"blue",1,0,0)
#LFEncoderSA(60,23*cm,3,1,"blue",1,1,1)
#LFEncoderSA(60,23*cm,2,1,"green/purple/red",1,1,1)
# MARE
MoveSyncGyro(70,27*cm,0,0,0)
SquaringBlackSA(60,30,2,"blue",45*cm,0,0,0)
MoveSyncGyro(50,4*cm,0,1,1)
wait(100)
MoveSyncGyro(-70,4*cm,1,1,1)
run_task(clawGoTo(OPEN,1,0,450))
wait(100)
print("gata patrat in",timer.time()/1000,"secunde")
# gata patrat

SMove(-80, 68, DREAPTA, 0, 1, 1, 0)
wait(100)
MoveSyncGyro(-85,50*cm,0,0,0)
MoveTime(-75,0.7,0,1)
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
run_task(clawGoTo(CLOSED,1,0,400))
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
SquaringWhiteSA(70,55,4,"red",0,0,0)
MoveSyncGyro(70,4*cm,0,1,1)
wait(100)
MoveSyncGyro(-70,4*cm,1,1,1)
wait(100)
RobotSpin(80,90,DREAPTA,1,0,1,1)
wait(100)
run_task(clawGoTo(CLOSED,1,0,500))
wait(100)
MoveSyncGyro(-70,53*cm,1,1,1)
wait(100)
CompasTime(70,1,DREAPTA,1)
wait(100)
RobotCompas(-50,abs(imux-Brick.imu.rotation(TopAxis)),DREAPTA, 0, 1, 1, 10)
wait(2000)
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
run_task(liftGoTo(30,1,1,500))
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
RobotSpin(70,90,DREAPTA,1,0,1,1)
wait(100)
LFEncoderSA(50,6*cm,3,5,"red",1,0,0)
LFIntersectionSA(70,3,5,35,2,"red",0,0,0)
MoveSyncGyro(70,2*cm,0,0,0)
LFEncoderSA(70,8*cm,3,5,"red",0,1,1)
wait(100)
# asta vreau sa schimb
run_task(liftGoTo(UP,1,1,500))
wait(100)
MoveSyncGyro(30,12*cm,1,1,1)
run_task(clawGoTo(-30,1,0,500))
run_task(liftGoTo(DOWN,1,0,500))
wait(100)
# pana aici
# gata luare nas rosu

# luare nas galben
# si asta trb schimbata
MoveSyncGyro(40,5*cm,1,0,0)
MSGandCLOSE(40,5*cm,0,0,1,30)
wait(100)
# pana aici
# gata luare nas galben

MoveSyncGyro(-80,20*cm,1,1,1)
RobotCompas(-80,88,STANGA,0,0,1,0)
MoveSyncGyro(-80,70*cm,1,0,0)
MoveTime(-70,0.5,0,1)
wait(100)

# lasare nas galben
MoveSyncGyro(80,21*cm,1,1,1)
wait(100)
RobotCompas(70,45,STANGA,0,0,1,0)
wait(100)
MoveSyncGyro(70,12*cm,1,1,1)
wait(100)
run_task(clawGoTo(OPEN,1,0,550))
wait(100)
# gata lasare nas galben

# lasare nas rosu
MoveSyncGyro(-70,16*cm,1,1,1)
wait(100)
RobotCompas(70,65,STANGA,0,0,1,5)
wait(100)
MoveSyncGyro(70,12*cm,1,1,1)
# gata lasare nas rosu

# THE END



print("FINAL",timer.time()/1000)
wait(20000)