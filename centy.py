from centy_Library import *

print(f"battery voltage : {Brick.battery.voltage()}")
timer = StopWatch()
Init()
print(Brick.imu.rotation(TopAxis))



#   START

MoveTime(-80,0.6,0,1)
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
run_task(liftGoTo(40,1,0,500))
MoveSyncGyro(90,50*cm,1,1,1)
run_task(liftGoTo(0,1,0,500))
# gata steag alb

wait(100)
run_task(liftGoTo(40,1,0,500))
wait(100)
mistake = imux - Brick.imu.rotation(TopAxis)
RobotSpin(80,90 + mistake,DREAPTA,1,0,1,1)
wait(100)
MoveSyncGyro(90,19.3*cm,1,1,1)
wait(100)
RobotSpin(80,abs(imux - Brick.imu.rotation(TopAxis)),(imux - Brick.imu.rotation(TopAxis))/abs(imux - Brick.imu.rotation(TopAxis)),1,0,1,1)
wait(100)

# steag rosu
MoveSyncGyro(75,7*cm,1,1,1)
MoveTime(75,0.3,0,1)
run_task(clawGoTo(-70,1,0,500))
wait(100)
MoveSyncGyro(-70,9*cm,0,0,1)
wait(100)
# gata steag rosu

print("gata steag in",timer.time()/1000,"secunde")

imux = imux + 90
RobotSpin(85,abs(imux - Brick.imu.rotation(TopAxis)),(imux - Brick.imu.rotation(TopAxis))/abs(imux - Brick.imu.rotation(TopAxis)),1,0,1,1)
wait(100)
imux = imux - 90
MoveSyncGyro(80,3*cm,1,1,1)
wait(100)
RobotSpin(80,89,DREAPTA,1,0,1,1)
wait(100)
MoveSyncGyro(-80,25*cm,1,0,0)
MoveTime(-80,0.6,0,1)

switchleftright1,switchleftright2,switchinsideoutside,caz = GetSwitchesAndCase('r','a','g','v')

mistake = imux - Brick.imu.rotation(TopAxis) + 180
run_task(liftGoTo(0,0.7,0,500))
run_task(clawGoTo(0,1,0,500))

# patrat
cpatrat = 1

if cpatrat == 1:
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
    SquaringBlack(-60,15,70*cm,0,0,1)
    MoveSyncGyro(-60,7*cm,0,1,1)
    wait(100)
    MoveSyncGyro(80,29.2*cm,1,1,1)
    wait(100)
    RobotCompas(-90, 88, DREAPTA, 0, 1, 1, 30)
    wait(100)
    MoveTime(-90,1,0,1)
elif cpatrat == 2:
    # galben
    MoveSyncGyro(80,31*cm,1,1,1)
    wait(100)
    RobotSpin(80,90,STANGA,1,0,1,1)
    wait(100)
    MoveSyncGyro(-80,90*cm,1,0,0)
    MoveTime(-90,0.6,0,1)
    MoveSyncGyro(70,12*cm,1,1,1)
    RobotCompas(70, 88, DREAPTA, 0, 1, 1, 0)
    MoveSyncGyro(50,2*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1,0,450))
    wait(100)
    SquaringBlack(-60,15,70*cm,0,0,0)
    MoveSyncGyro(-60,7*cm,0,1,1)
    wait(100)
    MoveSyncGyro(80,29.2*cm,1,1,1)
    RobotCompas(-90, 90, DREAPTA, 0, 1, 1, 30)
    wait(100)
    MoveTime(-90,1,0,1)
elif cpatrat == 3:
    # albastru
    MoveSyncGyro(80,86*cm,1,1,1)
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
    SquaringBlack(80,30,70*cm,0,0,0)
    MoveSyncGyro(65,3*cm,0,1,1)
    wait(100)
    MoveSyncGyro(-75,6*cm,0,1,1)
    wait(100)
    RobotCompas(90, 88, DREAPTA, 0, 1, 1, 30)
    MoveTime(-90,1,0,1)
else:
    # verde
    MoveSyncGyro(80,70*cm,1,0,0)
    SquaringBlack(80,20,30*cm,0,1,1)
    wait(100)
    RobotSpin(70,90,STANGA,1,0,1,1)
    wait(100)
    MoveSyncGyro(-90,70*cm,1,0,0)
    MoveTime(-90,0.6,0,1)
    MoveSyncGyro(70,5*cm,1,1,1)
    wait(100)
    RobotSpin(70,90,STANGA,1,0,1,1)
    wait(100)
    MoveSyncGyro(70,5*cm,1,1,1)
    wait(100)
    RobotCompas(80,45,DREAPTA,0,1,1,10)
    wait(100)
    run_task(clawGoTo(CLOSED,1,0,450))
    MoveSyncGyro(80,15*cm,1,1,1)
    wait(10)
    RobotCompas(80,45,STANGA,0,1,1,10)
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
LF1SEncoder(65, 23*cm, DREAPTA, 0, 0, 1, 70)
wait(100)
SMove(90, 36, DREAPTA, 0, 0, 1, 15)
wait(100)
MoveSyncGyro(70,25*cm,1,0,0)
SquaringBlack(60, 25, 33*cm, 0, 0, 0)
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
RobotCompas(80,88,DREAPTA,0,1,1,10)
wait(100)
SMove(-90, 63, STANGA, 0, 0, 1, 10)
wait(100)
MoveSyncGyro(-80,20*cm,1,0,0)
MoveTime(-80,0.8,0,1)
wait(100)
# lasare bolti 1 si 2
MoveSyncGyro(80,8.5*cm,1,1,1)
wait(100)
mistake = imux - Brick.imu.rotation(TopAxis)+180
RobotSpin(80,90,STANGA,1,0,1,1)
wait(100)
MoveSyncGyro(-50,4*cm,1,1,1)
wait(100)
SquaringBlack(50, 50,10*cm, 0, 0, 0)
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
SquaringWhite(80,90,0,0,1)
wait(200)
MoveSyncGyro(-80,4*cm,1,1,1)
wait(100)
RobotSpin(80,90,DREAPTA,1,0,1,1)
wait(100)
run_task(clawGoTo(CLOSED,1,0,500))

# gard galben
wait(100)
MoveSyncGyro(-70,53*cm,1,1,1)
wait(100)
CompasTime(80,1,DREAPTA,1)
wait(100)
RobotCompas(-60,35,DREAPTA, 0, 1, 1, 20)
wait(100)
# gata gard galben
print("gata gard galben in", timer.time()/1000, "secunde")

SquaringBlack(-70,30,70*cm,0,0,0)
MoveSyncGyro(-70,37*cm,0,1,1)
wait(200)
RobotSpin(80,90,STANGA,1,0,1,1)
wait(100)
MoveSyncGyro(-80,60*cm,1,0,0)
MoveTime(-60,0.8,0,1)
run_task(clawGoTo(OPEN,1,0,550))
wait(100)
mistake = imux - Brick.imu.rotation(TopAxis) + 180
MoveSyncGyro(80,20*cm,1,1,1)
run_task(liftGoTo(30,1,0,450))
wait(100)
MoveSyncGyro(-80,13*cm,1,1,1)
wait(100)
run_task(liftGoTo(0,0.7,0,450))
wait(100)
MoveSyncGyro(70,3*cm,1,0,0)
MSGandCLOSE(70,25*cm,0,1,1,40)
if switchleftright2:
    SwitchLefttoRight()
wait(100)

MoveSyncGyro(-80,30*cm,1,0,0)
MoveTime(-60,0.8,0,1)

# lasare bolti 3 si 4
MoveSyncGyro(80,8.5*cm,1,1,1)
wait(100)
RobotSpin(80,90,DREAPTA,1,0,1,1)
wait(100)
MoveSyncGyro(60,15*cm,1,0,0)
SquaringBlack(50,30,10*cm, 0, 0, 0)
MoveSyncGyro(50,1.5*cm,0,1,1)
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
SquaringWhite(70,90,0,0,0)
MoveSyncGyro(70,10*cm,0,0,1)
wait(100)
run_task(liftGoTo(30,1,0,500))
RobotSpin(70,90,DREAPTA,1,0,1,1)
wait(100)

# gard rosu
MoveSyncGyro(70,3*cm,1,1,1)
run_task(clawGoTo(CLOSED,1,0,550))
wait(100)
MoveSyncGyro(-70,12*cm,1,1,1)
run_task(clawGoTo(OPEN,1,0,550))
# gata gard rosu
print("gata gard rosu in", timer.time()/1000, "secunde")

SMove(-80,80,DREAPTA,0,1,1,0)
wait(200)
MoveSyncGyro(-80,20*cm,1,0,0)
MoveTime(-80,1,0,1)
run_task(liftGoTo(DOWN,0.7,0,500))
wait(100)
MoveSyncGyro(70,8*cm,1,1,1)
RobotCompas(80,88,DREAPTA,1,1,1,0)
wait(100)

# luare nas rosu
LF1SEncoder(50, 10*cm, DREAPTA, 1, 1, 1, 85)
run_task(liftGoTo(UP,1,0,500))
wait(100)
MoveSyncGyro(40,8*cm,1,1,1)
run_task(liftGoTo(20,0.7,0,500))
wait(100)
MoveSyncGyro(-80,10*cm,1,1,1)
run_task(liftGoTo(UP,1,0,500))
wait(100)
MoveSyncGyro(35,5*cm,1,1,1)
run_task(clawGoTo(-70,1,0,500))
wait(100)
run_task(liftGoTo(DOWN,0.7,0,500))
wait(100)
# gata luare nas rosu

# luare nas galben
MoveSyncGyro(40,15*cm,1,0,0)
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
MoveSyncGyro(80,22*cm,1,1,1)
wait(100)
RobotCompas(70,45,STANGA,0,0,1,0)
wait(100)
MoveSyncGyro(70,10*cm,1,1,1)
wait(100)
run_task(clawGoTo(OPEN,1,0,550))
wait(100)
# gata lasare nas galben

#lasare nas rosu
MoveSyncGyro(-70,15*cm,1,1,1)
wait(100)
RobotCompas(70,65,STANGA,0,0,1,0)
wait(100)
MoveSyncGyro(70,11*cm,1,1,1)
# gata lasare nas rosu

# THE END



print("final in",timer.time()/1000)
wait(20000)