from centy_Library import *

print(f"battery voltage : {Brick.battery.voltage()}")
timer = StopWatch()
Init()
print(Brick.imu.rotation(TopAxis))



#   START

MoveTime(-80,0.6,0,1)
imux = Brick.imu.rotation(TopAxis)
wait(50)
RobotCompas(80,80,DREAPTA,0,1,1,0)
wait(50)
MoveSyncGyro(80,10*cm,1,1,1)
wait(50)
RobotCompas(80,80,STANGA,0,1,1,0)
wait(150)
RobotSpin(70,abs(imux - Brick.imu.rotation(TopAxis)),(imux - Brick.imu.rotation(TopAxis))/abs(imux - Brick.imu.rotation(TopAxis)),1,0,0,1)
wait(150)

# steag alb
run_task(liftGoTo(40,1,0,500))
MoveSyncGyro(80,50*cm,1,1,1)
run_task(liftGoTo(0,1,0,500))
# gata steag alb

wait(100)
run_task(liftGoTo(40,1,0,500))
wait(100)
mistake = imux - Brick.imu.rotation(TopAxis)
RobotSpin(80,90 + mistake,DREAPTA,1,0,1,1)
wait(100)
MoveSyncGyro(80,19.2*cm,1,1,1)
wait(100)
RobotSpin(80,abs(imux - Brick.imu.rotation(TopAxis)),(imux - Brick.imu.rotation(TopAxis))/abs(imux - Brick.imu.rotation(TopAxis)),1,0,1,1)
wait(100)

# steag rosu
MoveSyncGyro(60,7*cm,1,1,1)
MoveTime(60,0.6,0,1)
run_task(clawGoTo(-70,1,0,500))
wait(100)
MoveSyncGyro(-70,9*cm,0,0,1)
wait(100)
# gata steag rosu

imux = imux + 90
RobotSpin(85,abs(imux - Brick.imu.rotation(TopAxis)),(imux - Brick.imu.rotation(TopAxis))/abs(imux - Brick.imu.rotation(TopAxis)),1,0,1,1)
wait(50)
imux = imux - 90
MoveSyncGyro(55,4*cm,1,1,1)
wait(50)
RobotSpin(70,89,DREAPTA,1,0,1,1)
MoveSyncGyro(-70,25*cm,1,1,1)
MoveTime(-80,0.6,0,1)

switchleftright1,switchleftright2,switchinsideoutside,caz = GetSwitchesAndCase('g','r','a','v')

mistake = imux - Brick.imu.rotation(TopAxis) + 180
run_task(clawGoTo(0,1,0,500))
run_task(liftGoTo(0,0.7,0,500))

# patrat
cpatrat = 1

if cpatrat == 1:
    # rosu
    MoveSyncGyro(80,58*cm,1,1,1)
    wait(100)
    imux = imux + 270
    RobotSpin(70,90,DREAPTA,1,0,1,1)
    wait(50)
    imux = imux - 270
    run_task(clawGoTo(0,1,0,500))
    MoveSyncGyro(80,27*cm,1,1,0)
    MoveSyncGyro(50,10*cm,0,1,1)
    run_task(clawGoTo(-200,1,0,500))
    wait(50)
    RobotCompas(90, 88, STANGA, 0, 1, 1, 10)
    wait(50)
    MoveSyncGyro(-90,15*cm,0,0,0)
    SquaringBlack(-60,10,70*cm,0,0,1)
    wait(100)
    MoveSyncGyro(75,23*cm,1,1,1)
    wait(100)
    RobotCompas(-90, 88, DREAPTA, 0, 1, 1, 15)
    wait(50)
    MoveTime(-90,1,0,1)
elif cpatrat == 2:
    # galben
    MoveSyncGyro(80,31*cm,1,1,1)
    wait(50)
    RobotSpin(80,90,STANGA,1,0,1,1)
    wait(50)
    MoveSyncGyro(-80,70*cm,1,0,0)
    MoveTime(-90,1,0,1)
    MoveSyncGyro(70,12*cm,1,1,1)
    RobotCompas(70, 88, DREAPTA, 0, 1, 1, 0)
    MoveSyncGyro(50,2*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1,0,450))
    wait(50)
    SquaringBlack(-60,15,70*cm,0,0,1)
    wait(50)
    MoveSyncGyro(60,23.5*cm,1,1,1)
    RobotCompas(-90, 90, DREAPTA, 0, 1, 1, 15)
    wait(100)
    MoveTime(-90,1,0,1)
elif cpatrat == 3:
    # albastru
    MoveSyncGyro(80,86*cm,1,1,1)
    RobotSpin(70,90,STANGA,1,0,1,1)
    MoveSyncGyro(-80,70*cm,0,0,0)
    MoveTime(-90,0.6,0,1)
    MoveSyncGyro(70,12*cm,1,1,1)
    RobotCompas(65, 88, STANGA, 0, 1, 1, 0)
    MoveSyncGyro(50,2*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1,0,500))
    wait(50)
    MoveSyncGyro(80,25*cm,1,0,0)
    SquaringBlack(80,30,70*cm,0,0,0)
    MoveSyncGyro(65,3*cm,0,1,1)
    wait(100)
    MoveSyncGyro(-75,6*cm,0,1,1)
    wait(100)
    RobotCompas(90, 88, DREAPTA, 0, 1, 1, 20)
    MoveTime(-90,1,0,1)
else:
    # verde
    MoveSyncGyro(80,59*cm,1,1,1)
    wait(100)
    imux = imux + 270
    RobotSpin(70,90,DREAPTA,1,0,1,1)
    wait(50)
    imux = imux - 270
    run_task(clawGoTo(0,1,0,500))
    MoveSyncGyro(80,27*cm,1,1,0)
    MoveSyncGyro(50,10*cm,0,1,1)
    run_task(clawGoTo(-200,1,0,500))
    wait(50)
    MoveSyncGyro(-75,6*cm,0,0,0)
    MoveSyncGyro(-90,15*cm,0,1,1)
    wait(50)
    run_task(clawGoTo(OPEN,1,0,500))
    MoveSyncGyro(-90,4*cm,1,1,1)
    wait(50)
    RobotSpin(80,90,DREAPTA,1,0,1,1)
    wait(50)
    ArcMove(80,25*cm,184,STANGA,0,1,1)
    wait(100)
    mistake = imux - Brick.imu.rotation(TopAxis) + 180
    #MoveSyncGyro(50,1*cm,0,1,1)
    wait(50)
    RobotSpin(80,100,STANGA,1,0,1,1)
    wait(50)
    MoveSyncGyro(-80,8*cm,1,0,0)
    MoveTime(-80,0.5,0,1)
    wait(50)
    MoveSyncGyro(80,15*cm,1,1,0)
    MoveSyncGyro(60,5*cm,0,1,1)
    wait(50)
    run_task(clawGoTo(CLOSED,1,0,450))
    wait(50)
    RobotCompas(-90, 90, STANGA, 0, 1, 1, 10)
    MoveSyncGyro(-60, 22*cm, 0, 0, 0)
    SquaringBlack(-60,30,70*cm,0,0,0)
    MoveSyncGyro(-60,3*cm,0,1,1)
    MoveSyncGyro(60,26*cm,1,1,1)
    RobotCompas(-90, 90, DREAPTA, 0, 1, 1, 15)
    MoveTime(-90,1,0,1)

run_task(clawGoTo(SEMIOPEN+20,1,0,400))
MoveSyncGyro(60,2*cm,1,1,1)
run_task(clawGoTo(CLOSED,1,0,400))
LF1SEncoder(55, 23*cm, DREAPTA, 0, 0, 1, 70)
wait(100)
SMove(80, 40, DREAPTA, 0, 0, 1, 15)
wait(100)
MoveSyncGyro(70,25*cm,1,0,0)
SquaringBlack(60, 25, 36*cm, 0, 0, 0)
MoveSyncGyro(50,2*cm,0,1,1)
wait(100)
MoveSyncGyro(-70,2*cm,0,0,1)
run_task(clawGoTo(OPEN,1,0,450))
wait(50)
# gata patrat

SMove(-80, 70, DREAPTA, 0, 1, 1, 0)
wait(100)
MoveSyncGyro(-70,50*cm,0,0,0)
MoveTime(-70,1,0,1)
wait(50)

# luare bolti

mistake = imux - Brick.imu.rotation(TopAxis) + 90
Cazuri(caz,mistake)
# gata luare bolti
wait(50)
run_task(clawGoTo(OPEN-100,1,0,450))
wait(50)
MSGandCLOSE(60,25*cm,1,1,1,20)
wait(50)
if switchinsideoutside:
    dist = 40
    if switchleftright2:
        SwitchLefttoRight()
        dist = 30
        switchleftright2 = False
    SwitchInsidetoOutside()
    if switchleftright1:
        SwitchLefttoRight()
        dist = 30
else:
    dist = 38
    if switchleftright1:
        dist = 20
        SwitchLefttoRight()
MoveSyncGyro(80,dist*cm,1,1,1)
wait(100)
RobotCompas(80,88,DREAPTA,0,1,1,0)
wait(50)
SMove(-80, 57, STANGA, 0, 0, 1, 0)
wait(50)
MoveSyncGyro(-80,20*cm,1,0,0)
MoveTime(-80,0.8,0,1)
wait(50)
# lasare bolti 1 si 2
MoveSyncGyro(80,7.5*cm,1,1,1)
wait(50)
mistake = imux - Brick.imu.rotation(TopAxis)+180
RobotSpin(80,90,STANGA,1,0,1,1)
wait(100)
MoveSyncGyro(-50,4*cm,1,1,1)
wait(100)
SquaringBlack(50, 50,10*cm, 0, 0, 0)
MoveSyncGyro(50,1.5*cm,0,1,1)
wait(50)
run_task(clawGoTo(OPEN,1,0,550))
wait(50)
RobotSpin(33,5,STANGA,1,0,1,1)
wait(50)
RobotSpin(40,12,DREAPTA,1,0,1,1)
wait(50)
RobotSpin(33,5,STANGA,1,0,1,1)
MoveSyncGyro(-60,3*cm,1,1,1)
wait(100)
RobotSpin(80,90,DREAPTA,1,0,1,1)
wait(50)
MoveTime(-70,0.5,0,1)
wait(50)
# gata lasare bolti 1 si 2

MoveSyncGyro(80,60*cm,1,0,0)
SquaringWhite(80,90,0,0,1)
wait(200)
MoveSyncGyro(-80,5*cm,1,1,1)
wait(50)
RobotSpin(80,90,DREAPTA,1,0,1,1)
wait(100)
run_task(clawGoTo(CLOSED,1,0,500))

# gard galben
MoveSyncGyro(-70,53*cm,1,1,1)
wait(100)
CompasTime(80,0.5,DREAPTA,1)
wait(100)
RobotCompas(-60,30,DREAPTA, 0, 1, 1, 20)
wait(100)
# gata gard galben

MoveSyncGyro(-70,46*cm,1,0,0)
SquaringBlack(-70,30,70*cm,0,0,0)
MoveSyncGyro(-50,5*cm,0,1,1)
wait(200)
RobotSpin(80,90,STANGA,1,0,1,1)
wait(50)
MoveSyncGyro(-80,60*cm,1,0,0)
MoveTime(-60,0.8,0,1)
run_task(clawGoTo(OPEN,1,0,550))
wait(50)
mistake = imux - Brick.imu.rotation(TopAxis) + 180
MoveSyncGyro(80,20*cm)
run_task(liftGoTo(30,1,0,450))
wait(50)
MoveSyncGyro(-80,13*cm,1,1,1)
wait(50)
run_task(liftGoTo(0,0.7,0,450))
wait(50)
MSGandCLOSE(70,13*cm,1,1,1,20)
if switchleftright2:
    SwitchLefttoRight()

MoveSyncGyro(-80,60*cm,1,0,0)
MoveTime(-60,0.8,0,1)
# lasare bolti 3 si 4
MoveSyncGyro(80,7*cm,1,1,1)
wait(50)
RobotSpin(80,90,DREAPTA,1,0,1,1)
wait(100)
SquaringBlack(50, 30,10*cm, 1, 0, 1)
wait(50)
run_task(clawGoTo(OPEN,1,0,450))
wait(10)
RobotSpin(33,5,STANGA,1,0,1,1)
wait(50)
RobotSpin(40,10,DREAPTA,1,0,1,1)
wait(50)
RobotSpin(33,5,STANGA,1,0,1,1)
MoveSyncGyro(-60,8*cm,1,1,1)
RobotCompas(60,90,STANGA,1,1,1,0)
wait(50)
MoveTime(-60,1,0,1)
# gata lasare bolti 3 si 4

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
MoveSyncGyro(-70,12*cm,1,1,1)
run_task(clawGoTo(OPEN,1,0,550))
# gata gard rosu

SMove(-80,80,DREAPTA,0,1,1,0)
wait(200)
MoveSyncGyro(-80,20*cm,1,0,0)
MoveTime(-80,1,0,1)
run_task(liftGoTo(DOWN,0.7,0,500))
wait(100)
MoveSyncGyro(70,8*cm,1,1,1)
RobotCompas(60,88,DREAPTA,1,1,1,0)
wait(100)

# luare nas rosu
LF1SEncoder(50, 10*cm, DREAPTA, 1, 1, 1, 85)
run_task(liftGoTo(UP,1,0,500))
wait(50)
MoveSyncGyro(40,8*cm,1,1,1)
run_task(liftGoTo(20,0.7,0,500))
wait(50)
MoveSyncGyro(-80,10*cm,1,1,1)
run_task(liftGoTo(UP,1,0,500))
wait(50)
MoveSyncGyro(35,5*cm,1,1,1)
run_task(clawGoTo(-70,1,0,500))
wait(50)
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
wait(50)

# lasare nas galben
MoveSyncGyro(80,22*cm,1,1,1)
RobotCompas(70,45,STANGA,0,0,1,0)
wait(50)
MoveSyncGyro(70,10*cm,1,1,1)
wait(100)
run_task(clawGoTo(OPEN,1,0,550))
wait(100)
# gata lasare nas galben

#lasare nas rosu
MoveSyncGyro(-70,15*cm,1,1,1)
RobotCompas(70,75,STANGA,0,0,1,0)
wait(50)
MoveSyncGyro(70,9*cm,1,1,1)
# gata lasare nas rosu

# THE END



print(timer.time()/1000)
wait(20000)