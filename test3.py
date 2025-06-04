from centy_Library import *

print(f"battery voltage : {Brick.battery.voltage()}")
timer = StopWatch()
Init()

cpatrat = 0
switchleftright1,switchleftright2,switchinsideoutside,caz = GetSwitchesAndCase(1,2,4,3)



#   START

# steag alb
run_task(clawGoTo(CLOSED,1,0,500))
MoveSyncGyro(90,10*cm,1,0,0)
LFEncoderSA(90,23.7*cm,0.5,1.65,3,5,"red",0,1,1)
wait(50)
imux = Brick.imu.rotation(TopAxis) - 90

run_task(clawGoTo(OPEN,1,0,500))
wait(10000)

cpatrat = 3

if cpatrat == 1:
    # verde
    MoveSyncGyro(90,10*cm,1,0,0)
    ArcMove(90,24*cm,90,DREAPTA,0,1,1)
    wait(100)
    mistake = imux - Brick.imu.rotation(TopAxis) + 270
    LFEncoderSA(70,15*cm,0.8,3,3,5,"red",1,0,0)
    LFIntersectionSA(70,3,5,30,1,"red",0,0,1)
    #MoveSyncGyro(70,2*cm,0,1,1)
    run_task(liftGoTo(7,2,2,300))
    #wait(100)
    RobotCompas(90,125,STANGA,0,1,1,20)
    wait(50)
    MoveSyncGyro(-90,7*cm,1,1,1)
    #wait(50)
    run_task(liftGoTo(-2,1,0,500))
    #wait(50)
    MoveSyncGyro(80,3*cm,1,0,0)
    MSGandCLOSE(70,12*cm,0,1,1,50)
    wait(50)
    RobotCompas(-90,40,STANGA,0,1,1,20)
    #wait(50)
    MoveSyncGyro(-90,10*cm,1,0,0)
    SquaringWhiteSA(-80,40,4,"red",0,0,0)
    MoveSyncGyro(-80,5*cm,0,1,1)
    #wait(100)
    MoveSyncGyro(90,5*cm,1,1,1)
    RobotCompas(90,85,STANGA,1,1,1,20)
    #wait(100)
    run_task(clawGoTo(0,1,0,400))
    #wait(50)
    MoveSyncGyro(70,3*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1,0,400))
    #wait(50)
    MoveSyncGyro(-90,27*cm,1,1,1)
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
    MoveSyncGyro(-80,17*cm,1,1,1)
elif cpatrat == 3:
    # albastru
    MoveSyncGyro(90,34*cm,1,0,0)
    ArcMove(90,48*cm,90,DREAPTA,0,1,1)
    wait(100)
    mistake = imux - Brick.imu.rotation(TopAxis) + 270
    RobotSpin(80,90,DREAPTA,1,0,1,1)
    wait(50)
    MoveTime(-70,1,1,1)
    MoveSyncGyro(70,10*cm,1,0,0)
    MSGandCLOSE(70,20*cm,0,0,0,40)
    SquaringWhiteSA(70,40,4,"red",0,0,0)
    MoveSyncGyro(70,7*cm,0,1,1)
    wait(100)
    mistake = imux - Brick.imu.rotation(TopAxis) + 360
    RobotCompas(80,85,DREAPTA,0,1,1,20)
    wait(200)
    run_task(clawGoTo(0,1,0,400))
    wait(50)
    MoveSyncGyro(60,3*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1,0,400))
    MoveSyncGyro(-70,26*cm,1,1,1)
else:
    # galben
    MoveSyncGyro(90,10*cm,1,0,0)
    ArcMove(90,23*cm,90,DREAPTA,0,0,0)
    wait(100)
    mistake = imux - Brick.imu.rotation(TopAxis) + 270
    #RobotSpin(70,mistake,DREAPTA,1,0,1,1)
    wait(100)
    LFEncoderSA(70,15*cm,0.5,2,3,5,"red",0,0,0)
    LFIntersectionSA(70,3,5,30,1,"red",0,0,0)
    MoveSyncGyro(70,2*cm,0,1,1)
    wait(100)
    RobotSpin(80,90,STANGA,1,0,1,1)
    MSGandCLOSE(60,12*cm,1,1,1,40)
    wait(100)
    MoveSyncGyro(-80,17*cm,1,0,0)
    SquaringWhiteSA(-70,40,4,"red",0,0,0)
    MoveSyncGyro(-60,3*cm,0,1,1)
    wait(100)
    MoveSyncGyro(70,5*cm,1,1,1)
    RobotCompas(80,83,STANGA,1,1,1,20)
    wait(100)
    run_task(clawGoTo(0,1,0,400))
    wait(50)
    MoveSyncGyro(60,3*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1,0,400))
    wait(50)
    MoveSyncGyro(-80,27*cm,1,1,1)
#endif

LFEncoderSA(65,22*cm,2,6,3,5,"red",1,1,1)
wait(100)
SMove(95,53,DREAPTA,0,1,1,25)
wait(100)
run_task(clawGoTo(OPEN,1,0,400))
RobotSpin(90,9,STANGA,0,0,1,1)
wait(100)
mistake = 180 - abs(((imux*100)%18000)/100 - ((Brick.imu.rotation(TopAxis) + 90)*100%18000)/100)
RobotSpin(90,mistake,DREAPTA,1,0,1,1)
wait(100)
MSGandCLOSE(90,20*cm,1,0,0,40)
SquaringBlackSA(80,30,3,"red",40*cm,0,0,0)
MoveSyncGyro(70,5*cm,0,1,1)
#wait(100)
MoveSyncGyro(-70,4*cm,1,1,1)
run_task(clawGoTo(OPEN,1,0,500))
#wait(100)
print("gata patrat in",timer.time()/1000,"secunde")
# gata patrat

SMove(-90,60,DREAPTA,0,1,1,10)
#wait(100)
MoveSyncGyro(-90,50*cm,0,0,0)
MoveTime(-80,0.5,0,1)
#wait(100)
print("acum cazuri", timer.time()/1000)

# luare bolti
imux = imux + 360
mistake = imux - Brick.imu.rotation(TopAxis)
Cazuri(caz,mistake,imux)
# gata luare bolti
print("am luat bolti", timer.time()/1000)
#wait(100)
run_task(clawGoTo(SEMIOPEN,1,0,500))
MSGandCLOSE(90,28*cm,1,1,1,20)
#wait(100)

# switch-uri
if switchinsideoutside:
    dist = 30
    if switchleftright2:
        SwitchLefttoRight()
        dist = 30
        switchleftright2 = False
    SwitchInsidetoOutside()
    if switchleftright1:
        SwitchLefttoRight()
        dist = 30
else:
    dist = 30
    if switchleftright1:
        dist = 20
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
MoveSyncGyro(80,8.4*cm,1,1,1)
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

# THE END



print("FINAL",timer.time()/1000)
wait(20000)