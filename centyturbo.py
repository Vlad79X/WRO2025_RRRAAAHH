from centy_fast import *

print(f"battery voltage : {Brick.battery.voltage()}")
timer = StopWatch()
Init()

cpatrat = 0
switchleftright1,switchleftright2,switchinsideoutside,caz = GetSwitchesAndCase(1,2,4,3)



#   START

# steag alb
run_task(clawGoTo(CLOSED,1,0,400))
MoveSyncGyro(90,10*cm,1,0,0)
LFEncoderSA(90,21*cm,0.5,1.65,3,5,"red",0,1,1)
wait(50)
imux = Brick.imu.rotation(TopAxis) - 90
RobotSpin(80,90,STANGA,1,0,1,1)
wait(100)
run_task(liftGoTo(50,3,0,400))
MoveSyncGyro(90,51*cm,1,1,1)
run_task(liftGoTo(10,3,0,400))
# gata steag alb

wait(50)
run_task(liftGoTo(40,2,2,400))
#wait(100)
mistake = imux - Brick.imu.rotation(TopAxis)
RobotSpin(80,90 + mistake,DREAPTA,1,0,1,1)
wait(100)

# citire caz patrat
while cpatrat == 0:
    cpatrat = ReadCubes([(260,120)])
print("cazul patratului",cpatrat)
# gata citire caz patrat

MoveSyncGyro(90,19*cm,1,1,1)
wait(100)
RobotSpin(80,abs(imux - Brick.imu.rotation(TopAxis))+6,(imux - Brick.imu.rotation(TopAxis))/abs(imux - Brick.imu.rotation(TopAxis)),1,0,1,1)
wait(50) 

# steag rosu
MoveSyncGyro(70,6*cm,1,1,1)
MoveTime(60,0.5,0,1)
run_task(clawGoTo(-20,1,0,400))
#wait(100)
MoveSyncGyro(-70,8*cm,0,0,1)
wait(50)
run_task(clawGoTo(CLOSED,1,0,400))
wait(50)
#MoveSyncGyro(70,1*cm,1,1,1)
print("gata steaguri in ",timer.time()/1000," secunde")
# gata steag rosu

imux = imux + 90
RobotSpin(80,abs(imux - Brick.imu.rotation(TopAxis)),(imux - Brick.imu.rotation(TopAxis))/abs(imux - Brick.imu.rotation(TopAxis)),1,0,1,1)
#wait(100)
imux = imux - 90
MoveSyncGyro(90,6*cm,1,1,1)
wait(100)
RobotSpin(80,90,DREAPTA,1,0,1,1)
wait(100)
MoveSyncGyro(-90,18*cm,1,0,0)
MoveTime(-80,0.5,0,1)
run_task(liftGoTo(-2,1,0,400))
run_task(clawGoTo(0,1,0,400))

# citire caz bolti
MoveSyncGyro(80,9*cm,1,1,1)
#wait(100)
mistake = imux - Brick.imu.rotation(TopAxis) + 180
RobotSpin(80,90,STANGA,1,0,1,1)
MoveSyncGyro(-60,3.5*cm,1,0,0)
MoveTime(-50,0.6,0,1)
#wait(100)
MoveSyncGyro(70,3*cm,1,0,0)
SquaringBlackSA(60,30,4,"red",15*cm,0,0,1)
print(timer.time()/1000)

cub1 = ReadCubes([(100,180)])
cub2 = ReadCubes([(250,180)])
cub3 = ReadCubes([(130,120)])
cub4 = ReadCubes([(180,120)])
for i in range(1,5):
    if(cub1 != i and cub2 != i and cub3 != i):
        cub4 = i
print(cub1,cub2,cub3,cub4)

switchleftright1,switchleftright2,switchinsideoutside,caz = GetSwitchesAndCase(cub1,cub2,cub3,cub4)
RobotSpin(80,90,DREAPTA,1,0,1,1)
#wait(100)
MoveTime(-80,0.8,1,1)
# gata citire bolti

# patrat
if cpatrat == 1:
    # verde
    MoveSyncGyro(90,13*cm,1,0,0)
    ArcMove(90,24*cm,90,DREAPTA,0,1,1)
    mistake = imux - Brick.imu.rotation(TopAxis) + 270
    wait(100)
    RobotSpin(70,mistake,STANGA,1,0,1,1)
    wait(100)
    LFEncoderSA(80,15*cm,0.8,2.5,3,5,"red",0,0,0)
    LFIntersectionSA(80,3,5,35,1,"red",0,0,1)
    #MoveSyncGyro(70,2*cm,0,1,1)
    run_task(clawGoTo(CLOSED,1,0,400))
    #wait(100)
    RobotCompas(80,125,STANGA,0,1,1,20)
    wait(50)
    MoveSyncGyro(-90,3*cm,1,1,1)
    run_task(clawGoTo(OPEN,1,0,400))
    RobotSpin(90,7,STANGA,1,0,1,1)
    wait(50)
    MoveSyncGyro(90,2*cm,1,0,0)
    MSGandCLOSE(90,10*cm,0,1,1,60)
    wait(50)
    RobotCompas(-90,40,STANGA,0,1,1,20)
    #wait(50)
    MoveSyncGyro(-90,13*cm,1,0,0)
    SquaringWhiteSA(-80,40,4,"red",0,0,0)
    MoveSyncGyro(-80,6*cm,0,1,1)
    #wait(100)
    MoveSyncGyro(90,5*cm,1,1,1)
    RobotCompas(90,80,STANGA,1,1,1,20)
    #wait(100)
    run_task(clawGoTo(0,1,0,400))
    #wait(50)
    MoveSyncGyro(70,3*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1,0,400))
    #wait(50)
    MoveSyncGyro(-90,25*cm,1,1,1)
elif cpatrat == 2:
    # rosu
    MoveSyncGyro(90,27*cm,1,0,0)
    #wait(100)
    ArcMove(90,32*cm,90,DREAPTA,0,1,1)
    #wait(100)
    #mistake = imux - Brick.imu.rotation(TopAxis) + 270
    #RobotSpin(50,mistake,DREAPTA,1,0,0,1)
    wait(100)
    MSGandCLOSE(90,13*cm,1,0,0,60)
    MoveTime(80,0.4,0,1)
    #wait(100)
    ArcMove(-90,11.5*cm,180,DREAPTA,0,1,1,25)
    run_task(clawGoTo(OPEN,1,0,400))
    MoveSyncGyro(-70,1*cm,1,1,1)
    wait(50)
    MoveSyncGyro(70,4*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1,0,400))
    MoveSyncGyro(-90,21*cm,1,1,1)
elif cpatrat == 3:
    # albastru
    MoveSyncGyro(90,32*cm,1,0,0)
    ArcMove(90,46.5*cm,90,DREAPTA,0,1,1)
    #wait(100)
    mistake = imux - Brick.imu.rotation(TopAxis) + 270
    RobotSpin(80,90 + mistake,DREAPTA,1,0,1,1)
    wait(50)
    MoveTime(-70,1,1,1)
    MoveSyncGyro(90,10*cm,1,0,0)
    MSGandCLOSE(90,20*cm,0,0,0,50)
    SquaringWhiteSA(80,40,4,"red",0,0,0)
    MoveSyncGyro(80,7*cm,0,1,1)
    wait(50)
    mistake = imux - Brick.imu.rotation(TopAxis) + 360
    RobotCompas(90,85,DREAPTA,0,1,1,20)
    wait(50)
    run_task(clawGoTo(0,1,0,400))
    #wait(50)
    MoveSyncGyro(70,3*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1,0,400))
    MoveSyncGyro(-90,26*cm,1,1,1)
else:
    # galben
    MoveSyncGyro(90,11*cm,1,0,0)
    ArcMove(90,25*cm,90,DREAPTA,0,1,1)
    #wait(100)
    mistake = imux - Brick.imu.rotation(TopAxis) + 270
    RobotSpin(70,mistake,DREAPTA,1,0,1,1)
    wait(50)
    LFEncoderSA(80,10*cm,0.5,2,3,5,"red",0,0,0)
    LFIntersectionSA(80,3,5,30,1,"red",0,0,0)
    MoveSyncGyro(80,2*cm,0,1,1)
    run_task(liftGoTo(20,2,0,400))
    wait(50)
    RobotSpin(90,90,STANGA,1,0,1,1)
    run_task(liftGoTo(-2,2,0,400))
    wait(50)
    MSGandCLOSE(90,12*cm,1,1,1,50)
    wait(50)
    MoveSyncGyro(-90,17*cm,1,0,0)
    SquaringWhiteSA(-80,40,4,"red",0,0,0)
    MoveSyncGyro(-80,3*cm,0,1,1)
    wait(50)
    MoveSyncGyro(90,5*cm,1,1,1)
    RobotCompas(90,83,STANGA,1,1,1,20)
    wait(50)
    run_task(clawGoTo(0,1,0,400))
    #wait(50)
    MoveSyncGyro(70,3*cm,1,1,1)
    run_task(clawGoTo(CLOSED,1,0,400))
    wait(50)
    MoveSyncGyro(-90,27*cm,1,1,1)
#endif

LFEncoderSA(65,22*cm,1.5,6,3,5,"red",1,1,1)
wait(100)
# RobotSpin(90,90,DREAPTA,1,0,1,1,15,12)
# wait(100)
# MoveSyncGyro(90,5*cm,1,1,1)
# wait(100)
# RobotSpin(90,89,STANGA,1,0,1,1,15,12)
# wait(100)
RobotCompas(95,52,DREAPTA,1,1,1,20)
wait(100)
RobotCompas(95,51,STANGA,1,1,1,20)
wait(100)
#SMove(90,55,DREAPTA,0,1,1,20)
#wait(100)
MoveSyncGyro(90,8*cm,1,0,0)
SquaringBlackSA(80,25,3,"red",40*cm,0,0,0)
MoveSyncGyro(70,4*cm,0,1,1)
wait(100)
MoveSyncGyro(-70,5*cm,1,1,1)
run_task(clawGoTo(OPEN,1,0,400))
#wait(100)
print("gata patrat in",timer.time()/1000,"secunde")
# gata patrat

SMove(-90,105,DREAPTA,0,1,1,10)
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
run_task(clawGoTo(SEMIOPEN+50,1,0,400))
MSGandCLOSE(90,28*cm,1,1,1,30)
#wait(100)

# switch-uri
if switchinsideoutside:
    dist = 32
    if switchleftright2:
        SwitchLefttoRight()
        dist = 30
        switchleftright2 = False
    SwitchInsidetoOutside()
    if switchleftright1:
        SwitchLefttoRight()
        dist = 29
else:
    dist = 28       
    if switchleftright1:
        dist = 26
        SwitchLefttoRight()

print("hai la campina", timer.time()/1000)
run_task(clawGoTo(OPEN-70,1,0,400))
#wait(100)
MSGandCLOSE(90,dist*cm,1,1,1,30)
#wait(100)
# gata switch-uri

RobotCompas(90,88,DREAPTA,1,1,1,20)
#wait(200)
SMove(-90,50,STANGA,0,1,1,30)
#wait(200)
MoveSyncGyro(-90,20*cm,1,0,0)
MoveTime(-80,0.5,0,1)
#wait(100)

# lasare bolti 1 si 2
MoveSyncGyro(80,7.8*cm,1,1,1)
wait(100)
RobotSpin(80,90,STANGA,1,0,1,1)
wait(100)
MoveSyncGyro(-50,6*cm,1,1,1)
wait(100)
SquaringBlackSA(50,30,4,"red",10*cm,0,0,0)
MoveSyncGyro(50,2.5*cm,0,1,1)
wait(100)
run_task(clawGoTo(OPEN,1,0,400))
wait(100)
RobotSpin(50,1.5,STANGA,1,0,0,1)
imux = imux - 360
wait(50)
MoveSyncGyro(-80,3*cm,1,1,1)
wait(50)
RobotSpin(90,90,DREAPTA,1,0,1,1)
wait(50)
MoveTime(-80,0.5,0,1)
#wait(50)
print("gata bolti 1 si 2 in", timer.time()/1000, "secunde")
# gata lasare bolti 1 si 2

# gard galben
MSGandCLOSE(90,60*cm,1,0,0,30)
SquaringWhiteSA(80,45,4,"red",0,0,0)
MoveSyncGyro(80,4*cm,0,1,1)
wait(50)
MoveSyncGyro(-80,4*cm,1,1,1)
wait(20)
RobotSpin(80,90,DREAPTA)
imux = Brick.imu.rotation(TopAxis)
#wait(20)
MoveSyncGyro(-90,55*cm,1,1,1)
wait(20)
CompasTime(70,0.8,DREAPTA,1)
wait(50)
RobotCompas(-70,abs(imux - Brick.imu.rotation(TopAxis)),DREAPTA, 0, 1, 1, 20)
imux = imux - 90
wait(20)
print("gata gard galben in", timer.time()/1000, "secunde")
# gata gard galben

# gard rosu
SquaringBlackSA(-80,35,3,"red",35*cm,0,0,1)
wait(50)
RobotCompas(-80,89.5,STANGA,0,1,1,10)
#wait(100)
run_task(liftGoTo(20,2,0,400))
run_task(liftGoTo(5.5,1,0,200))
run_task(clawGoTo(OPEN,1,0,400))
#wait(50)
MoveSyncGyro(70,23.5*cm,1,0,0)
MoveTime(60,0.4,0,1)
#wait(50)
run_task(clawGoTo(CLOSED,1,0,400))   
MoveSyncGyro(-90,11*cm,1,1,1)
run_task(clawGoTo(OPEN,2,0,400))
MoveSyncGyro(-90,20*cm,1,0,0)
MoveTime(-80,0.7,0,1)
# gata gard rosu

# luare nasi
#wait(100)
MoveSyncGyro(90,15*cm,1,1,1)
run_task(liftGoTo(-2,1,0,400))
wait(50)
RobotSpin(90,90,DREAPTA,1,0,1,1)
wait(50)
MoveSyncGyro(-90,8*cm,1,1,1)
wait(50)
LFEncoderSA(70,8*cm,1,2,3,5,"red",1,0,0)
LFIntersectionSA(70,3,5,30,1,"red",0,0,0)
MoveSyncGyro(70,2*cm,0,0,0)
LFEncoderSA(70,10*cm,0.8,2,3,5,"red",0,1,1)
wait(50)
run_task(liftGoTo(UP,1.3,0,400))
#wait(100)
MoveSyncGyro(70,15*cm,1,1,1)
run_task(clawGoTo(-30,1,0,400))
run_task(liftGoTo(DOWN,1.5,0,400))
#wait(50)
MSGandCLOSE(50,4*cm,1,1,1,50)
wait(50)
# gata luare nasi

# lasare nas galben
MoveSyncGyro(-90,22*cm,1,1,1)
RobotSpin(90,90,STANGA)
wait(50)
LFEncoderSA(90,35*cm,1,2,3,5,"red",1,0,0)
MoveSyncGyro(90,11*cm,0,1,1)
RobotCompas(90,130,DREAPTA,0,1,1,25)
wait(50)
MoveSyncGyro(80,4.5*cm,1,1,1)
#wait(50)
run_task(clawGoTo(OPEN,1,0,400))
#wait(50)
# gata lasare nas galben

# lasare nas rosu
MoveSyncGyro(-90,18*cm,1,1,1)
wait(50)
RobotCompas(90,75,STANGA,0,0,1,5)
wait(50)  
#MoveSyncGyro(70,2*cm,1,1,1)
run_task(liftGoTo(30,2,0,400))
MoveSyncGyro(-90,20*cm,1,1,1)
run_task(liftGoTo(DOWN,1.5,0,400))
MoveSyncGyro(70,4*cm,1,0,0)
MSGandCLOSE(60,13*cm,0,1,1,60)
wait(50)
# gata lasare nas rosu

RobotCompas(-90,138,STANGA,0,0,1,5)
MoveSyncGyro(-90,10*cm,1,0,0)
MoveTime(-80,0.5,0,1)
run_task(clawGoTo(OPEN,1,0,400))
wait(50)
MSGandCLOSE(80,18*cm,1,1,1,40)

if switchleftright2:
    SwitchLefttoRight()
#endif
MoveSyncGyro(-90,13*cm,1,0,0)
MoveTime(-80,0.5,0,1)

# lasare bolti 3 si 4
MoveSyncGyro(80,8.5*cm,1,1,1)
mistake = imux - Brick.imu.rotation(TopAxis) - 180
#print(mistake)
wait(100)
RobotSpin(80,90,DREAPTA,1,0,1,1)
wait(100)
MoveSyncGyro(-60,5*cm,1,1,1)
SquaringBlackSA(50,25,4,"red",10*cm,1,0,0)
MoveSyncGyro(60,2*cm,0,1,1)
wait(50)
run_task(clawGoTo(OPEN,1,0,400))
wait(100)
RobotSpin(50,1.5,DREAPTA,1,0,0,1)
# gata lasare bolti 3 si 4

# THE END



print("FINAL",timer.time()/1000)
wait(20000)