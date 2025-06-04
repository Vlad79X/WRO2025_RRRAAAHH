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



run_task(clawGoTo(CLOSED,1,0,500))
wait(100)
LFEncoderSA(60,50*cm,2,5,3,5,"red",1,1,1)


wait(100000)
def Cazuri(caz: str,mistake:float, imux: float):
    if caz == "drept":
        MoveSyncGyro(80,3*cm,1,1,1)
        wait(100)
        RobotCompas(70, 88, STANGA, 0, 1, 1, 0)
        wait(100)
        LFEncoderSA(50,8*cm,3,5,"red",1,0,0)
        SquaringWhiteSA(50,30,4,"blue",0,0,1)
        wait(100)
        run_task(clawGoTo(CLOSED,1,0,500))
        MoveSyncGyro(-70,9*cm,1,1,1)
        wait(100)
        run_task(
            multitask(
                clawGoTo(OPEN,1,0,450),
                liftGoTo(UP,10,5,500)
            )
        )
        wait(100)
        MoveSyncGyro(50,11*cm,0,1,1)
        wait(100)
        run_task(
            multitask(
                liftGoTo(DOWN,1.5,0,500),
            )
        )
        wait(100)
        MoveSyncGyro(50,6.5*cm,0,0,1)
        wait(100)
        run_task(clawGoTo(CLOSED,1,0,500))
        wait(100)
        MoveSyncGyro(-60,15*cm,0,0,1)
        wait(100)
        RobotCompas(-80,88,STANGA,0,1,1,0)
        wait(100)
        MoveTime(-70,0.5,0,1)
    elif caz == "lateral":
        MoveSyncGyro(60,27*cm,1,1,1)
        wait(100)
        RobotSpin(80,90-mistake,DREAPTA,1,0,1,1)
        wait(100)
        MoveSyncGyro(-70,40*cm,1,0,0)
        MoveTime(-70,1,0,1)
        run_task(liftGoTo(UP,1,0,550))
        wait(100)
        MoveSyncGyro(60,8*cm,1,1,1)
        wait(100)
        RobotSpin(80,88,DREAPTA,1,0,1,1)
        wait(100)
        MoveSyncGyro(-50,3*cm,1,0,0)
        MoveTime(-40,0.5,0,1)
        run_task(liftGoTo(DOWN,1.5,0,500))
        MoveSyncGyro(60,6*cm,1,1,1)
        run_task(clawGoTo(CLOSED,1,0,500))
        wait(100)
        MoveSyncGyro(-50,3*cm,1,0,0)
        MoveTime(-40,0.2,0,1)
        run_task(clawGoTo(OPEN,1,0,500))
        run_task(liftGoTo(UP,1,0,550))
        wait(100)
        MoveSyncGyro(60,10*cm,1,1,1)
        run_task(liftGoTo(DOWN,1.5,0,500))
        wait(100)
        MoveSyncGyro(60,5*cm,1,1,1)
        run_task(clawGoTo(CLOSED,1,0,500))
        wait(100)
        MoveSyncGyro(-70,15*cm,1,1,1)
        ArcMove(70,15*cm,90,STANGA,0,0,1)
        wait(100)
        MoveSyncGyro(70,30*cm,1,1,1)
        wait(100)
        RobotSpin(80,90,STANGA,1,0,1,1)
        wait(100)
        MoveTime(-70,1,0,1)
    elif caz == "diagonal":
        run_task(clawGoTo(-70,1,0,500))
        # run_task(liftGoTo(UP,2.5,0,550))
        MoveSyncGyro(80,17*cm,1,1,1)
        wait(100)
        RobotSpin(70,90,STANGA,1,1,1,2)
        wait(200)
        MoveSyncGyro(80,24*cm,1,1,1)
        wait(100)
        run_task(clawGoTo(CLOSED,1,0,500))
        wait(100)
        MoveSyncGyro(-80,12*cm,1,1,1)
        run_task(clawGoTo(-70,1,0,500))
        run_task(liftGoTo(UP,2.5,0,550))
        MoveSyncGyro(80,10*cm,1,1,1)
        run_task(liftGoTo(DOWN,2.5,0,550))
        MoveSyncGyro(80,7*cm,1,1,1)
        run_task(clawGoTo(CLOSED,1,0,500))
        wait(100)
        MoveSyncGyro(-90,13*cm,1,1,1)
        wait(100)
        run_task(clawGoTo(OPEN,1,0,500))
        MoveSyncGyro(-90,11*cm,1,1,1)
        wait(100)
        RobotSpin(80,17,STANGA,1,0,1,7)
        wait(100)
        MoveSyncGyro(60,24*cm,1,1,1)
        run_task(clawGoTo(CLOSED,0.6,0,600))
        run_task(liftGoTo(UP,2.5,0,550))
        MoveSyncGyro(-90,23*cm,1,1,1)
        wait(100)
        run_task(liftGoTo(DOWN,2.5,0,550))
        run_task(clawGoTo(OPEN,1,0,500))
        run_task(liftGoTo(UP,2.5,0,550))
        MoveSyncGyro(70,10*cm,1,1,1)
        run_task(liftGoTo(DOWN,2.5,0,550))
        MoveSyncGyro(70,3*cm,1,1,1)
        RobotSpin(65,10,STANGA,1,0,1,8)
        wait(100)
        MoveSyncGyro(60,12*cm,1,1,1)
        run_task(clawGoTo(CLOSED,1,0,500))
    #
#


#rosu
#print(Sensor.reflection())

Cazuri("diagonal",0,imux)