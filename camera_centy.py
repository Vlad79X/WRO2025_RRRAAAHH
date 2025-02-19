import sensor
import time
from machine import LED
import image

sensor.reset()  # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)  # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.HQVGA)  # Set frame size to QVGA (320x240)
sensor.set_auto_exposure(False,exposure_us = 8000)
sensor.set_framerate(70)
sensor.set_brightness(-3)
sensor.set_hmirror(False)
sensor.set_saturation(3)
sensor.skip_frames(time=500)  # Wait for settings take effect.
clock = time.clock()  # Create a clock object to track the FPS.

ledr = LED("LED_RED")
ledg = LED("LED_GREEN")
ledb = LED("LED_BLUE")

colors = [[255,0,0],[255,255,0],[0,0,255]]
k = -1
timer = 0.0
xradiusofsensor = 25
yradiusofsensor = 15
xradiusofblob = 120
yradiusofblob = 60

xmid = int(sensor.width()/2)
ymid = int(sensor.height()/2)
roi = [int(xmid - xradiusofblob / 2), int(ymid - yradiusofblob / 2), xradiusofblob, yradiusofblob]
colorrgb = (0,0,0)

s1 = s2 = 0

def patrioti_adevarati():
    global timer,k
    if (timer>=0.5):
        timer = 0
        k = (k+1)%3
        ledr.off()
        ledg.off()
        ledb.off()
        if(colors[k][0] == 255):
            ledr.on()
        if(colors[k][1] == 255):
            ledg.on()
        if(colors[k][2] == 255):
            ledb.on()

def linefollower(imgg):
    global s1,s2
    imgg = imgg.to_grayscale()
    imgg = imgg.gamma(gamma = 0.001, contrast = 5, brightness = -2)

    x1 = int(imgg.width()/2-45+xradiusofsensor)
    x2 = int(imgg.width()/2+45-xradiusofsensor)
    y1 = y2 = int(imgg.height()-30)

    imgg.draw_rectangle(x1,y1,xradiusofsensor,yradiusofsensor,[255,0,0],1)
    imgg.draw_rectangle(x2,y2,xradiusofsensor,yradiusofsensor,[255,0,0],1)

    s1sum = s2sum = 0

    for x in range(0,xradiusofsensor):
        for y in range(0,yradiusofsensor):
            s1sum += imgg.get_pixel(x1 + x,y1 + y)/255
            s2sum += imgg.get_pixel(x2 + x,y2 + y)/255

    s1 = int(100 * s1sum / (xradiusofsensor * yradiusofsensor))
    s2 = int(100 * s2sum / (xradiusofsensor * yradiusofsensor))

    print(f"sensors : {s1},{s2}")

def color(imgg):
    global colorrgb
    imgg = imgg.gamma(gamma = 1, contrast = 1.3, brightness = -0.1)
    imgg = imgg.lens_corr(zoom = 1.7)

    imgg.draw_rectangle(int(xmid-xradiusofblob/2),int(ymid-yradiusofblob/2),xradiusofblob,yradiusofblob,color = [255,0,0])

    hist = imgg.get_histogram([(0,80,-100,100,-100,100)],roi=roi).get_statistics()

    lab = (hist.l_median(),hist.a_median(),hist.b_median())
    colorrgb = image.lab_to_rgb(lab)

    print(colorrgb)

while True:
    clock.tick()

    img = sensor.snapshot()
    img.lens_corr(1.4)

    lfimg = img.copy()
    linefollower(lfimg)

    clrimg = img.copy()
    color(clrimg)

    timer += 1.0/(clock.fps())
    patrioti_adevarati()

    # print(f"FPS : {clock.fps()}")
