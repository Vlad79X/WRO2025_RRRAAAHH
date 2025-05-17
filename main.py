from machine import Pin
from machine import SoftI2C
from pupremote import PUPRemoteSensor
from pyhuskylens import HuskyLens, ALGORITHM_COLOR_RECOGNITION
from servo import Servo
import time

# Set up Huskylens
# Ensure Huskylens is in i2c mode via General Settings > Protocol Type
time.sleep(400) # Wait for the Huskylens to boot
i2c = SoftI2C(scl=Pin(20), sda=Pin(19)) # use only these pins for HuskyLens
huskylens = HuskyLens(i2c)

print("Huskylens connected is", huskylens.knock())
huskylens.set_alg(ALGORITHM_COLOR_RECOGNITION) # set mode for camera
huskylens.show_text("Connected LMS-ESP32 !")

def camBl(q_id):
    try:
        blocks = []
        for blk in huskylens.get_blocks():
            blocks.extend([
                blk.x & 0xFF,
                blk.y & 0xFF,
                blk.width & 0xFF,
                blk.height & 0xFF,
                blk.ID & 0xFF,
            ])
        blocks = (blocks + [1] * 80)[:80]
        return blocks
    except Exception as e:
        print("camBl error:", e)
        return [0]*80

# Set up comms with SPIKE hub
pr = PUPRemoteSensor(power=True)

# from_hub_fmt = gets paramter from PyBricks
# to_hub_fmt   = sends paramter from PyBricks

# command to get data from Huskylens based in ID of color
# needs ID of color and returns 5 bytes of data
pr.add_command('camBl',to_hub_fmt='80b',from_hub_fmt='b')

pr.process() # Connect to hub

# must a forever loop to work
while True:
    pr.process()
