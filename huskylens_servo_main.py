from machine import Pin
from machine import SoftI2C
from pupremote import PUPRemoteSensor
from pyhuskylens import HuskyLens, ALGORITHM_COLOR_RECOGNITION
from servo import Servo
import time

def camBl(q_id):
    for cur_id in range(1,5):
        blocks.append(huskylens.get_blocks(cur_id))
    
    return blocks

# Set up comms with SPIKE hub
pr = PUPRemoteSensor(power=True)

# from_hub_fmt = gets paramter from PyBricks
# to_hub_fmt   = sends paramter from PyBricks

# command to get data from Huskylens based in ID of color
# needs ID of color and returns 5 bytes of data
pr.add_command('camBl',to_hub_fmt='80b',from_hub_fmt='')

pr.process() # Connect to hub

# Set up Huskylens
# Ensure Huskylens is in i2c mode via General Settings > Protocol Type
time.sleep(4) # Wait for the Huskylens to boot
i2c = SoftI2C(scl=Pin(20), sda=Pin(19)) # use only these pins for HuskyLens
huskylens = HuskyLens(i2c)
print("Huskylens connected is", huskylens.knock())
huskylens.set_alg(ALGORITHM_COLOR_RECOGNITION) # set mode for camera
huskylens.show_text("Connected LMS-ESP32 !")

# must a forever loop to work
while True:
    pr.process()
