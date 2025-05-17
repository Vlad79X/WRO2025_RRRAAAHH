from machine import Pin
from machine import SoftI2C
from pupremote import PUPRemoteSensor
from pyhuskylens import HuskyLens, ALGORITHM_COLOR_RECOGNITION
from servo import Servo
import time

# Set up Huskylens
# Ensure Huskylens is in i2c mode via General Settings > Protocol Type
time.sleep(2) # Wait for the Huskylens to boot
i2c = SoftI2C(scl=Pin(20), sda=Pin(19)) # use only these pins for HuskyLens
huskylens = HuskyLens(i2c)

print("Huskylens connected is", huskylens.knock())
huskylens.set_alg(ALGORITHM_COLOR_RECOGNITION) # set mode for camera
huskylens.show_text("Connected LMS-ESP32 !")

def camBl(q_id):
    blocks = huskylens.get_blocks(ID = q_id)
    
    if not blocks:
        return [0] * 20
    
    i = 0
    final_blocks = []
    
    while i < 4 and i < len(blocks):
        b = blocks[i]
        final_blocks.append(b.x,b.y,b.width,b.height,b.ID)
        i += 1
        
    return final_blocks

# Set up comms with SPIKE hub
pr = PUPRemoteSensor(power=True)

# from_hub_fmt = gets paramter from PyBricks
# to_hub_fmt   = sends paramter from PyBricks

# command to get data from Huskylens based in ID of color
# needs ID of color and returns 5 bytes of data
pr.add_command('camBl',to_hub_fmt='20b',from_hub_fmt='b')

# must a forever loop to work
while True:
    pr.process()

