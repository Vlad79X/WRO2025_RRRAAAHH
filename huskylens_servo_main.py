from machine import Pin
from machine import SoftI2C
from pupremote import PUPRemoteSensor
from pyhuskylens import HuskyLens, ALGORITHM_COLOR_RECOGNITION
import time
from servo import Servo
from sys import getsizeof

# function to get the block based on ID
# returns 5 parametes, as seen
def camBl(query_id):
    blocks = huskylens.get_blocks(ID=query_id)
    if blocks:
        b = blocks[0]
        return (b.x, b.y, b.width, b.height, b.ID)
    # not seen → zeros
    return (0, 0, 0, 0, 0)

def camVl(q_id):
    for cur_id in range(1,5):
        blocks.append(huskylens.get_blocks(cur_id))
    
    while(getsizeof(blocks) < 50):
        blocks += 'x'
    
    return blocks

# function to move specific servo, based on server_pin and angle
def servo(servo_pin, angle):
    if servo_pin == 21:
        servo21.angle( angle )
        # print("Servo moved by ", angle) # for debugging purposes
    else:
        servo22.angle( angle )
        # print("Servo moved by ", angle) # for debugging purposes

# Set up comms with SPIKE hub
pr = PUPRemoteSensor(power=True)

# from_hub_fmt = gets paramter from PyBricks
# to_hub_fmt   = sends paramter from PyBricks

# command to get data from Huskylens based in ID of color
# needs ID of color and returns 5 bytes of data
pr.add_command('camBl',to_hub_fmt='5b',from_hub_fmt='b')

pr.add_command('camVl',to_hub_fmt='50b',from_hub_fmt='b')

# command to move the specific servo
# needs 2 parameters
pr.add_command('servo',from_hub_fmt='2b')

pr.process() # Connect to hub

# Set up Huskylens
# Ensure Huskylens is in i2c mode via General Settings > Protocol Type
time.sleep(4) # Wait for the Huskylens to boot
i2c = SoftI2C(scl=Pin(20), sda=Pin(19)) # use only these pins for HuskyLens
huskylens = HuskyLens(i2c)
print("Huskylens connected is", huskylens.knock())
huskylens.set_alg(ALGORITHM_COLOR_RECOGNITION) # set mode for camera
huskylens.show_text("Connected LMS-ESP32 !")

# setting up the servos
# min_pulse and max_pulse will change so that the servo will spin all the range
servo21 = Servo(21, min_pulse=400, max_pulse=2700, min_angle=-90, max_angle=90)
servo22 = Servo(22, min_pulse=400, max_pulse=2700, min_angle=-90, max_angle=90)

# must a forever loop to work
while True:
    pr.process()
