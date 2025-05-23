# hub.py

from pupremote import PUPRemoteHub
from pybricks.parameters import Port
from pybricks.tools import wait
from ustruct import calcsize
from umath import ceil

def compute_wait_ms(fps, from_hub_fmt, per_byte_ms=1.5, overhead_ms=10):
    """
    fps:            Pixy frame‐rate in frames per second (e.g. 25)
    from_hub_fmt:   struct format string for the outbound payload (e.g. '2H')
    per_byte_ms:    empirical cost to send one byte over I²C (default 1.5 ms/byte)
    overhead_ms:    extra margin for I²C retries + processing (default 10 ms)

    Returns total wait time in ms, rounded *up* to the nearest multiple of 5.
    """
    # 1) How long between fresh frames:
    frame_period_ms = 1000.0 / fps               # e.g. 25 fps → 40.0 ms

    # 2) How long to send the outbound payload:
    payload_bytes   = calcsize(from_hub_fmt)  # e.g. '2H' → 4 bytes
    payload_time_ms = payload_bytes * per_byte_ms    # e.g. 4 * 1.5 ms = 6.0 ms

    # 3) Sum with overhead:
    total_ms = frame_period_ms + payload_time_ms + overhead_ms  # e.g. 40 + 6 + 10 = 56 ms

    # 4) Round *up* to the next 5 ms:
    return int(ceil(total_ms / 5.0) * 5)

# ——— Example for camCl ———
# Pixy2 default ~25 fps → ~40 ms
# from_hub_fmt='2H'            → 4 bytes payload → ~6 ms
# overhead_ms=10                → ~10 ms
# raw = 40 + 6 + 10 = 56 → rounds up to  60


# ——— Initialize the connection on Port A to the ESP32 ———
pr = PUPRemoteHub(Port.B)

# ——— Register remote commands exposed by ESP32 ———

# Line tracker readings
# Get all 7 relative readings
pr.add_command('lfall', to_hub_fmt='BBBBBBB', from_hub_fmt='')     
# Get one specific sensor
pr.add_command('lfone', to_hub_fmt='B',        from_hub_fmt='B')   
# Get two specific sensors
pr.add_command('lftwo', to_hub_fmt='BB',       from_hub_fmt='BB')  
# Get three specific sensors
pr.add_command('lftre', to_hub_fmt='BBB',      from_hub_fmt='BBB') 

# Line tracker LED color control
# Set all LEDs to a single color by name
pr.add_command('lfacl', from_hub_fmt='repr')   
# Set each LED to a separate color
pr.add_command('lficl', from_hub_fmt='repr')   

# Pixy2 camera
# use only one for now! otherwise will get stuck!
# biggest block for signature
#pr.add_command('camBg', to_hub_fmt='BHHHHB',       from_hub_fmt='B') 
# closest block signature to given (x, y)
pr.add_command('camCl', to_hub_fmt='B',    from_hub_fmt='2H')  

# Servo control
# Move a specific servo to angle
#pr.add_command('servo', to_hub_fmt='',         from_hub_fmt='2B')  


# ——— Initial LED setup: set all to blue at start ———
pr.call('lfacl', 'red')

# ——— Servo motion setup ———
angle = 0  # Start angle
mod = 180    # Will alternate sign to swing angle back and forth

# Estimate wait times: 
# ——— Example for camCl ———
# Huskylens default ~15 fps → ~30 ms
# from_hub_fmt='2H'            → 4 bytes payload → ~6 ms
# overhead_ms=10                → ~10 ms
# raw = 30 + 6 + 10 = 46 → rounds up to  50
WAIT_CL = compute_wait_ms(fps=15, from_hub_fmt='2H')
def ReadCubes(coords:tuple):
    for x, y in coords:
        closest_sig = pr.call('camCl', x, y, wait_ms=WAIT_CL)
        # print(f"Closest block to ({x:3}, {y:3}): sig={closest_sig}")
    return closest_sig



# cub1 = ReadCubes([(320,120)])
# cub2 = ReadCubes([(250,200)])
# cub3 = ReadCubes([(100,100)])
# cub4 = ReadCubes([(180,100)])

# print(cub1)
coords = [(250,180)]
# ——— Main loop for some tests ———
while True:
    # Read all 7 line tracker sensors and print them
    s1, s2, s3, s4, s5, s6, s7 = pr.call('lfall')
    print(pr.call('lfall'))

    for x, y in coords:
        closest_sig = pr.call('camCl', x, y, wait_ms=WAIT_CL)
        print(f"Closest block to ({x:3}, {y:3}): sig={closest_sig}")
        wait(50)
    
wait(500)