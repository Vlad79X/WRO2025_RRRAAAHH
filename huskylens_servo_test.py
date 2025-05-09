from pupremote import PUPRemoteHub
from pybricks.parameters import Port, Direction
from pybricks.robotics import DriveBase
from pybricks.pupdevices import Motor
from pybricks.tools import wait
import usys

usys.stdout.write("\x1b[2J\x1b[H")

# set port for the ESP32 NOT E NOT F
pr = PUPRemoteHub(Port.B)

# gets the blocks of the ID given
# needs 1 parameter, the ID of the color
# receives 5 parameters: X, Y, width, height and ID
pr.add_command('camBl',to_hub_fmt='5b',from_hub_fmt='b')
pr.add_command('camVl',to_hub_fmt='50b',from_hub_fmt='b')

# moves the servo
# sends 2 paramters: servo pin number, degrees

def is_similar(block1, block2, threshold=0.9):
    """
    Check if two blocks are similar by comparing their coordinates (x, y) and size (width, height)
    using a ratio-based comparison.
    :param block1: The first block tuple (x, y, width, height, id)
    :param block2: The second block tuple (x, y, width, height, id)
    :param threshold: The minimum ratio threshold for blocks to be considered similar.
    :return: True if blocks are considered similar, False otherwise.
    """

    x1, y1, w1, h1, id1 = block1
    x2, y2, w2, h2, id2 = block2
    
    # Check if the ratio of corresponding values is above the threshold (>= 0.9)

    return (
        (min(x1, x2) / max(x1, x2) >= threshold if x2 != 0 else False) and
        (min(y1, y2) / max(y1, y2) >= threshold if y2 != 0 else False) and
        (min(w1, w2) / max(w1, w2) >= threshold if w2 != 0 else False) and
        (min(h1, h2) / max(h1, h2) >= threshold if h2 != 0 else False)
    )


def clean_blocks(blocks):
    """
    Clean the blocks by removing duplicates and invalid blocks (like (0, 0, 0, 0, 0)).
    Only removes duplicates for blocks that share the same id.
    :param blocks: List of block tuples (x, y, width, height, id).
    :return: A cleaned list of blocks with duplicates removed.
    """

    cleaned_blocks = []

    # Iterate through the blocks and remove (0, 0, 0, 0, 0)

    for block in blocks:
        if block == (0, 0, 0, 0, 0):
            continue  # Skip invalid blocks
        
        # Check if the block has the same id as any already added block
        # If it does, compare the coordinates and size

        found_similar = False
        for existing_block in cleaned_blocks:
            # If blocks have the same id and are similar in terms of coordinates and size

            if block[4] == existing_block[4] and is_similar(block, existing_block):
                found_similar = True
                break
        
        # If no similar block was found, add this block to the cleaned list

        if not found_similar:
            cleaned_blocks.append(block)
    
    return cleaned_blocks

lenn = 10

def read_blocks(verbose):
    block = pr.call('camVl',7979)

    cleaned_blocks = clean_blocks(block)

    if verbose == 1 or verbose == True:
        print(block)
        print()    
        print(cleaned_blocks)

def read_block2():
    pr.call()

while 1==1:
    usys.stdout.write("\x1b[2J\x1b[H")
    read_blocks(True)
    wait(5000)

def which_is_where():
    print(f"pizda matii")