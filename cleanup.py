from pybricks.pupdevices import Motor, ColorDistanceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

def is_similar(block1, block2, threshold=5):
    """
    Manually check if two blocks are similar by comparing their coordinates (x, y) and size (width, height).
    :param block1: The first block tuple (x, y, width, height, id)
    :param block2: The second block tuple (x, y, width, height, id)
    :param threshold: The threshold below which blocks are considered similar (in terms of coordinates and size).
    :return: True if blocks are considered similar, False otherwise.
    """
    x1, y1, w1, h1, id1 = block1
    x2, y2, w2, h2, id2 = block2
    
    # Check if the difference in position and size is below the threshold
    return (
        abs(x1 - x2) <= threshold and
        abs(y1 - y2) <= threshold and
        abs(w1 - w2) <= threshold and
        abs(h1 - h2) <= threshold
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

# Example usage:
block = [(0, 0, 0, 0, 0), (74, 97, 49, 13, 2), (0, 0, 0, 0, 0), (0, 0, 0, 0, 0),
         (70, 98, 45, 14, 2), (-2, -98, 49, 36, 3), (-2, -98, 49, 36, 3), (27, 95, 19, 26, 4),
         (0, 0, 0, 0, 0), (0, 0, 0, 0, 0)]

cleaned_blocks = clean_blocks(block)

print("Cleaned blocks:")
print(cleaned_blocks)
