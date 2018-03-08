# Import related libraries
from __future__ import print_function
import random

# Global Variables
max_x = 20
max_y = 20
num_poly = 7


# Class representing a given polyamino configuration
class Polyamino:

    # Constructor -- pass in number of blocks and list of block coordinates
    def __init__(self, num_blocks, coors):
        self.num_blocks = num_blocks
        self.coors = coors

    def __init__(self, num_blocks):
        self.num_blocks = num_blocks
        # Generate randomly selected coordinates
        coors = []
        for block in range(num_blocks):
            coors.append((random.randint(0, max_x - 1), random.randint(0, max_y - 1)))
        self.coors = coors


# MAIN FUNCTION

# Initialize polyamino
poly = Polyamino(num_poly)

# Initialize output string
out_string = [[' ' for x in range(max_x)] for y in range(max_y)]

# Populate output string
for coor in poly.coors:
    out_string[coor[1]][coor[0]] = 'X'

# Print out randomly selected blocks
for row in range(max_y):
    for elem in range(max_x):
        print(out_string[row][elem], end='')
    print('')




'''

for each coordinate in shape
    add each surrounding coordinate to 2D array

for each coordinate in list
    check for multiples
    check for already existing coordinates
'''