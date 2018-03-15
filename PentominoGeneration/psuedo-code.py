# Import related libraries
from __future__ import print_function
import random

# Global Variables
max_x = 20
max_y = 10
num_poly = 7

# Class representing a given polyomino configuration
class Polyomino:

    # Constructor -- pass in number of blocks and list of block coordinates
    def __init__(self, num_blocks):
        self.num_blocks = num_blocks
        self.array = [[0 for i in range(2*num_blocks-1)] for j in range(2*num_blocks-1)]
        self.array[num_blocks-1][num_blocks-1] = 1
        
# Initialize polyomino
poly = Polyomino(num_poly)
for row in range(2*num_poly-1):
    print(poly.array[row])



#Function takes in 2-D coordinate array and returns list of available spaces around it
    # For each coordinate in shape
        # add each surrounding coordinate to list
        # Check for existing coordinates
            # Do not add multiples
    # Add all potential spaces for each existing coordinate,
        # Remove existing coordinates

# max_x = 10
# max_y = 12
# coor_array = []

# available_spaces():
#     out_spaces = [['0' for x in range(max_x)] for y in range(max_y)]
#     for i in range(0, x_max):
#         for j in range(0, y_max):
#             if coor_array[i[j]] == 1:

#                 out_spaces[i[j]] = 1




'''
functionName(arg1, arg2, arg3)
arg1: integer
arg2: array
arg3:
output:
Description: function takes in a number of arguments equal to arg1
'''
def functionName(arg1, arg2, arg3):
    return







