# PolyominoGenerator.py (for use with Python 3)
#
# To run, use "python PolyominoGenerator.py n" where n is replaced
# with the integer of the expected number of units in the polyomino.
#
# This program uses a lexicographic ordering to ensure the same 
# configuration is only printed once (though symmetries and transformations
# of configurations do appear). The program uses a recursive function that 
# adds positive integers as "used" units, and uses negative integers to
# enumerate locations for future units to be placed. Any negative integers
# with magnitudes greater than the largest positive integer are changed to
# -1 to signify that this location cannot be used for future units, though
# in the textbook, these numbers are circled instead (this is the basis for 
# the lexicographic ordering).

# For example:
# Following the example from pg. 231 of the geometry textbook used to build
# the program, let's begin with a matrix like this:

# | 0 | 0 | 0 | 0 | 0 | 0 |
# | 0 | 0 | 0 | 0 | 0 | 0 |
# | 0 | 0 |-3 | 0 | 0 | 0 |
# | 0 | 0 | 1 |-2 | 0 | 0 |
# | 0 | 0 | 0 | 0 | 0 | 0 |

# The positive "1" represents the only unit currently in the configuration.
# The "-2" and "-3" represent possible future placements. Here are the next level
# configurations:

# | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |
# | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 |-6 | 0 | 0 | 0 |
# | 0 | 0 |-3 |-5 | 0 | 0 |    | 0 |-4 | 3 |-5 | 0 | 0 |
# | 0 | 0 | 1 | 2 |-4 | 0 |    | 0 | 0 | 1 |-1 | 0 | 0 |
# | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |

# In the left matrix, the unit of "-2" was inverted to add it to the configuration,
# and then two more potential placements were added for the next branch. In the right,
# "3" was added to the configuration instead, but since this meant the largest number
# of the configuration is larger than the magnitude of "-2", the "-2" location was
# changed to "-1" to signify it cannot be a placement for future units (as this case is
# taken care of with further branches of the left configuration). The daughter branches
# of the left matrix are:

# | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |
# | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |
# | 0 | 0 |-7 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 |-7 | 0 | 0 |
# | 0 |-6 | 3 |-5 | 0 | 0 |    | 0 | 0 |-1 |-5 |-7 | 0 |    | 0 | 0 |-1 | 5 |-6 | 0 |
# | 0 | 0 | 1 | 2 |-4 | 0 |    | 0 | 0 | 1 | 2 | 4 |-6 |    | 0 | 0 | 1 | 2 |-1 | 0 |
# | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |

# The daughter branches of the right matrix are:

# | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |
# | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 |-9 | 0 | 0 | 0 |
# | 0 |-8 |-6 | 0 | 0 | 0 |    | 0 | 0 |-6 |-8 | 0 | 0 |    | 0 |-7 | 6 |-8 | 0 | 0 |
# |-7 | 4 | 3 |-5 | 0 | 0 |    | 0 |-1 | 3 | 5 |-7 | 0 |    | 0 |-1 | 3 |-1 | 0 | 0 |
# | 0 | 0 | 1 |-1 | 0 | 0 |    | 0 | 0 | 1 |-1 | 0 | 0 |    | 0 | 0 | 1 |-1 | 0 | 0 |
# | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |


# TODO:
# 0. Follow the extra placement constraints from the textbook (i.e. none above the row
#    of the first block, nor to the left of the block).
# 1. Minimize matrices before printing to remove unnecessary spaces.
# 2. Use paths around the configuration to get a unique label for each shape. This
#    should be the basis for the lexicographic ordering, and it also naturally removes
#    rotational symmetries (and makes other symmetries easy to remove).
# 3. Apply to 6- and 8-sided shapes as well.

# Importing modules
from __future__ import print_function
import sys

# Declare Polyomino class
class Polyomino:

    # Constructor function (copy constructor when a copy is used)
    # num_blocks - Number of blocks in final configuration
    # copy - Polyomino used as copy for self
    def __init__(self, num_blocks, copy=None):

        if copy is None:
            self.num_blocks = num_blocks    # Number of blocks in the final configuration
            self.maxInt = 1                 # Largest integer in the configuration
            self.numInts = 1                # Total number of active or "used" integers
            self.size = 1                   # Number of blocks currently placed
            self.dim = 2*num_blocks-1       # Matrix dimensions

            # Clear matrix array and place first block in the center
            self.array = [[0 for i in range(self.dim)] for j in range(self.dim)]
            self.array[num_blocks-1][num_blocks-1] = 1

        else:
            self.num_blocks = copy.num_blocks
            self.maxInt = copy.maxInt
            self.numInts = copy.numInts
            self.size = copy.size
            self.dim = copy.dim

            # Initalize matrix array and use copy
            self.array = [[0 for i in range(self.dim)] for j in range(self.dim)]
            for i in range(self.dim):
                for j in range(self.dim):
                    self.array[i][j] = copy.array[i][j]

    # refreshSpaces() runs through the matrix and does the following:
    #   - replaces zeroes next to positive numbers with negative integers
    #   - for negative numbers whose magnitude is greater than the largest
    #       number in the current arrangement, replaces with -1
    def refreshSpaces(self):
        negInt = -self.numInts-1
        for i in range(self.dim):
            for j in range(self.dim):
                if self.array[i][j] > 0:
                    if i > 0:
                        if self.array[i-1][j] == 0:
                            self.array[i-1][j] = negInt
                            negInt -= 1
                            self.numInts += 1
                    if i < self.dim-1:
                        if self.array[i+1][j] == 0:
                            self.array[i+1][j] = negInt
                            negInt -= 1
                            self.numInts += 1
                    if j > 0:
                        if self.array[i][j-1] == 0:
                            self.array[i][j-1] = negInt
                            negInt -= 1
                            self.numInts += 1
                    if j < self.dim-1:
                        if self.array[i][j+1] == 0:
                            self.array[i][j+1] = negInt
                            negInt -= 1
                            self.numInts += 1
                if self.array[i][j] < -1:
                    if self.maxInt > -self.array[i][j]:
                        self.array[i][j] = -1

    # recFinder() is a recursive function to find unique arrangements
    def recFinder(self):

        # If final configuration is reached, print it
        if self.size == self.num_blocks:
            self.printX()

        # If incomplete, test out each potential option to place a new block
        else:
            optionsArray = []
            for i in range(self.dim):
                for j in range(self.dim):
                    if self.array[i][j] < -1:
                        optionsArray.append([i,j])
            for option in range(len(optionsArray)):
                newArray = Polyomino(self.num_blocks, self)
                (x,y) = optionsArray[option]

                # Invert negative integer to include it in the configuration
                newArray.array[x][y] *= -1
                newArray.size += 1

                # Set new largest integer in configuration as needed
                if newArray.array[x][y] > newArray.maxInt:
                    newArray.maxInt = newArray.array[x][y]
                newArray.refreshSpaces()
                newArray.recFinder()

    # printX() prints a minimalist version of each configuration
    def printX(self):
        for i in range(self.dim):
            for j in range(self.dim):
                print('X' if self.array[i][j] > 0 else ' ', end='')
            print('')
        
# Main function body (sys.argv[1] is an integer n in the executable "py PolyominoGenerator.py n")
if __name__ == "__main__":
    polynum = int(sys.argv[1])
    poly = Polyomino(polynum)
    poly.refreshSpaces()
    poly.recFinder()


