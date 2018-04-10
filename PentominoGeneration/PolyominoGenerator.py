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
# -1 to signify that this location cannot be used for future units (this
# is the basis for the lexicographic ordering).

# TODO:
# Minimize matrices before printing to remove unnecessary spaces.
# Avoid printing symmetrical or transformed configurations that
# are otherwise the same shape.
# Add 6- and 8-sided configurations. This will be tricky, but
# for hexagons at least, you can simply break them down to 
# equilateral triangles. Octagons may have to use more advanced
# graphics libraries(?) to check for overlap.

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
            self.max = 2*num_blocks-1       # Matrix dimensions

            # Clear matrix array and place first block in the center
            self.array = [[0 for i in range(self.max)] for j in range(self.max)]
            self.array[num_blocks-1][num_blocks-1] = 1

        else:
            self.num_blocks = copy.num_blocks
            self.maxInt = copy.maxInt
            self.numInts = copy.numInts
            self.size = copy.size
            self.max = copy.max

            # Initalize matrix array and use copy
            self.array = [[0 for i in range(self.max)] for j in range(self.max)]
            for i in range(self.max):
                for j in range(self.max):
                    self.array[i][j] = copy.array[i][j]

    # refreshSpaces() runs through the matrix and does the following:
    #   - replaces zeroes next to positive numbers with negative integers
    #   - for negative numbers whose magnitude is greater than the largest
    #       number in the current arrangement, replaces with -1
    def refreshSpaces(self):
        negInt = -self.numInts-1
        for i in range(self.max):
            for j in range(self.max):
                if self.array[i][j] > 0:
                    if i > 0:
                        if self.array[i-1][j] == 0:
                            self.array[i-1][j] = negInt
                            negInt -= 1
                            self.numInts += 1
                    if i < self.max-1:
                        if self.array[i+1][j] == 0:
                            self.array[i+1][j] = negInt
                            negInt -= 1
                            self.numInts += 1
                    if j > 0:
                        if self.array[i][j-1] == 0:
                            self.array[i][j-1] = negInt
                            negInt -= 1
                            self.numInts += 1
                    if j < self.max-1:
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
            for i in range(self.max):
                for j in range(self.max):
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
        for i in range(self.max):
            for j in range(self.max):
                print('X' if self.array[i][j] > 0 else ' ', end='')
            print('')
        
# Main function body (sys.argv[1] is an integer n in the executable "py PolyominoGenerator.py n")
if __name__ == "__main__":
    polynum = int(sys.argv[1])
    poly = Polyomino(polynum)
    poly.refreshSpaces()
    poly.recFinder()


